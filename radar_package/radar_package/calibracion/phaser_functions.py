#!/usr/bin/env python3
# Copyright (C) 2022 Analog Devices, Inc.
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     - Neither the name of Analog Devices, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#     - The use of this software may or may not infringe the patent rights
#       of one or more patent holders.  This license does not release you
#       from the requirement that you obtain separate licenses from these
#       patent holders to use this software.
#     - Use of the software either in source or binary form, must be run
#       on or directly connected to an Analog Devices Inc. component.
#
# THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED.
#
# IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, INTELLECTUAL PROPERTY
# RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
# THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Funciones de calibración del CN0566 "Phaser".
# Adaptadas de los ejemplos de pyadi-iio (examples/phaser) de Analog Devices:
#   - phaser_functions.py: calibración de canales, ganancia y fase
#   - phaser_find_hb100.py: búsqueda de la frecuencia de la fuente HB100
# Cambios respecto al original:
#   - los mensajes se envían a una función 'log' (logger de ROS) en vez de print
#   - phase_cal_sweep busca el máximo solo alrededor del tono (el original lo sobrescribía con el máximo global)
#   - la configuración del SDR recibe la ruta del filtro LTE20_MHz.ftr (el original la asumía en el directorio actual)
#   - signal_contrast_db: verificación de que la fuente está presente antes de calibrar

from time import sleep

import numpy as np
from numpy import absolute, log10, multiply
from numpy.fft import fft, fftfreq, fftshift
from scipy import signal

CAL_RX_GAIN_DB = 6  # ganancia Rx del PlutoSDR durante la calibración de canales/ganancia/fase (valor de ADI)
DC_GUARD_HZ = 0.5e6  # zona alrededor de DC que se ignora al buscar el tono (fuga del LO del PlutoSDR)
USABLE_BW_HZ = 19e6  # ancho de banda útil del filtro LTE20_MHz.ftr


def dc_guard_bins(sdr):
    """ Cantidad de bins de la FFT que cubre DC_GUARD_HZ a cada lado de DC """
    return int(np.ceil(DC_GUARD_HZ / (float(sdr.sample_rate) / int(sdr.rx_buffer_size))))


def _log(log, msg):
    if log is not None:
        log(msg)


def to_sup(angle):
    """ Return suplimentary angle if greater than 180 degrees. """
    if angle > 180.0:
        angle -= 360.0
    return angle


def spec_est(x, fs, ref=2 ** 15):
    """ Espectro en dB (ventana Kaiser) y su eje de frecuencias, ambos centrados con fftshift. """
    N = len(x)

    # Apply window
    window = signal.windows.kaiser(N, beta=38)
    window /= np.average(window)
    x = multiply(x, window)

    # Use FFT to get the amplitude of the spectrum
    ampl = 1 / N * fftshift(absolute(fft(x)))
    ampl = 20 * log10(ampl / ref + 10 ** -20)

    # FFT frequency bins
    freqs = fftshift(fftfreq(N, 1 / fs))

    # ampl and freqs for real data
    if not np.iscomplexobj(x):
        ampl = ampl[0: len(ampl) // 2]
        freqs = freqs[0: len(freqs) // 2]

    return ampl, freqs


# ----------------- configuración del SDR -----------------

def configure_sdr_cw(sdr, sample_rate, rx_lo, rx_gain, buffer_size, filter_path):
    """ Configuración común del PlutoSDR para recibir un tono continuo (sin rampa FMCW).
        Equivale a la parte de recepción de CN0566.SDR_init() de pyadi-iio. """
    sdr._ctrl.debug_attrs["adi,frequency-division-duplex-mode-enable"].value = "1"  # modo FDD
    sdr._ctrl.debug_attrs["adi,ensm-enable-txnrx-control-enable"].value = "0"  # SPI controla los estados
    sdr._ctrl.debug_attrs["initialize"].value = "1"

    sdr.rx_enabled_channels = [0, 1]  # Rx1 (voltage0) y Rx2 (voltage1)
    sdr.gain_control_mode_chan0 = "manual"  # sin AGC, de lo contrario no se ven los máximos y nulos
    sdr.gain_control_mode_chan1 = "manual"
    sdr._rxadc.set_kernel_buffers_count(1)  # sin búferes viejos: se mide justo después de cada cambio
    rx = sdr._ctrl.find_channel("voltage0")
    rx.attrs["quadrature_tracking_en"].value = "1"
    sdr.sample_rate = int(sample_rate)
    sdr.rx_lo = int(rx_lo)
    sdr.rx_buffer_size = int(buffer_size)
    sdr.rx_hardwaregain_chan0 = int(rx_gain)
    sdr.rx_hardwaregain_chan1 = int(rx_gain)
    # filtro LTE 20 MHz al final, igual que SDR_init: cambiar sample_rate después lo reemplazaría
    # (el archivo fija la tasa de muestreo en 30.72 MSPS)
    sdr.filter = filter_path


# ----------------- búsqueda de la HB100 -----------------

def find_hb100(cn0566, f_start, f_stop, f_step):
    """ Barre el PLL del phaser entre f_start y f_stop y une los espectros de cada banda.
        Retorna (frecuencias RF en Hz, amplitudes en dB, frecuencia del máximo en Hz).

        El LO del phaser queda por encima de la señal: el tono aparece en banda base en
        f_bb = f_paso - f_RF, por lo que f_RF = f_paso - f_bb (espectro invertido).
        De cada banda se usa DC_GUARD_HZ < |f_bb| < USABLE_BW_HZ/2: sin la fuga del LO en DC
        ni los bordes del filtro. Con f_step <= 9 MHz las bandas vecinas cubren esos huecos. """
    sdr = cn0566.sdr
    fs = float(sdr.sample_rate)
    full_freqs, full_ampl = [], []

    for freq in np.arange(f_start, f_stop, f_step):
        cn0566.SignalFreq = freq
        cn0566.frequency = (int(freq) + int(sdr.rx_lo)) // 4  # realimentación del PLL a través de VCO/4
        sleep(0.05)

        data = sdr.rx()
        data_sum = data[0] + data[1]
        ampl, f_bb = spec_est(data_sum, fs, ref=2 ** 12)
        keep = (np.abs(f_bb) > DC_GUARD_HZ) & (np.abs(f_bb) < USABLE_BW_HZ / 2)

        full_freqs.append(freq - f_bb[keep])
        full_ampl.append(ampl[keep])

    full_freqs = np.concatenate(full_freqs)
    full_ampl = np.concatenate(full_ampl)
    order = np.argsort(full_freqs)
    full_freqs, full_ampl = full_freqs[order], full_ampl[order]

    peak_freq = full_freqs[np.argmax(full_ampl)]
    return full_freqs, full_ampl, peak_freq


# ----------------- calibración de canales, ganancia y fase -----------------

def find_peak_bin(cn0566):
    """ Simple function to find the peak frequency bin of the incoming signal.
        sets nomial phases and gains first. Ignora la zona de DC (fuga del LO). """
    win = np.blackman(cn0566.sdr.rx_buffer_size)
    # First, locate fundamental.
    cn0566.set_all_gain(127)
    cn0566.set_beam_phase_diff(0.0)
    data = cn0566.sdr.rx()  # read a buffer of data
    y_sum = (data[0] + data[1]) * win
    s_sum = np.fft.fftshift(np.absolute(np.fft.fft(y_sum)))
    center, guard = len(s_sum) // 2, dc_guard_bins(cn0566.sdr)
    s_sum[center - guard: center + guard + 1] = 0
    return np.argmax(s_sum)


def signal_contrast_db(cn0566, peak_bin, width=10):
    """ Diferencia (dB) del tono entre todos los elementos encendidos y todos apagados.
        Si la fuente no está, el máximo del espectro es la fuga del LO del PlutoSDR,
        que no cambia al apagar los elementos del ADAR1000 (contraste cercano a 0 dB). """
    win = signal.windows.flattop(cn0566.sdr.rx_buffer_size)
    levels = []
    for gain in (127, 0):
        cn0566.set_all_gain(gain, apply_cal=False)
        sleep(1.0)  # todo - remove when driver fixed to compensate for ADAR1000 quirk
        cn0566.sdr.rx()  # descartar búfer viejo
        data = cn0566.sdr.rx()
        s_sum = np.fft.fftshift(np.absolute(np.fft.fft((data[0] + data[1]) * win)))
        levels.append(max(np.max(s_sum[peak_bin - width: peak_bin + width]), 1e-15))
    return 20 * np.log10(levels[0] / levels[1])


def channel_calibration(cn0566, log=None):
    """ Do this BEFORE gain_calibration.
        Performs calibration between the two ADAR1000 channels. Accounts for all
        sources of mismatch between the two channels: ADAR1000s, mixers, and
        the SDR (Pluto) inputs. Retorna el espectro medido en cada canal. """
    peak_bin = find_peak_bin(cn0566)
    channel_levels, plot_data = measure_channel_gains(cn0566, peak_bin)
    ch_mismatch = 20.0 * np.log10(channel_levels[0] / channel_levels[1])
    _log(log, f"Desbalance entre canales: {ch_mismatch:.2f} dB")
    if ch_mismatch > 0:  # Channel 0 higher, boost ch1:
        cn0566.ccal = [0.0, ch_mismatch]
    else:  # Channel 1 higher, boost ch0:
        cn0566.ccal = [-ch_mismatch, 0.0]
    return plot_data


def gain_calibration(cn0566, log=None):
    """ Perform the Gain Calibration routine.
        Mide cada elemento por separado (el resto con ganancia 0) y escala todos
        al elemento más débil. Retorna el espectro medido por elemento. """
    cn0566.gain_cal = True  # Gain Calibration Flag
    gcalibrated_values = []  # Intermediate cal values list
    plot_data = []
    peak_bin = find_peak_bin(cn0566)
    _log(log, f"Bin del tono: {peak_bin} de {cn0566.sdr.rx_buffer_size}")
    # gcal_element indicates current element/channel which is being calibrated
    for gcal_element in range(0, (cn0566.num_elements)):
        gcal_val, spectrum = measure_element_gain(cn0566, gcal_element, peak_bin)
        _log(log, f"Elemento {gcal_element}: nivel medido {gcal_val:.4g} (cuentas ADC)")
        gcalibrated_values.append(gcal_val)  # make a list of intermediate cal values
        plot_data.append(spectrum)

    # Minimum gain of intermediated cal val is set to Maximum value as we cannot go beyond max value and gain
    # of all other channels are set accordingly
    for k in range(0, 8):
        cn0566.gcal[k] = min(gcalibrated_values) / (gcalibrated_values[k])

    cn0566.gain_cal = False  # Reset the Gain calibration Flag once system gain is calibrated
    return plot_data


def measure_channel_gains(cn0566, peak_bin):
    """ Mide el nivel del tono en cada canal del PlutoSDR, encendiendo solo los 4 elementos de ese canal.
        parameters:
            peak_bin: type=int
                        Peak bin to examine around for amplitude
    """
    width = 10  # Bins around fundamental to sum
    win = signal.windows.flattop(cn0566.sdr.rx_buffer_size)
    win /= np.average(np.abs(win))  # Normalize to unity gain
    plot_data = []
    channel_level = []
    cn0566.set_rx_hardwaregain(CAL_RX_GAIN_DB, False)
    for channel in range(0, 2):
        # Start with sdr CH0 elements
        cn0566.set_all_gain(0, apply_cal=False)  # Start with all gains set to zero
        for element in range(0, 4):
            # 1-channel because wonky channel mapping!!
            cn0566.set_chan_gain((1 - channel) * 4 + element, 127, apply_cal=False)  # Set element to max

        sleep(1.0)  # todo - remove when driver fixed to compensate for ADAR1000 quirk
        total_sum = 0
        spectrum = np.zeros(cn0566.sdr.rx_buffer_size)

        for count in range(0, cn0566.Averages):  # repeat loop and average the results
            data = cn0566.sdr.rx()  # todo - remove once confirmed no flushing necessary
            data = cn0566.sdr.rx()  # read a buffer of data
            y_sum = (data[0] + data[1]) * win

            s_sum = np.fft.fftshift(np.absolute(np.fft.fft(y_sum)))
            spectrum += s_sum

            # Look for peak value within window around fundamental (reject interferers)
            s_mag_sum = np.max(s_sum[peak_bin - width: peak_bin + width])
            total_sum += s_mag_sum

        spectrum /= cn0566.Averages * cn0566.sdr.rx_buffer_size
        PeakValue_sum = total_sum / (cn0566.Averages * cn0566.sdr.rx_buffer_size)
        plot_data.append(spectrum)
        channel_level.append(PeakValue_sum)

    return channel_level, plot_data


def measure_element_gain(cn0566, cal, peak_bin):
    """ Mide el nivel del tono con un solo elemento encendido.
        parameters:
            cal: type=int
                        Elemento que se está midiendo
            peak_bin: type=int
                        Peak bin to examine around for amplitude
    """
    width = 10  # Bins around fundamental to sum
    cn0566.set_rx_hardwaregain(CAL_RX_GAIN_DB)  # Channel calibration defaults to True
    cn0566.set_all_gain(0, apply_cal=False)  # Start with all gains set to zero
    cn0566.set_chan_gain(cal, 127, apply_cal=False)  # Set element to max
    sleep(1.0)  # todo - remove when driver fixed to compensate for ADAR1000 quirk
    total_sum = 0
    win = signal.windows.flattop(cn0566.sdr.rx_buffer_size)
    win /= np.average(np.abs(win))  # Normalize to unity gain
    spectrum = np.zeros(cn0566.sdr.rx_buffer_size)

    for count in range(0, cn0566.Averages):  # repeat loop and average the results
        data = cn0566.sdr.rx()  # todo - remove once confirmed no flushing necessary
        data = cn0566.sdr.rx()  # read a buffer of data
        y_sum = (data[0] + data[1]) * win

        s_sum = np.fft.fftshift(np.absolute(np.fft.fft(y_sum)))
        spectrum += s_sum

        # Look for peak value within window around fundamental (reject interferers)
        s_mag_sum = np.max(s_sum[peak_bin - width: peak_bin + width])
        total_sum += s_mag_sum

    spectrum /= cn0566.Averages * cn0566.sdr.rx_buffer_size
    PeakValue_sum = total_sum / (cn0566.Averages * cn0566.sdr.rx_buffer_size)

    return PeakValue_sum, spectrum


def phase_cal_sweep(cn0566, peak_bin, ref=0, cal=1):
    """ Barre la fase del elemento 'cal' con respecto al elemento 'ref' (ambos con ganancia máxima calibrada).
        peak_bin: type=int
                    Which bin the fundamental is in.
                    This prevents detecting other spurs when deep in a null.
    """
    cn0566.set_all_gain(0)  # Reset all elements to zero
    cn0566.set_chan_gain(ref, 127, apply_cal=True)  # Set two adjacent elements to zero
    cn0566.set_chan_gain(cal, 127, apply_cal=True)
    sleep(1.0)

    cn0566.set_chan_phase(ref, 0.0, apply_cal=False)  # Reference element
    win = signal.windows.flattop(cn0566.sdr.rx_buffer_size)  # Super important!
    win /= np.average(np.abs(win))  # Normalize to unity gain
    width = 10  # Bins around fundamental to sum
    sweep_angle = 180
    # These are all the phase deltas (i.e. phase difference between Rx1 and Rx2, then Rx2 and Rx3, etc.) we'll sweep
    PhaseValues = np.arange(-(sweep_angle), (sweep_angle), cn0566.phase_step_size)

    gain = []  # Create empty lists
    for phase in PhaseValues:  # These sweeps phase value from -180 to 180
        # set Phase of channels based on Calibration Flag status and calibration element
        cn0566.set_chan_phase(cal, phase, apply_cal=False)
        total_sum = 0
        for count in range(0, cn0566.Averages):  # repeat loop and average the results
            data = cn0566.sdr.rx()  # read a buffer of data
            data = cn0566.sdr.rx()
            y_sum = (data[0] + data[1]) * win
            s_sum = np.fft.fftshift(np.absolute(np.fft.fft(y_sum)))

            # use flat-top window, look for peak (solo alrededor del tono: en el nulo el máximo global puede ser un espurio)
            s_mag_sum = np.max(s_sum[peak_bin - width: peak_bin + width])
            total_sum += s_mag_sum
        PeakValue_sum = total_sum / (cn0566.Averages * cn0566.sdr.rx_buffer_size)
        gain.append(PeakValue_sum)

    return PhaseValues, gain


def phase_calibration(cn0566, log=None):
    """ Perform the Phase Calibration routine.
        Para cada par de elementos adyacentes busca el nulo de la suma; el desfase que lo
        produce entrega la corrección de fase. Depende de la calibración de ganancia (gcal).
        Retorna (fases barridas, nivel medido para cada par). """
    peak_bin = find_peak_bin(cn0566)
    _log(log, f"Bin del tono: {peak_bin} de {cn0566.sdr.rx_buffer_size}")

    cn0566.pcal = [0, 0, 0, 0, 0, 0, 0, 0]
    cn0566.ph_deltas = [0, 0, 0, 0, 0, 0, 0]
    plot_data = []
    # cal_element indicates current element/channel which is being calibrated
    # As there are 8 channels and we take two adjacent chans for calibration we have 7 cal_elements
    for cal_element in range(0, 7):
        PhaseValues, gain = phase_cal_sweep(cn0566, peak_bin, cal_element, cal_element + 1)

        null_phase = PhaseValues[gain.index(min(gain))]
        ph_delta = to_sup((180 - null_phase) % 360.0)
        _log(log, f"Elementos {cal_element}-{cal_element + 1}: nulo en {null_phase:.2f}°, corrección {ph_delta:.2f}°")
        cn0566.ph_deltas[cal_element] = ph_delta

        cn0566.pcal[cal_element + 1] = to_sup((cn0566.pcal[cal_element] - ph_delta) % 360.0)
        plot_data.append(gain)
    return PhaseValues, plot_data
