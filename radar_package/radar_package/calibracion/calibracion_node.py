#!/usr/bin/env python3
# ejecutable con interprete python3
"""
Calibración inicial del Phaser CN0566.
Nodo de uso puntual: ejecuta la calibración, guarda los archivos y se cierra.

Procedimiento de Analog Devices (phaser_find_hb100.py y phaser_examples.py cal):
https://wiki.analog.com/resources/eval/user-guides/circuits-from-the-lab/cn0566/quickstart

Parámetros
Pasos, siempre en este orden:
  - hb100: barrido del PLL para encontrar la frecuencia exacta de la HB100 -> hb100_freq_val.pkl
  - channel: desbalance de ganancia entre los dos canales Rx del PlutoSDR (dB) -> channel_cal_val.pkl
  - gain: ganancia relativa de cada uno de los 8 elementos -> gain_cal_val.pkl
  - phase: corrección de fase de cada elemento (grados) -> phase_cal_val.pkl
Si estos pasos son omitidos, se usan los archivos de una calibración anterior. Si no existen se deja sin corrección.

Montaje: HB100 encendida frente al arreglo, en el eje mecánico (boresight), a ~1.5 m.
No ejecutar junto con radar_node: ambos configuran el mismo hardware.
"""
import sys # interprete de python
import time # control temporal
import pickle # archivos de calibracion
from pathlib import Path # manejar rutas de archivos

import rclpy # ros2
from rclpy.node import Node # clase nodo de ros2
import adi # libreria de analog devices
import numpy as np # calculo matematico
from ament_index_python.packages import get_package_share_directory # archivos de recursos

from radar_package.calibracion import phaser_functions as pf # funciones de calibracion de ADI

# Parametros constantes, los parametros de config entran a definir cuáles de esos pasos quieres ejecutar esta vez
PKG = 'radar_package' # fuente del paquete
STEPS = ('hb100', 'channel', 'gain', 'phase') # orden de ejecucion
SOURCES = ('hb100', 'tx') # fuentes de señal admitidas
# mismos nombres que cargan radar_node y las funciones load_*_cal() de pyadi-iio
FILES = {
    'hb100': 'hb100_freq_val.pkl',
    'channel': 'channel_cal_val.pkl',
    'gain': 'gain_cal_val.pkl',
    'phase': 'phase_cal_val.pkl',
} # archivos de guardado

# parametros fijos tomados desde el ejemplo de ADI
HB100_RX_GAIN_DB = 0 # ganancia Rx del PlutoSDR durante el barrido (phaser_find_hb100.py) - rx_hardwaregain_chan0 y rx_hardwaregain_chan1
HB100_ELEMENT_GAIN = 64 # ganancia de cada elemento durante el barrido (phaser_find_hb100.py) - set_chan_gain


class CalibracionAbortada(Exception):
    """Detiene la calibración sin guardar el paso en curso"""
    pass


def source_resource_dir():
    """
    Carpeta resource/ del código fuente de radar_package, deducida desde la ruta de instalación
    (<ws>/install/radar_package/share/radar_package, o <ws>/install/share/radar_package con --merge-install -> flag para instalar todos los paquetes del espacio de trabajo en una única estructura de carpetas compartida).
    El workspace es la raíz del repositorio (UC_SmartFarmRadar), con los paquetes directamente en ella y sin src/:
    <ws>/radar_package/resource. Ahí están los .pkl que se instalan con colcon build y que carga radar_node.
    None si no se encuentra.
    """
    share = Path(get_package_share_directory(PKG)).resolve()
    ws = next((p.parent for p in share.parents if p.name == 'install'), None)
    if ws is None:
        return None
    pkg_dir = ws / PKG # paquete en la raíz del repositorio
    # comprueba que sea la raiz por medio del .xml y verifica que exista resource
    if (pkg_dir / 'package.xml').is_file() and (pkg_dir / 'resource').is_dir():
        return pkg_dir / 'resource'
    return None


class CalibracionNode(Node):
    def __init__(self):
        super().__init__('calibracion_node')

        # parámetros compartidos con radar_node
        # radar.yaml
        self.declare_parameter('sdr_uri', "ip:phaser.local:50901")
        self.declare_parameter('phaser_uri', "ip:phaser.local")
        self.declare_parameter('element_spacing_m', 0.014)
        # parámetros de calibración
        # calibracion.yaml
        self.declare_parameter('cal_steps', list(STEPS))
        self.declare_parameter('cal_signal_source', 'hb100')
        self.declare_parameter('cal_default_signal_freq_hz', 10.525e9)
        self.declare_parameter('cal_sample_rate_hz', 30.0e6)
        self.declare_parameter('cal_rx_lo_hz', 2.2e9)
        self.declare_parameter('cal_tone_offset_hz', 1.0e6)
        self.declare_parameter('cal_buffer_size', 1024)
        self.declare_parameter('cal_tx_gain_db', -6)
        self.declare_parameter('cal_averages', 4)
        self.declare_parameter('cal_min_signal_contrast_db', 15.0)
        self.declare_parameter('hb100_sweep_start_hz', 10.0e9)
        self.declare_parameter('hb100_sweep_stop_hz', 10.7e9)
        self.declare_parameter('hb100_sweep_step_hz', 5.0e6)
        self.declare_parameter('hb100_min_snr_db', 15.0)
        self.declare_parameter('cal_output_dir', "")
        self.declare_parameter('cal_save_plots', True)
        self.declare_parameter('cal_start_delay_s', 10.0)
        self.declare_parameter('connect_retries', 3)
        self.declare_parameter('retry_interval_s', 5.0)

        # carga de valores
        p = self.get_parameter
        self.sdr_uri = p('sdr_uri').value
        self.phaser_uri = p('phaser_uri').value
        self.element_spacing_m = p('element_spacing_m').value
        self.steps = [s for s in STEPS if s in p('cal_steps').value] # se respeta el orden de STEPS
        self.signal_source = p('cal_signal_source').value
        self.default_signal_freq_hz = p('cal_default_signal_freq_hz').value
        self.sample_rate_hz = p('cal_sample_rate_hz').value
        self.rx_lo_hz = p('cal_rx_lo_hz').value
        self.tone_offset_hz = p('cal_tone_offset_hz').value
        self.buffer_size = p('cal_buffer_size').value
        self.tx_gain_db = p('cal_tx_gain_db').value
        self.averages = p('cal_averages').value
        self.min_contrast_db = p('cal_min_signal_contrast_db').value
        self.hb100_start_hz = p('hb100_sweep_start_hz').value
        self.hb100_stop_hz = p('hb100_sweep_stop_hz').value
        self.hb100_step_hz = p('hb100_sweep_step_hz').value
        self.hb100_min_snr_db = p('hb100_min_snr_db').value
        self.save_plots = p('cal_save_plots').value
        self.start_delay_s = p('cal_start_delay_s').value
        self.connect_retries = p('connect_retries').value
        self.retry_interval_s = p('retry_interval_s').value

        unknown = set(p('cal_steps').value) - set(STEPS)
        if unknown:
            raise ValueError(f"Pasos desconocidos en cal_steps: {sorted(unknown)}. Opciones: {list(STEPS)}")
        if self.signal_source not in SOURCES:
            raise ValueError(f"cal_signal_source debe ser uno de {list(SOURCES)}")
        # el tono debe quedar fuera de la zona de DC y dentro del filtro
        if not 1.2 * pf.DC_GUARD_HZ <= abs(self.tone_offset_hz) <= pf.USABLE_BW_HZ / 2 - 1e6:
            raise ValueError(f"cal_tone_offset_hz debe estar entre {1.2 * pf.DC_GUARD_HZ:.0f} y "
                             f"{pf.USABLE_BW_HZ / 2 - 1e6:.0f} Hz")
        # bandas vecinas del barrido deben cubrir la zona de DC de cada banda
        if self.hb100_step_hz > pf.USABLE_BW_HZ / 2 - pf.DC_GUARD_HZ:
            raise ValueError(f"hb100_sweep_step_hz debe ser <= {pf.USABLE_BW_HZ / 2 - pf.DC_GUARD_HZ:.0f} Hz")

        # filtro LTE 20 MHz requerido por la configuración del SDR
        self.filter_path = Path(get_package_share_directory(PKG)) / 'resource' / 'LTE20_MHz.ftr'

        # carpeta de resultados
        output_dir = p('cal_output_dir').value
        if output_dir:
            self.output_dir = Path(output_dir).expanduser()
        else:
            self.output_dir = source_resource_dir()
            if self.output_dir is None:
                self.output_dir = Path.home() / '.ros' / 'radar_calibracion'
                self.get_logger().warn(
                    f"No se encontró resource/ del código fuente: los resultados quedan en {self.output_dir}. "
                    "Cópielos a radar_package/resource para que radar_node los use.")
        self.plots_dir = self.output_dir / 'calibracion_graficos'
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.sdr = None
        self.phaser = None
        self.initial_rf_bw = None # (rx_rf_bandwidth, tx_rf_bandwidth) antes de calibrar
        self.signal_freq_hz = None # frecuencia de la señal de calibración (medida o cargada)
        self.saved = [] # archivos escritos en esta ejecución

        self.get_logger().info(
            f"Pasos: {self.steps} | fuente: {self.signal_source} | resultados en: {self.output_dir}")

    # ----------------- utilidades -----------------

    def path(self, step):
        return self.output_dir / FILES[step]

    def save(self, step, values):
        """Guarda un resultado en el mismo formato pickle que usa pyadi-iio"""
        path = self.path(step)
        with open(path, 'wb') as f:
            pickle.dump(values, f)
        self.saved.append(path)
        self.get_logger().info(f"[{step}] Guardado en {path}")

    def save_plot(self, name, title, xlabel, ylabel, curves, marker_x=None):
        """Guarda un gráfico PNG (sin ventanas: funciona por SSH). curves: lista de (x, y, etiqueta)"""
        if not self.save_plots:
            return
        import matplotlib
        matplotlib.use('Agg') # backend sin pantalla
        import matplotlib.pyplot as plt

        self.plots_dir.mkdir(parents=True, exist_ok=True)
        fig, ax = plt.subplots(figsize=(9, 5))
        for x, y, label in curves:
            ax.plot(x, y, label=label)
        if marker_x is not None:
            ax.axvline(marker_x, color='red', linestyle='--', linewidth=1)
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        if len(curves) > 1:
            ax.legend(fontsize='small')
        path = self.plots_dir / f'{name}.png'
        fig.savefig(path, dpi=120, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f"Gráfico guardado en {path}")

    @staticmethod
    def to_db(spectrum):
        return 20 * np.log10(np.maximum(np.asarray(spectrum, dtype=float), 1e-15))

    # ----------------- hardware -----------------

    def connect(self):
        for attempt in range(1, self.connect_retries + 2):
            try:
                self.get_logger().info(f"Conectando con el phaser ({self.phaser_uri}) y el PlutoSDR ({self.sdr_uri})")
                self.sdr = adi.ad9361(uri=self.sdr_uri)
                self.phaser = adi.CN0566(uri=self.phaser_uri, sdr=self.sdr)
                self.get_logger().info("Hardware conectado")
                # LTE20_MHz.ftr cambia también el filtro analógico (líneas BWRX/BWTX): se restaura al terminar
                self.initial_rf_bw = (int(self.sdr.rx_rf_bandwidth), int(self.sdr.tx_rf_bandwidth))
                return
            except Exception as e:
                self.get_logger().error(f"Error de conexión (intento {attempt}): {e}")
                if attempt > self.connect_retries:
                    break
                time.sleep(self.retry_interval_s)
        raise CalibracionAbortada("No se pudo conectar con el phaser")

    def release_hardware(self):
        """Deja el transmisor apagado (con fuente 'tx' el DDS queda emitiendo) y restaura el filtro analógico"""
        if self.sdr is None:
            return
        try:
            self.sdr.tx_hardwaregain_chan0 = -88
            self.sdr.tx_hardwaregain_chan1 = -88
            self.sdr.disable_dds()
        except Exception as e:
            self.get_logger().warn(f"No se pudo apagar el transmisor: {e}")
        if self.initial_rf_bw is not None:
            try:
                self.sdr.rx_rf_bandwidth, self.sdr.tx_rf_bandwidth = self.initial_rf_bw
                self.get_logger().info(
                    f"Filtro analógico restaurado: Rx {self.sdr.rx_rf_bandwidth} Hz, Tx {self.sdr.tx_rf_bandwidth} Hz")
            except Exception as e:
                self.get_logger().warn(f"No se pudo restaurar el filtro analógico: {e}")

    def wait_for_operator(self):
        if self.signal_source == 'hb100':
            msg = "Ubique la HB100 encendida frente al arreglo, en el eje mecánico (boresight), a ~1.5 m"
        else:
            msg = "Conecte la antena a la salida TX del phaser y ubíquela frente al arreglo, en el eje mecánico"
        if sys.stdin is not None and sys.stdin.isatty(): # ros2 run: terminal interactiva
            input(f"{msg}. Presione Enter para comenzar...")
        else: # ros2 launch: sin entrada estándar
            self.get_logger().info(f"{msg}. Comenzando en {self.start_delay_s:.0f} s...")
            time.sleep(self.start_delay_s)

    def resolve_signal_freq(self):
        """Frecuencia de la señal de calibración: medida en esta ejecución, guardada o por defecto"""
        if self.signal_freq_hz is not None:
            return self.signal_freq_hz
        if self.signal_source == 'hb100' and self.path('hb100').is_file():
            with open(self.path('hb100'), 'rb') as f:
                freq = float(pickle.load(f))
            self.get_logger().info(f"Usando frecuencia de la HB100 guardada: {freq / 1e9:.6f} GHz")
            return freq
        if self.signal_source == 'hb100':
            self.get_logger().warn(
                f"No hay medición de la HB100: se usa {self.default_signal_freq_hz / 1e9:.3f} GHz. "
                "Incluya el paso 'hb100' para medirla.")
        return self.default_signal_freq_hz

    def configure_cw(self):
        """Recepción de onda continua para calibrar (phaser_examples.py), con el PLL sin rampa"""
        ph, sdr = self.phaser, self.sdr
        self.signal_freq_hz = self.resolve_signal_freq()

        ph.configure(device_mode="rx") # modo receptor
        ph.element_spacing = self.element_spacing_m # espaciado de elementos
        pf.configure_sdr_cw(sdr, self.sample_rate_hz, self.rx_lo_hz, pf.CAL_RX_GAIN_DB,
                            self.buffer_size, str(self.filter_path))

        # calibraciones anteriores: las usan los pasos que dependen de otros (si no existen: sin corrección)
        ph.load_channel_cal(str(self.path('channel')))
        ph.load_gain_cal(str(self.path('gain')))
        ph.load_phase_cal(str(self.path('phase')))
        ph.set_rx_hardwaregain(pf.CAL_RX_GAIN_DB) # con compensación entre canales

        if self.signal_source == 'tx':
            # tono de 2 MHz por Tx2, el phaser lo sube a la frecuencia de calibración
            sdr.tx_lo = int(self.rx_lo_hz)
            sdr.tx_cyclic_buffer = True
            sdr.tx_hardwaregain_chan0 = -88 # Tx1 apagado
            sdr.tx_hardwaregain_chan1 = int(self.tx_gain_db)
            sdr.dds_single_tone(int(2e6), 0.9, 1) # (frecuencia, escala 0 a 1, canal Tx)
        else:
            # transmisor apagado: la señal viene de la HB100
            sdr.tx_hardwaregain_chan0 = -88
            sdr.tx_hardwaregain_chan1 = -88
            sdr.tx_lo = int(1.0e9) # lejos de la frecuencia de recepción
            sdr.disable_dds()

        # PLL ADF4159 en frecuencia fija (sin rampa FMCW)
        # el desplazamiento deja el tono de la HB100 en +cal_tone_offset_hz de banda base y no en DC,
        # donde está la fuga del LO del PlutoSDR (el script de ADI lo deja en DC)
        ph.SignalFreq = self.signal_freq_hz
        ph.frequency = (int(ph.SignalFreq) + int(self.rx_lo_hz) + int(self.tone_offset_hz)) // 4 # realimentación del PLL a través de VCO/4
        ph.freq_dev_step = 5690
        ph.freq_dev_range = 0
        ph.freq_dev_time = 0
        ph.powerdown = 0
        ph.ramp_mode = "disabled"

        ph.Averages = self.averages # promedios por medición
        for i in range(8):
            ph.set_chan_gain(i, 127, apply_cal=True)
        ph.set_beam_phase_diff(0.0) # haz al frente
        self.get_logger().info(f"Recepción configurada en {self.signal_freq_hz / 1e9:.6f} GHz")

    def check_signal(self):
        """Sin una señal clara la calibración no sirve: se verifica antes de medir"""
        peak_bin = pf.find_peak_bin(self.phaser)
        contrast = pf.signal_contrast_db(self.phaser, peak_bin)
        self.get_logger().info(f"Tono en el bin {peak_bin}: cae {contrast:.1f} dB al apagar todos los elementos")
        if contrast < self.min_contrast_db:
            raise CalibracionAbortada(
                f"No se detecta la señal de calibración (contraste {contrast:.1f} dB < {self.min_contrast_db} dB). "
                "Verifique que la fuente esté encendida y apuntando al arreglo.")

    # ----------------- pasos -----------------

    def step_hb100(self):
        log = self.get_logger()
        ph, sdr = self.phaser, self.sdr
        log.info(f"[hb100] Barrido de {self.hb100_start_hz / 1e9:.3f} a {self.hb100_stop_hz / 1e9:.3f} GHz "
                 f"en pasos de {self.hb100_step_hz / 1e6:.0f} MHz")

        ph.configure(device_mode="rx")
        pf.configure_sdr_cw(sdr, self.sample_rate_hz, self.rx_lo_hz, HB100_RX_GAIN_DB,
                            self.buffer_size, str(self.filter_path))
        sdr.tx_hardwaregain_chan0 = -88 # transmisor atenuado
        sdr.tx_hardwaregain_chan1 = -88
        for i in range(8):
            ph.set_chan_gain(i, HB100_ELEMENT_GAIN, apply_cal=False)
        ph.set_beam_phase_diff(0.0) # HB100 al frente del arreglo

        freqs, ampl, peak_freq = pf.find_hb100(ph, self.hb100_start_hz, self.hb100_stop_hz, self.hb100_step_hz)
        peak_db = np.max(ampl)
        snr_db = peak_db - np.median(ampl)
        log.info(f"[hb100] Máximo en {peak_freq / 1e9:.6f} GHz, {snr_db:.1f} dB sobre la mediana del espectro")

        # otro máximo fuerte lejos del principal: reflejos u otra fuente cerca
        far = np.abs(freqs - peak_freq) > 1e6
        if np.any(far) and peak_db - np.max(ampl[far]) < 6.0:
            other = freqs[far][np.argmax(ampl[far])]
            log.warn(f"[hb100] Hay otro máximo similar en {other / 1e9:.6f} GHz: "
                     "reubique la HB100 y revise que no haya otras fuentes cerca")

        self.save_plot('hb100_espectro', f"Espectro completo, máximo en {peak_freq / 1e9:.6f} GHz",
                       "Frecuencia [GHz]", "Nivel [dB]", [(freqs / 1e9, ampl, 'espectro')], marker_x=peak_freq / 1e9)

        if snr_db < self.hb100_min_snr_db:
            raise CalibracionAbortada(
                f"Tono de la HB100 débil ({snr_db:.1f} dB < {self.hb100_min_snr_db} dB). "
                "Verifique que esté encendida, apuntando al arreglo y sin otras fuentes cerca.")

        # el máximo debe entrar por el arreglo: al apagar los elementos tiene que caer (descarta espurios del SDR)
        ph.frequency = (int(peak_freq) + int(self.rx_lo_hz) + int(self.tone_offset_hz)) // 4
        contrast = pf.signal_contrast_db(ph, pf.find_peak_bin(ph))
        log.info(f"[hb100] El máximo cae {contrast:.1f} dB al apagar todos los elementos")
        if contrast < self.min_contrast_db:
            raise CalibracionAbortada(
                f"El máximo no viene del arreglo (contraste {contrast:.1f} dB < {self.min_contrast_db} dB). "
                "Verifique que la HB100 esté encendida y apuntando al arreglo.")

        self.signal_freq_hz = float(peak_freq)
        self.save('hb100', self.signal_freq_hz)

    def step_channel(self):
        ph = self.phaser
        self.get_logger().info("[channel] Calibrando desbalance entre los canales Rx del PlutoSDR")
        ph.set_beam_phase_diff(0.0)
        spectra = pf.channel_calibration(ph, log=self.get_logger().info)
        ph.ccal = [float(v) for v in ph.ccal]
        self.save('channel', ph.ccal)
        ph.set_rx_hardwaregain(pf.CAL_RX_GAIN_DB) # aplicar la nueva compensación en los pasos siguientes

        bins = np.arange(len(spectra[0]))
        self.save_plot('canales_fft', "Calibración de canales: FFT de cada canal Rx", "Bin FFT", "Nivel [dB]",
                       [(bins, self.to_db(s), f'canal {i}') for i, s in enumerate(spectra)])

    def step_gain(self):
        ph = self.phaser
        self.get_logger().info("[gain] Calibrando ganancia de los 8 elementos")
        ph.set_beam_phase_diff(0.0)
        spectra = pf.gain_calibration(ph, log=self.get_logger().info)
        ph.gcal = [float(v) for v in ph.gcal]
        self.save('gain', ph.gcal)

        bins = np.arange(len(spectra[0]))
        self.save_plot('ganancia_fft', "Calibración de ganancia: FFT de cada elemento", "Bin FFT", "Nivel [dB]",
                       [(bins, self.to_db(s), f'elemento {i}') for i, s in enumerate(spectra)])

    def step_phase(self):
        ph = self.phaser
        self.get_logger().info("[phase] Calibrando fase entre elementos adyacentes")
        phases, sweeps = pf.phase_calibration(ph, log=self.get_logger().info)
        ph.pcal = [float(v) for v in ph.pcal]
        self.save('phase', ph.pcal)

        self.save_plot('fase_barridos', "Calibración de fase: barrido de pares adyacentes",
                       "Diferencia de fase [°]", "Nivel [dB]",
                       [(phases, self.to_db(g), f'elementos {i}-{i + 1}') for i, g in enumerate(sweeps)])

    # ----------------- rutina principal -----------------

    def run(self):
        self.connect()
        self.wait_for_operator()

        if 'hb100' in self.steps:
            if self.signal_source == 'tx':
                self.get_logger().warn("Paso 'hb100' omitido: con fuente 'tx' se usa cal_default_signal_freq_hz")
            else:
                self.step_hb100()

        cw_steps = [s for s in ('channel', 'gain', 'phase') if s in self.steps]
        if cw_steps:
            self.configure_cw()
            self.check_signal()
            for step in cw_steps:
                getattr(self, f'step_{step}')()

        self.summary()

    def summary(self):
        log = self.get_logger()
        ph = self.phaser
        log.info("=============== Calibración terminada ===============")
        if self.signal_freq_hz is not None:
            log.info(f"Frecuencia de calibración: {self.signal_freq_hz / 1e9:.6f} GHz")
        if 'channel' in self.steps:
            log.info(f"Canales (ccal, dB): {np.round(ph.ccal, 2).tolist()}")
        if 'gain' in self.steps:
            log.info(f"Ganancia (gcal): {np.round(ph.gcal, 3).tolist()}")
        if 'phase' in self.steps:
            log.info(f"Fase (pcal, °): {np.round(ph.pcal, 2).tolist()}")
        for path in self.saved:
            log.info(f"Archivo: {path}")
        log.info("Para que radar_node use esta calibración: colcon build --packages-select radar_package")


def main(args=None):
    rclpy.init(args=args)
    node = CalibracionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Calibración interrumpida")
    except CalibracionAbortada as e:
        node.get_logger().error(f"Calibración abortada: {e}")
    finally:
        node.release_hardware()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
