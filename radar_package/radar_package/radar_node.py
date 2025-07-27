#!/usr/bin/env python3
# ejecutable con interprete python3
import rclpy # ros2 
from rclpy.node import Node
import adi
import time
import numpy as np
# mensajes de ros
#from std_msgs.msg import Float32MultiArray
from radar_msg.msg import RadarData
from std_msgs.msg import Bool
from std_msgs.msg import Header
import os
from ament_index_python.packages import get_package_share_directory
from radar_package.parametros import *

pkg_share = get_package_share_directory('radar_package')
path_gain = os.path.join(pkg_share, 'resource', 'gain_cal_val.pkl')
path_phase = os.path.join(pkg_share, 'resource', 'phase_cal_val.pkl')

# herencia de node, al instanciar se registra en el grafo de ROS2
class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_node')
        print("iniciando")

        # direcciones ip
        #self.sdr_uri_raspberry = "ip:192.168.2.1"
        #self.phaser_uri_raspberry = "ip:localhost"

        # windows
        self.sdr_uri_computerhost = "ip:phaser.local:50901"
        self.phaser_uri_computerhost = "ip:phaser.local"

        # ubuntu
        #self.sdr_uri_computerhost = "ip:10.42.0.1:50901"
        #self.phaser_uri_computerhost = "ip:10.42.0.1"


        # parámetros configurables desde línea de comandos o launch 
        self.declare_parameter('angle_min', -80) # grados
        self.declare_parameter('angle_max', 80) # grados
        self.declare_parameter('angle_step', 1) # grados

        # Leer parámetros
        p = self.get_parameter
        self.angle_min = p('angle_min').get_parameter_value().integer_value
        self.angle_max = p('angle_max').get_parameter_value().integer_value
        self.angle_step  = p('angle_step').get_parameter_value().integer_value

        # Inicializar hardware
        self._init_hardware()

        # --- Publisher ---
        # clase de mensaje, nombre del topico, tamaño de buffer
        self.pub_matrix = self.create_publisher(RadarData, 'radar_data', 10)
        self.sub_trigger = self.create_subscription(Bool, 'trigger_sweep', self.trigger_callback, 10)

        self.ready_for_trigger = True

    def _init_hardware(self):
        try:
            #sdr = adi.ad9361(uri=self.sdr_uri_raspberry)
            #phaser = adi.CN0566(uri=self.phaser_uri_raspberry, sdr=sdr)
            sdr = adi.ad9361(uri=self.sdr_uri_computerhost)
            phaser = adi.CN0566(uri=self.phaser_uri_computerhost, sdr=sdr)
        except Exception as e:
            self.get_logger().error(f'Error al conectar dispositivos: {e}')
            rclpy.shutdown() # apaga el nodo
            return

        # Configuración del Phaser: ADAR1000: gananacia y fase
        # IMPORTANTE: se debe tener los archivos de calibracion en la misma carpeta
        phaser.configure(device_mode="rx")
        phaser.load_gain_cal(path_gain)
        phaser.load_phase_cal(path_phase)
        print("calibrado")
        for ch in range(8):
            phaser.set_chan_phase(ch, 0)

        gains = [6, 27, 66, 100, 100, 66, 27, 6]
        for i, g in enumerate(gains):
            phaser.set_chan_gain(i, g, apply_cal=True)

        # configuración de los pines GPIO de la Raspberry Pi
        try:
            phaser._gpios.gpio_tx_sw = 0 # 0 = TX_OUT_2, 1 = TX_OUT_1
            phaser._gpios.gpio_vctrl_1 = 1
            phaser._gpios.gpio_vctrl_2 = 1 
        except:
            phaser.gpios.gpio_tx_sw = 0 # 0 = TX_OUT_2, 1 = TX_OUT_1
            phaser.gpios.gpio_vctrl_1 = 1 
            phaser.gpios.gpio_vctrl_2 = 1

        # Parámetros de configuración de la señal en sdr
        sample_rate = sample_rate # frecuencia de muestreo
        center_freq = center_freq # 2.2 frecuencia central
        signal_freq = signal_freq # frecuencia de la señal transmitida    
        fft_size = fft_size 

        # Configuración del receptor Rx del SDR
        sdr.sample_rate = int(sample_rate)
        sdr.rx_lo = int(center_freq)  # total = output_freq - (la frecuencia del HB100)
        sdr.rx_enabled_channels = [0, 1] # Habilita los canales de recepción: Canal 0 (Rx1 / voltage0) - Canal 1 (Rx2 / voltage1)
        sdr.rx_buffer_size = int(fft_size)
        sdr.gain_control_mode_chan0 = "manual" # manual o slow_attack
        sdr.gain_control_mode_chan1 = "manual" # manual o slow_attack
        sdr.rx_hardwaregain_chan0 = int(30) # -3 and 70
        sdr.rx_hardwaregain_chan1 = int(30) # -3 and 70

        # Configuración del transmisor Tx del SDR
        sdr.tx_lo = int(center_freq)
        sdr.tx_enabled_channels = [0, 1]
        sdr.tx_cyclic_buffer = True 
        sdr.tx_hardwaregain_chan0 = -88 # 0 and -88
        sdr.tx_hardwaregain_chan1 = -0 # 0 and -88

        # Configuración del PLL ADF4159 (Phase-Locked Loop) en el phaser
        output_freq = 10e9
        BW = 500e6 
        num_steps = 500
        ramp_time = 0.5e3 # microsegundos

        phaser.frequency = int(output_freq + sdr.rx_lo) // 4
        phaser.freq_dev_range = int(BW / 4)
        phaser.freq_dev_step = int((BW/4) / num_steps) 
        phaser.freq_dev_time = int(ramp_time)
        ramp_time_s = ramp_time / 1e6

        phaser.delay_word = 4095  # Palabra de retardo de 12 bits. 4095 * PFD = 40.95 us.
        # PFD Phase Frequency Detector - Detector de Fase y Frecuencia -> ajuste de frecuencia de oscilacion y sincronizacion
        # Para rampas en diente de sierra (sawtooth), este valor también define la duración de la señal Ramp_complete.
        phaser.delay_clk = "PFD" # Reloj de referencia para el retardo. Puede ser 'PFD' (Phase Frequency Detector) o 'PFD*CLK1' (multiplicado por un factor adicional de tiempo).
        phaser.delay_start_en = 0 # Habilitación del retardo de inicio. 0 = deshabilitado, 1 = habilitado
        phaser.ramp_delay_en = 0 # Habilitación del retardo entre rampas. 0 = sin retardo, 1 = introduce un retardo entre cada rampa
        phaser.trig_delay_en = 0 # Habilitación del retardo en el disparo de la señal (trigger delay). 0 = deshabilitado, 1 = habilitado
        phaser.ramp_mode = "continuous_triangular" # puede ser: "disabled", "continuous_sawtooth", "continuous_triangular", "single_sawtooth_burst", "single_ramp_burst"
        phaser.sing_ful_tri = 0 # deshabilitar/habilitar triángulo completo: esto se utiliza con el modo single_ramp_burst
        phaser.tx_trig_en = 0 # iniciar una rampa con TXdata
        phaser.enable = 0 # 0 = PLL habilitado, actualiza todos los registros -> colocar al final

        # Generación de una señal senoidal
        fs = int(sdr.sample_rate)
        N = int(sdr.rx_buffer_size)
        fc = int(signal_freq / (fs / N)) * (fs / N)
        ts = 1 / float(fs)
        t = np.arange(0, N * ts, ts)
        i = np.cos(2 * np.pi * t * fc) * 2 ** 14
        q = np.sin(2 * np.pi * t * fc) * 2 ** 14
        iq = 1 * (i + 1j * q)

        # ENVÍO DE DATOS AL SDR
        sdr._ctx.set_timeout(0)
        # canal Tx 2
        sdr.tx([iq * 0, iq])


        # Guardar instancias
        self.my_sdr    = sdr
        self.my_phaser = phaser

    def trigger_callback(self, msg):
        if msg.data and self.ready_for_trigger:
            self.get_logger().info("Trigger TRUE recibido → ejecutando barrido.")
            self.ready_for_trigger = False # bloquea hasta recibir False
            self._do_sweep()
        elif not msg.data:
            self.ready_for_trigger = True
            self.get_logger().info("Trigger FALSE recibido → barrido habilitado de nuevo.")
        else:
            self.get_logger().info("Trigger TRUE ignorado → ya se ejecutó, espera un FALSE para rearmar.")

    def _do_sweep(self):
        angles = np.arange(self.angle_min, self.angle_max+1, self.angle_step)
        radar_data_matriz = []

        for theta in angles:
            # 1) Beam steering
            phase_delta = (2*np.pi * 10.25e9 * 0.014 *np.sin(np.radians(theta))) / 3e8
            self.my_phaser.set_beam_phase_diff(np.degrees(phase_delta))
            time.sleep(0.1)

            # 2) Recepción y FFT
            data = self.my_sdr.rx()
            sig  = data[0] + data[1]
            win  = np.blackman(len(sig))
            sp   = np.fft.fftshift(np.abs(np.fft.fft(sig * win)))
            mag  = np.abs(sp) / np.sum(win)
            mag  = np.maximum(mag, 1e-15)
            s_db = 20 * np.log10(mag / (2**11))
            radar_data_matriz.append(s_db)

        mat = np.vstack(radar_data_matriz) # shape (161,4096)
        
        # GUARDAR DATA .npy
        #save_dir = '/home/dammr/Desktop/UC_SmartFarmRadar/capturas_radar' # Carpeta donde guardar
        #os.makedirs(save_dir, exist_ok=True) # crea la carpeta si no existe
        #existing_files = [f for f in os.listdir(save_dir) if f.endswith('.npy')]
        #numbers = [int(f.replace('.npy', '')) for f in existing_files if f.replace('.npy', '').isdigit()]
        #next_number = max(numbers) + 1 if numbers else 0
        #save_path = os.path.join(save_dir, f"{next_number}.npy")
        #np.save(save_path, mat)
        #self.get_logger().info(f'Datos guardados en {save_path}')

        # publicar ros
        msg = RadarData()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg() # timestamp actual
        msg.header.frame_id = 'radar_sensor'
        msg.rows = mat.shape[0]
        msg.cols = mat.shape[1]
        msg.dtype = str(mat.dtype) # "float64"
        msg.data = mat.flatten().tolist() # aplanado
        self.pub_matrix.publish(msg)
        self.get_logger().info(f"Publicado Matrix2D: {msg.rows} {msg.cols}, dtype={msg.dtype}")

        self.get_logger().info('Barrido completo')

def main(args=None):
    rclpy.init(args=args)
    node = RadarNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()