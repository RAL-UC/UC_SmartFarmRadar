#!/usr/bin/env python3
# ejecutable con interprete python3
import rclpy # ros2 
from rclpy.node import Node
import adi # libreria de analog devices
import time
import numpy as np
# mensajes de ros
#from std_msgs.msg import Float32MultiArray
from radar_msg.msg import RadarData
from std_msgs.msg import Bool
from std_msgs.msg import Header
import os
from ament_index_python.packages import get_package_share_directory # archivos de calibracion
from radar_package.parametros import *
#import sys
#import os

# recursos
pkg_share = get_package_share_directory('radar_package')
path_gain = os.path.join(pkg_share, 'resource', 'gain_cal_val.pkl')
path_phase = os.path.join(pkg_share, 'resource', 'phase_cal_val.pkl')

# herencia de node, al instanciar se registra en el grafo de ROS2
class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_node')
        self.retry_count = 0
        self.max_retries = 1 # -1 para infinitos reintentos
        self.retry_interval = 5.0 # segundos
        self.hardware_ready = False
        self.reconnect_timer = None

        # parámetros configurables desde línea de comandos o launch 
        self.declare_parameter('angle_min', -80) # grados
        self.declare_parameter('angle_max', 80) # grados
        self.declare_parameter('angle_step', 1) # grados

        # Leer parámetros
        p = self.get_parameter
        self.angle_min = p('angle_min').get_parameter_value().integer_value
        self.angle_max = p('angle_max').get_parameter_value().integer_value
        self.angle_step  = p('angle_step').get_parameter_value().integer_value

        self.angles = np.arange(self.angle_min, self.angle_max+1, self.angle_step) # vector de angulos en recorrido

        # --- Publisher ---
        # clase de mensaje, nombre del topico, tamaño de buffer
        self.pub_matrix = self.create_publisher(RadarData, 'radar_data', 10)
        # -- Subscriber ---
        self.sub_trigger = self.create_subscription(Bool, 'allow_sweep', self.trigger_callback, 10)

        # Inicializar hardware
        self.init_hardware()

    def init_hardware(self):
        try:
            sdr = adi.ad9361(uri=SDR_URI)
            phaser = adi.CN0566(uri=PHASER_URI, sdr=sdr)
            self.get_logger().info("Hardware conectado")
        except Exception as e:
            self.get_logger().error(f'Error de conexión con radar: {e}')
            self.retry_count += 1
            if self.retry_count > self.max_retries and self.max_retries >= 0:
                self.get_logger().fatal("Se excedió el número máximo de reintentos. Apagando nodo.")

                if self.reconnect_timer is not None:
                    self.reconnect_timer.cancel()
                    self.reconnect_timer = None
                rclpy.shutdown() # apaga el nodo
                #os._exit(0)
                #sys.exit(1)
            else:
                self.get_logger().info(f"Reintentando conexión en {self.retry_interval} segundos...")
                if self.reconnect_timer is None:
                    self.reconnect_timer = self.create_timer(self.retry_interval, self.init_hardware)
            return
        
        if self.reconnect_timer is not None:
            self.reconnect_timer.cancel()
            self.reconnect_timer = None
        
        self.get_logger().info("Iniciando rutina de configuración")
        # Configuración del Phaser ADAR1000: gananacia y fase
        # IMPORTANTE: se deben importar los archivos de calibracion
        phaser.configure(device_mode="rx")
        phaser.element_spacing = ELEMENT_SPACING
        phaser.load_gain_cal(path_gain)
        phaser.load_phase_cal(path_phase)

        # ajustar a 0 la fase de cada canal con compensacion de calibracion
        for ch in range(8):
            phaser.set_chan_phase(ch, 0, apply_cal=True)

        # ganancia con compensacion de calibracion por cada canal
        gains = [6, 27, 66, 100, 100, 66, 27, 6] # ventana blackman
        for i, g in enumerate(gains):
            phaser.set_chan_gain(i, g, apply_cal=True)

        # configuración de los pines GPIO de la Raspberry Pi
        phaser._gpios.gpio_tx_sw = 0 # 0 = TX_OUT_2, 1 = TX_OUT_1
        # Control de la fuente del oscilador local (LO):
        # 1 = Usa el PLL/LO integrado en el hardware
        # 0 = Desactiva el PLL y el VCO, y cambia el interruptor para usar una entrada LO externa
        phaser._gpios.gpio_vctrl_1 = 1
        # Control de la ruta de transmisión:
        # 1 = Envía la señal del LO al circuito de transmisión
        # 0 = Desactiva la ruta de transmisión y envía la señal LO a la salida LO_OUT
        phaser._gpios.gpio_vctrl_2 = 1

        # Configuración del receptor Rx del PlutoSDR
        sdr.sample_rate = int(SAMPLE_RATE) # Establece la tasa de muestreo en Hz
        sdr.rx_lo = int(CENTER_FREQ) # Configura el oscilador local (LO) en la frecuencia central
        sdr.rx_enabled_channels = [0, 1] # Habilita los canales de recepción: Canal 0 (Rx1 / voltage0) - Canal 1 (Rx2 / voltage1)
        sdr.gain_control_mode_chan0 = "manual" # manual o slow_attack (automatico segun señal de recepcion)
        sdr.gain_control_mode_chan1 = "manual" # manual o slow_attack (automatico segun señal de recepcion)
        sdr.rx_hardwaregain_chan0 = int(RX_GAIN_CHAN0) # valor entre -3 y 70
        sdr.rx_hardwaregain_chan1 = int(RX_GAIN_CHAN1) # valor entre -3 y 70

        # Configuración del transmisor Tx del SDR
        sdr.tx_lo = int(CENTER_FREQ) # Configura el oscilador local (LO) para transmisión en la misma frecuencia central
        sdr.tx_enabled_channels = [0, 1] # Habilita los canales de transmision: Canal 0 Tx1 - Canal 1 Tx2
        sdr.tx_cyclic_buffer = True # buffer ciclico para modo rafaga TDD
        sdr.tx_hardwaregain_chan0 = TX_GAIN_CHAN0 # valor entre 0 y -88
        sdr.tx_hardwaregain_chan1 = TX_GAIN_CHAN1 # valor entre 0 y -88

        # Configuración del PLL ADF4159 (Phase-Locked Loop) en el phaser como rampa
        # PLL tiene retroalimentacion con valor /4

        phaser.frequency = int(OUTPUT_FREQ + sdr.rx_lo + SIGNAL_FREQ) // 4 # 10GHz + 100kHz + 2.2GHz
        phaser.freq_dev_range = int(BANDWIDTH / 4) # desviación de frecuencia total de la rampa de frecuencia completa en Hz
        phaser.freq_dev_step = int((BANDWIDTH/4) / NUM_STEPS) # desviacion en cada paso
        phaser.freq_dev_time = int(RAMP_TIME) # tiempo total de la rampa de frecuencia completa en us

        phaser.delay_word = 4095 # Palabra de retardo de 12 bits. 4095 * PFD = 40.95 us.
        # PFD Phase Frequency Detector - Detector de Fase y Frecuencia -> ajuste de frecuencia de oscilacion y sincronizacion
        # Para rampas en diente de sierra, este valor también define la duración de la señal Ramp_complete.
        phaser.delay_clk = "PFD" # Reloj de referencia para el retardo. Puede ser 'PFD' (Phase Frequency Detector) o 'PFD*CLK1' (multiplicado por un factor adicional de tiempo)
        phaser.delay_start_en = 0 # Habilitación del retardo de inicio: 0 = deshabilitado, 1 = habilitado
        phaser.ramp_delay_en = 0 # Habilitación del retardo entre rampas: 0 = sin retardo, 1 = introduce un retardo entre cada rampa
        phaser.trig_delay_en = 0 # Habilitación del retardo en el disparo de la señal (trigger delay): 0 = deshabilitado, 1 = habilitado
        phaser.ramp_mode = "single_sawtooth_burst" # puede ser: "disabled", "continuous_sawtooth", "continuous_triangular", "single_sawtooth_burst", "single_ramp_burst"
        phaser.sing_ful_tri = 0 # deshabilitar/habilitar triángulo completo: esto se utiliza con el modo single_ramp_burst
        phaser.tx_trig_en = 1 # iniciar una rampa con TXdata
        phaser.enable = 0 # 0 = PLL habilitado, 1 = PLL deshabilitado, actualiza todos los registros -> colocar al final

        # Sincronizar chirridos con el inicio de cada búfer de recepción de PlutoSDR
        # Configurar el controlador TDD
        sdr_pins = adi.one_bit_adc_dac(SDR_URI) # se crea un objeto sdr_pins para controlar los GPIOs del PlutoSDR
        # - True: se habilita la activación de captura externa mediante el GPIO L24N en PlutoSDR
        # - False: se generará un pulso de activación interno cada segundo
        sdr_pins.gpio_tdd_ext_sync = True 
        tdd = adi.tddn(SDR_URI) # Se crea el objeto tdd que representa el controlador TDD (Time Division Duplexing) del PlutoSDR
        sdr_pins.gpio_phaser_enable = True # Habilita el pin gpio_phaser_enable 
        tdd.enable = False # deshabilitar TDD para configurar los registros
        tdd.sync_external = True # la sincronización será por señal externa
        tdd.startup_delay_ms = 0 # no se espera ningún retardo tras recibir el trigger externo
        tdd.frame_length_ms = PRI # cada chirrido está espaciado a esta distancia
        tdd.burst_count = NUM_CHIRPS # numero de chirridos en un búfer de recepción continuo

        # canal 0
        tdd.channel[0].enable = True # Habilita el canal 0 del TDD
        tdd.channel[0].polarity = False # polaridad de la señal (activacion con nivel alto)
        tdd.channel[0].on_raw = 0 # el canal se activa al instante 0 del cuadro
        tdd.channel[0].off_raw = 10 # se desactiva en el tick 10
        # cuando recibir, transmitir o activar perifericos
        # canal 1
        tdd.channel[1].enable = True 
        tdd.channel[1].polarity = False
        tdd.channel[1].on_raw = 0
        tdd.channel[1].off_raw = 10
        # canal 2
        tdd.channel[2].enable = True
        tdd.channel[2].polarity = False
        tdd.channel[2].on_raw = 0
        tdd.channel[2].off_raw = 10

        tdd.enable = True # se activa el TDD con toda la configuración aplicada
        # el Pluto responderá a los triggers externos, activando los canales configurados, transmitiendo chirridos y capturando datos en los búferes de recepción

        # Desde el inicio de cada rampa, ¿cuántos puntos "buenos" queremos?
        # Para una mejor linealidad de frecuencia, evite el inicio de las rampas

        self.start_offset_time = tdd.channel[0].on_ms/1e3 + BEGIN_OFFSET_TIME # desde el inicio de encendido del canal TDD hasta donde realmente empiezan las muestras útiles
        self.start_offset_samples = int(self.start_offset_time * SAMPLE_RATE) # cuántas muestras deben ignorarse para empezar justo desde esa parte útil

        # dimension de la fft para el número de puntos de datos de rampa
        # FFT funcionan más rápido y eficientemente cuando su tamaño es una potencia de 2
        # relleno con ceros (zero-padding) -> refina la resolución espectral
        # Mayor número de puntos en el eje de frecuencias, mejor visualizacion del espectro
        # Si fft_size < num_samples_frame, se perderían muestras, lo cual distorsiona el análisis
        # La resolución en el dominio de la frecuencia depende de:
        # delta f = fs/N
        # fs: tasa de muestreo y N el tamaño de la FFT
        # aumentar FFT disminuye delta f permitiendo distinguir frecuencias mas cercanas
        # Define cuántos puntos usas para calcular el espectro de cada chirrido o ráfaga
        # análisis de frecuencia de un subconjunto del buffer
        power = 8 # potencia
        self.fft_size = int(2**power) # potencia de 2^8 = 256 
        self.num_samples_frame = int(tdd.frame_length_ms/1000*SAMPLE_RATE) # cuántas muestras hay en un frame TDD completo
        # aumento del tamaño de la FFT para que sea mayor que num_samples_frame
        while self.num_samples_frame > self.fft_size:     
            power=power+1
            self.fft_size = int(2**power) 
            if power==18:
                break
        #print("fft_size =", self.fft_size) # 1024

        # el tamaño del búfer de recepción de PlutoSDR debe ser mayor que el tiempo total para cada uno de los conjuntos de chirridos
        # cuántas muestras puede almacenar el PlutoSDR de una vez antes de que las leas del computerhost
        # para modo rafaga TDD adquisicion en tiempo real y de forma continua
        # se calcula para que pueda contener todos los chirridos de un frame TDD completo
        # buffer_size >= SAMPLE_RATE > duracion total de los chirps
        # eficiencia de hardware uso de potencia de 2
        total_time = tdd.frame_length_ms * NUM_CHIRPS # tiempo en ms
        buffer_time = 0
        power=12
        while total_time > buffer_time:     
            power=power+1
            buffer_size = int(2**power) 
            buffer_time = buffer_size/SAMPLE_RATE*1000 # buffer time in ms
            if power==22:
                break # El tamaño máximo del búfer de PlutoSDR es 2**23, pero para el modo ráfaga tdd, configúrelo en 2**22
        #print("buffer_size:", buffer_size) # 8192
        #print("buffer_time:", buffer_time, " ms") # 13.653333333333332
        sdr.rx_buffer_size = buffer_size

        # Generación de una señal senoidal
        fs = int(SAMPLE_RATE) # frecuencia de muestreo
        N = int(sdr.rx_buffer_size) # tamaño del buffer de captura
        fc = int(SIGNAL_FREQ / (fs / N)) * (fs / N) # frecuencia más cercana representable con ese tamaño de muestra y frecuencia de muestreo
        # evitar fugas espectrales, producir un peak limpio, mejora de analisis espectral
        ts = 1 / float(fs) # periodo
        t = np.arange(0, N * ts, ts) # vector de tiempo
        i = np.cos(2 * np.pi * t * fc) * 2 ** 14 # señal en fase escalada a 14 bits
        q = np.sin(2 * np.pi * t * fc) * 2 ** 14 # señal en cuadratura escalada a 14 bits
        iq = 1 * (i + 1j * q) # Construir la señal compleja I/Q (mezcla de fase y cuadratura)

        # ENVÍO DE DATOS AL SDR
        sdr._ctx.set_timeout(30000) # error de tiempo de espera de 30 s
        sdr._rx_init_channels() # verifica si ya existe una máscara de canales Rx configurada -> si no está, significa que los canales Rx aún no se han preparado para recibir datos.
        # crea una máscara de canales IIO usando iio.ChannelsMask con dispositivo de recepción (RxADC)
        # se define qué canales se habilitarán y estarán activos cuando se crea el buffer de recepcion DMA (direct memory acces)
        sdr.tx([iq, iq]) # se transmite con ganancia solo en Tx2

        # guardar instancias
        self.my_sdr = sdr
        self.my_phaser = phaser

        self.hardware_ready = True
        self.get_logger().info("Fin rutina de configuración")

        self.ready_for_allow = True
        self.get_logger().info("Publicar True en /allow_sweep para iniciar barrido de angulos")

    def trigger_callback(self, msg):
        if not self.hardware_ready:
            self.get_logger().warn("Hardware no listo: Ignorando actividad")
            return
    
        if msg.data and self.ready_for_allow:
            self.get_logger().info("/allow_sweep True recibido: Ejecutando barrido")
            self.ready_for_allow = False # bloquea hasta recibir False
            try:
                self.do_sweep()
            except Exception as e:
                self.get_logger().error(f"¡Error durante el barrido! Hardware podría estar desconectado: {e}")
                self.hardware_ready = False
                self.retry_count = 0
                self.get_logger().info(f"Intentando reconexión en {self.retry_interval} segundos...")
                if self.reconnect_timer is None:
                    self.reconnect_timer = self.create_timer(self.retry_interval, self.init_hardware)
            finally:
                self.ready_for_allow = True
                self.get_logger().info("Hardware listo: Esperando /allow_sweep True")

    def do_sweep(self):
        radar_data_matriz = [] # matriz de data con tamaño (len_angles, len_data)

        for theta in self.angles:
            # 1) direccion del haz
            # Fórmula:
            # Phase delta = 2*Pi*d*sin(theta)/lambda = 2*Pi*d*sin(theta)*f/c
            # Se usan: f_signal_freq = 10.25 GHz, d = 0.014 m, c = 3e8 m/s
            # se utiliza f_signal_freq = 10.25 GHz ya que el phaser escucha entre 10GHz y 10.5GHz
            # se utiliza desplazamiento de fase en un rango pequeño en base a la frecuencia central
            # introduce un error el cual es pequeño 
            phase_delta = (2*np.pi * SIGNAL_FREQ_PHASER_RECEPTION * ELEMENT_SPACING * np.sin(np.radians(theta))) / C
            self.my_phaser.set_beam_phase_diff(np.degrees(phase_delta))
            time.sleep(0.1)

            self.my_phaser._gpios.gpio_burst = 0
            self.my_phaser._gpios.gpio_burst = 1
            self.my_phaser._gpios.gpio_burst = 0
            # 2) Recepción y FFT
            data = self.my_sdr.rx()
            sum_data = data[0] + data[1] # canal 1 y canal 2

            rx_bursts = np.zeros((NUM_CHIRPS, GOOD_RAMP_SAMPLES), dtype=complex)
            for burst in range(NUM_CHIRPS): # para cada chirrido individual
                # indicie inicial y final dentro del arreglo sum_data
                start_index = self.start_offset_samples + burst*self.num_samples_frame
                stop_index = start_index + GOOD_RAMP_SAMPLES
                rx_bursts[burst] = sum_data[start_index:stop_index]
                burst_data = np.ones(self.fft_size, dtype=complex)*1e-10 # arreglo con tamaño fft_size complejo con valores pequeños
                win_funct = np.blackman(len(rx_bursts[burst])) # ventana blackman
                #win_funct = np.ones(len(rx_bursts[burst])) # ventana rectangular
                # Se coloca el chirp extraído en una posición dentro de burst_data, multiplicado por la ventana.
                burst_data[self.start_offset_samples:(self.start_offset_samples+GOOD_RAMP_SAMPLES)] = rx_bursts[burst]*win_funct

            sp = np.fft.fftshift(np.abs(np.fft.fft(burst_data)))
            s_mag = np.abs(sp) / np.sum(win_funct)
            s_mag = np.maximum(s_mag, 10 ** (-15))
            s_dbfs = 20 * np.log10(s_mag / (2 ** 11))

            radar_data_matriz.append(s_dbfs)

        mat = np.vstack(radar_data_matriz) # shape (161,1024)
        print(mat.shape)
        
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

        self.ready_for_allow = True
        self.get_logger().info('Barrido completado: Habilitado para recibir /allow_sweep True')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = RadarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Nodo interrumpido por el usuario.")
    except Exception as e:
        print(f"Excepción no controlada: {e}")
    finally:
        #node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        #sys.exit(0)
        #os._exit(0)

if __name__ == '__main__':
    main()