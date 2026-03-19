#!/usr/bin/env python3
# ejecutable con interprete python3
import rclpy # ros2 
from rclpy.node import Node # clase nodo de ros2
import adi # libreria de analog devices
#import time # control temporal
import numpy as np # calculo matematico
# mensajes de ros
from radar_msg.msg import RadarData
from std_msgs.msg import Header
#from std_msgs.msg import Bool
import os # sistema
#import sys # interprete
from ament_index_python.packages import get_package_share_directory # archivos de recursos
#from radar_package.parametros import * # importar parametros
from rclpy.action import ActionServer # acciones de ros2
from radar_msg.action import RadarBeamform #accion de beamforming

# recursos de calibracion
pkg_share = get_package_share_directory('radar_package')
path_gain = os.path.join(pkg_share, 'resource', 'gain_cal_val.pkl') # ganancia
path_phase = os.path.join(pkg_share, 'resource', 'phase_cal_val.pkl') # fase

class MaxRetriesExceeded(Exception):
    """Excepción personalizada para indicar que se agotaron los reintentos de conexión"""
    pass

# herencia de node, al instanciar se registra en el grafo de ROS2
class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_node') # declarar herencia

        # parámetros configurables desde línea de comandos o archivo de configuracion
        # radar.yaml
        self.declare_parameter('sdr_uri', "ip:phaser.local:50901")
        self.declare_parameter('phaser_uri', "ip:phaser.local")
        self.declare_parameter('element_spacing_m', 0.014)
        self.declare_parameter('sample_rate_hz', 0.6e6)
        self.declare_parameter('center_frequency_hz', 2.2e9)
        self.declare_parameter('signal_freq_hz', 100e3)
        self.declare_parameter('output_freq_hz', 10e9)
        self.declare_parameter('bandwidth_hz', 500e6)
        self.declare_parameter('rx_gain_chan0', 70)
        self.declare_parameter('rx_gain_chan1', 70)
        self.declare_parameter('tx_gain_chan0', -88)
        self.declare_parameter('tx_gain_chan1', 0)
        self.declare_parameter('num_steps', 500)
        self.declare_parameter('ramp_time_us', 500)
        self.declare_parameter('pri_ms', 1.5)
        self.declare_parameter('num_chirps', 1)
        self.declare_parameter('begin_offset_time_s', 0.00005)
        self.declare_parameter('signal_freq_phaser_rx_hz', 10.25e9)
        self.declare_parameter('speed_of_light', 3e8)
        self.declare_parameter('good_ramp_samples', 270)
        self.declare_parameter('rbeam_angledeg_min', -80)
        self.declare_parameter('rbeam_angledeg_max', 80)
        self.declare_parameter('rbeam_angledeg_step', 1)

        # carga de valores
        # radar.yaml
        self.sdr_uri = self.get_parameter('sdr_uri').value
        self.phaser_uri = self.get_parameter('phaser_uri').value
        self.element_spacing_m = self.get_parameter('element_spacing_m').value
        self.sample_rate_hz = self.get_parameter('sample_rate_hz').value
        self.center_frequency_hz = self.get_parameter('center_frequency_hz').value
        self.signal_freq_hz = self.get_parameter('signal_freq_hz').value
        self.output_freq_hz = self.get_parameter('output_freq_hz').value
        self.bandwidth_hz = self.get_parameter('bandwidth_hz').value
        self.rx_gain_chan0 = self.get_parameter('rx_gain_chan0').value
        self.rx_gain_chan1 = self.get_parameter('rx_gain_chan1').value
        self.tx_gain_chan0 = self.get_parameter('tx_gain_chan0').value
        self.tx_gain_chan1 = self.get_parameter('tx_gain_chan1').value
        self.num_steps = self.get_parameter('num_steps').value
        self.ramp_time_us = self.get_parameter('ramp_time_us').value
        self.pri_ms = self.get_parameter('pri_ms').value
        self.num_chirps = self.get_parameter('num_chirps').value
        self.begin_offset_time_s = self.get_parameter('begin_offset_time_s').value
        self.signal_freq_phaser_rx_hz = self.get_parameter('signal_freq_phaser_rx_hz').value
        self.speed_of_light = self.get_parameter('speed_of_light').value
        self.good_ramp_samples = self.get_parameter('good_ramp_samples').value
        self.rbeam_angledeg_min  = self.get_parameter('rbeam_angledeg_min').value
        self.rbeam_angledeg_max = self.get_parameter('rbeam_angledeg_max').value
        self.rbeam_angledeg_step  = self.get_parameter('rbeam_angledeg_step').value

        # reconexion
        self.retry_count = 0 # conteo de reintentos
        self.max_retries = 1 # -1 para infinitos reintentos
        self.retry_interval = 5.0 # segundos
        self.hardware_ready = False
        self.reconnect_timer = None

        self.angles = np.arange(self.rbeam_angledeg_min, self.rbeam_angledeg_max+1, self.rbeam_angledeg_step) # vector de angulos en recorrido
        #self.get_logger().info(f"self.angles: {self.angles}")

        # servidor basado en acciones
        # tipo de accion, nombre de la accion, funcion callback
        self.beamform_server = ActionServer(self, RadarBeamform, 'radar_beamform', self.execute_beamform_cb)

        # ventana que multiplica los valores antes de la transformada de fourier
        #self.win_funct = np.ones(GOOD_RAMP_SAMPLES, dtype=np.float64) # ventana rectangular
        #self.win_funct = np.blackman(GOOD_RAMP_SAMPLES) # ventana blackman
        #self.win_funct = np.hamming(GOOD_RAMP_SAMPLES) # ventana hamming
        # obtener suma para normalizar la amplitud del espectro
        #self.sum_win_funct = np.sum(self.win_funct)

        # inicializar hardware
        self.init_hardware() # restriccion: para inicializar se debe esperar correctamente a que el disposivo encienda

    def init_hardware(self):
        try:
            self.get_logger().info("Intentando conexión")
            sdr = adi.ad9361(uri=self.sdr_uri)
            phaser = adi.CN0566(uri=self.phaser_uri, sdr=sdr)
        except Exception as e:
            self.get_logger().error(f'Error de conexión con radar: {e}')
            self.retry_count += 1
            if self.retry_count > self.max_retries and self.max_retries >= 0:
                self.get_logger().fatal("Se excedió el número máximo de reintentos: Apagando nodo radar.")

                if self.reconnect_timer is not None:
                    self.reconnect_timer.cancel()
                    self.reconnect_timer = None

                raise MaxRetriesExceeded("Límite de reintentos superado")
            else:
                self.get_logger().info(f"Reintentando conexión en {self.retry_interval} segundos...")
                if self.reconnect_timer is None:
                    self.reconnect_timer = self.create_timer(self.retry_interval, self.init_hardware)
            return
        
        self.get_logger().info("Hardware conectado")
        if self.reconnect_timer is not None:
            self.reconnect_timer.cancel()
            self.reconnect_timer = None
        
        self.get_logger().info("Iniciando rutina de configuración")
        # Configuración del Phaser ADAR1000: gananacia y fase
        # IMPORTANTE: se deben importar los archivos de calibracion
        phaser.configure(device_mode="rx") # modo receptor
        phaser.element_spacing = self.element_spacing_m # espaciado de elementos
        phaser.load_gain_cal(path_gain) # calibracion de ganancia
        phaser.load_phase_cal(path_phase) # calibracion de fase

        # ganancia y direccionamiento deseado 
        # ajustar a 0 la fase de cada canal con compensacion de calibracion
        for ch in range(8):
            phaser.set_chan_phase(ch, 0, apply_cal=True)

        # ganancia con compensacion de calibracion por cada canal
        gains = [6, 27, 66, 100, 100, 66, 27, 6] # ventana blackman en ganancia de arreglo
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
        sdr.sample_rate = int(self.sample_rate_hz) # Establece la tasa de muestreo en 600 kHz
        sdr.rx_lo = int(self.center_frequency_hz) # Configura el oscilador local (LO) en la frecuencia central 2.2 GHz
        sdr.rx_enabled_channels = [0, 1] # Habilita los canales de recepción: Canal 0 (Rx1 / voltage0) - Canal 1 (Rx2 / voltage1)
        sdr.gain_control_mode_chan0 = "manual" # manual o slow_attack (automatico segun señal de recepcion)
        sdr.gain_control_mode_chan1 = "manual" # manual o slow_attack (automatico segun señal de recepcion)
        sdr.rx_hardwaregain_chan0 = int(self.rx_gain_chan0) # valor entre -3 y 70
        sdr.rx_hardwaregain_chan1 = int(self.rx_gain_chan1) # valor entre -3 y 70

        # Configuración del transmisor Tx del SDR
        sdr.tx_lo = int(self.center_frequency_hz) # Configura el oscilador local (LO) para transmisión en la misma frecuencia central
        sdr.tx_enabled_channels = [0, 1] # Habilita los canales de transmision: Canal 0 Tx1 - Canal 1 Tx2
        sdr.tx_cyclic_buffer = True # buffer ciclico para modo rafaga TDD burst, de lo contrario hay comportamiento aleatorio
        sdr.tx_hardwaregain_chan0 = self.tx_gain_chan0 # valor entre 0 y -88
        sdr.tx_hardwaregain_chan1 = self.tx_gain_chan1 # valor entre 0 y -88

        # Configuración del PLL ADF4159 (Phase-Locked Loop) en el phaser como rampa
        # PLL tiene retroalimentacion con valor /4

        phaser.frequency = int(self.output_freq_hz + sdr.rx_lo + self.signal_freq_hz) // 4 # 10GHz + 100kHz + 2.2GHz
        phaser.freq_dev_range = int(self.bandwidth_hz / 4) # desviación de frecuencia total de la rampa de frecuencia completa en 500 MHz
        phaser.freq_dev_step = int((self.bandwidth_hz/4) / self.num_steps) # desviacion en cada paso 500 steps
        phaser.freq_dev_time = int(self.ramp_time_us) # tiempo total de la rampa de frecuencia completa en 500 us (1step/us)

        phaser.delay_word = 4095 # Palabra de retardo de 12 bits. 4095 * PFD = 40.95 us -> reloj de 10ns o 100 MHz
        # PFD Phase Frequency Detector - Detector de Fase y Frecuencia -> ajuste de frecuencia de oscilacion y sincronizacion
        # Para rampas en diente de sierra, este valor define la duración de la señal
        phaser.delay_clk = "PFD" # Reloj de referencia para el retardo. Puede ser 'PFD' (Phase Frequency Detector) o 'PFD*CLK1' (multiplicado por un factor adicional de tiempo)
        phaser.delay_start_en = 0 # Habilitación del retardo de inicio: 0 = deshabilitado, 1 = habilitado
        phaser.ramp_delay_en = 0 # Habilitación del retardo entre rampas: 0 = sin retardo, 1 = introduce un retardo entre cada rampa
        phaser.trig_delay_en = 0 # Habilita un retardo adicional para rampas triangulares: 0 = deshabilitado, 1 = habilitado -> cortar la punta del triangulo
        phaser.ramp_mode = "single_sawtooth_burst" # puede ser: "disabled", "continuous_sawtooth", "continuous_triangular", "single_sawtooth_burst", "single_ramp_burst"
        phaser.sing_ful_tri = 0 # 0 deshabilitar/1 habilitar triángulo completo: esto se utiliza con el modo single_ramp_burst
        phaser.tx_trig_en = 1 # iniciar una rampa con TXdata
        phaser.enable = 0 # 0 = PLL habilitado, 1 = PLL deshabilitado, actualiza todos los registros -> colocar al final

        # Sincronizar chirridos en phaser con el inicio de cada búfer de recepción de PlutoSDR - en microcontrolador
        # Configurar el controlador TDD
        # no jitter de sotfware -> señales a nivel de hardware
        sdr_pins = adi.one_bit_adc_dac(self.sdr_uri) # se crea un objeto sdr_pins para controlar los GPIOs del PlutoSDR
        # - True: se habilita la activación de captura externa mediante el GPIO L24N en PlutoSDR
        # - False: se generará un pulso de activación interno cada segundo
        sdr_pins.gpio_tdd_ext_sync = True # sincronizacion por trigger
        tdd = adi.tddn(self.sdr_uri) # Se crea el objeto tdd que representa el controlador TDD (Time Division Duplexing) del PlutoSDR
        sdr_pins.gpio_phaser_enable = True # Habilita el pin gpio_phaser_enable para control sobre el phaser
        tdd.enable = False # deshabilitar TDD para configurar los registros
        tdd.sync_external = True # la sincronización será por señal externa - pluto esclavo de un pulso
        tdd.startup_delay_ms = 0 # no se espera ningún retardo tras recibir el trigger externo
        tdd.frame_length_ms = self.pri_ms # cada chirrido está espaciado a esta distancia
        tdd.burst_count = self.num_chirps # numero de chirridos en un búfer de recepción continuo

        # microcontrolador, pluto, phaser -> buffers
        # cuando transmitir, recibir o activar perifericos
        # canal 0 (sincronizacion de las salidas de datos TX)
        tdd.channel[0].enable = True # Habilita el canal 0 del TDD
        tdd.channel[0].polarity = False # polaridad de la señal (activacion con nivel alto)
        tdd.channel[0].on_raw = 0 # el canal se activa al instante 0 del cuadro
        tdd.channel[0].off_raw = 10 # se desactiva en el tick 10
        # canal 1 (sincronizacion de inicio para el bufer recibido de Pluto)
        tdd.channel[1].enable = True 
        tdd.channel[1].polarity = False
        tdd.channel[1].on_raw = 0
        tdd.channel[1].off_raw = 10
        # canal 2 (sincronizacion entre el bufer de transmision y el de recepcion)
        # sirve para creacion de ondas digital codificadas 
        tdd.channel[2].enable = True
        tdd.channel[2].polarity = False
        tdd.channel[2].on_raw = 0
        tdd.channel[2].off_raw = 10

        tdd.enable = True # se activa el TDD con toda la configuración aplicada
        # el Pluto responderá a los triggers externos, activando los canales configurados, transmitiendo chirridos y capturando datos en los búferes de recepción

        # Desde el inicio de start_offset_time de cada rampa, ¿cuántos puntos "buenos" queremos?
        # Para una mejor linealidad de frecuencia, evite el inicio de las rampas

        self.start_offset_time = int(tdd.channel[0].on_ms/1e3 + self.begin_offset_time_s) # desde el inicio de encendido del canal TDD hasta donde realmente empiezan las muestras útiles
        self.start_offset_samples = int(self.start_offset_time * self.sample_rate_hz) # cuántas muestras deben ignorarse para empezar justo desde esa parte útil
        #self.get_logger().info(f"self.start_offset_time: {self.start_offset_time}, self.start_offset_samples: {self.start_offset_samples}") # 0, 0

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
        self.fft_size = int(2**power) # potencia de 2^8 = 256 a 4096
        self.num_samples_frame = int(tdd.frame_length_ms/1000*self.sample_rate_hz) # cuántas muestras hay en un frame TDD completo
        #.get_logger().info(f"self.num_samples_frame: {self.num_samples_frame}")
        # aumento del tamaño de la FFT para que sea mayor que num_samples_frame
        while self.num_samples_frame > self.fft_size:     
            power=power+1
            self.fft_size = int(2**power) 
            if power==18:
                break

        #self.get_logger().info(f"fft_size: {self.fft_size}) # 1024

        # el tamaño del búfer de recepción de PlutoSDR debe ser mayor que el tiempo total para cada uno de los conjuntos de chirridos
        # cuántas muestras puede almacenar el PlutoSDR de una vez antes de que las leas del computerhost
        # para modo rafaga TDD adquisicion en tiempo real y de forma continua
        # se calcula para que pueda contener todos los chirridos de un frame TDD completo
        # buffer_size/self.sample_rate_hz > duracion total de los chirps
        # eficiencia de hardware uso de potencia de 2
        total_time = tdd.frame_length_ms * self.num_chirps # tiempo en ms
        power=12
        buffer_size = int(2**power)
        buffer_time = buffer_size/self.sample_rate_hz*1000 # buffer time in ms
        while total_time > buffer_time:     
            power=power+1
            buffer_size = int(2**power) 
            buffer_time = buffer_size/self.sample_rate_hz*1000
            if power==22:
                break # El tamaño máximo del búfer de PlutoSDR es 2**23, pero para el modo ráfaga tdd, configúrelo en 2**22
        
        #self.get_logger().info(f"buffer_size: {self.buffer_size}, buffer_time: {buffer_time}") # 8192, 13.653333333333332
        sdr.rx_buffer_size = buffer_size

        # Generación de una señal senoidal
        fs = int(self.sample_rate_hz) # frecuencia de muestreo 600 kHz
        N = int(sdr.rx_buffer_size) # tamaño del buffer de captura
        fc = int(self.signal_freq_hz / (fs / N)) * (fs / N) # frecuencia más cercana representable con ese tamaño de muestra y frecuencia de muestreo
        # evitar fugas espectrales, producir un peak limpio, mejora de analisis espectral
        ts = 1 / float(fs) # periodo
        t = np.arange(0, N * ts, ts) # vector de tiempo
        i = np.cos(2 * np.pi * t * fc) * 2 ** 14 # señal en fase escalada a 14 bits
        q = np.sin(2 * np.pi * t * fc) * 2 ** 14 # señal en cuadratura escalada a 14 bits
        iq = 1 * (i + 1j * q) # Construir la señal compleja I/Q (mezcla de fase y cuadratura)

        # ENVÍO DE DATOS AL SDR
        sdr._ctx.set_timeout(30000) # tiempo de espera de error a 30 s
        sdr._rx_init_channels() # metodo no implementado
        sdr.tx([iq, iq]) # se transmite con ganancia solo en Tx2

        # guardar instancias
        self.my_sdr = sdr
        self.my_phaser = phaser

        self.hardware_ready = True
        self.get_logger().info("Fin rutina de configuración")


    async def execute_beamform_cb(self, goal_handle):
        """
        callback de accion ros2
        asincronica
        se ejecuta cuando llega una meta, esta basada en una peticion request, retorna un feedback y permite finalizar cuando se completa

        Ejecuta una adquisición beamforming y publica el RadarData resultante. 
        Devuelve success/message.
        """
        pan  = int(goal_handle.request.pan_deg)
        tilt = int(goal_handle.request.tilt_deg)
        self.get_logger().info(f"[Radar] Goal: pan={pan}°, tilt={tilt}°")

        feedback = RadarBeamform.Feedback()
        feedback.status = "Inicializando adquisición"
        goal_handle.publish_feedback(feedback)

        # Validación de estado de HW
        if not self.hardware_ready:
            feedback.status = "Hardware no listo; abortando"
            goal_handle.publish_feedback(feedback)
            goal_handle.abort()
            result = RadarBeamform.Result()
            result.success = False
            result.message = "Hardware no listo"
            return result

        try:
            feedback.status = "Adquiriendo y procesando"
            goal_handle.publish_feedback(feedback)

            # logica de beamforming
            mat = self.do_sweep() # matriz de valores

            # logica de captura para un solo angulo
            #mat = self.do_chirp() # # matriz de valores

            #self.get_logger().info(f"mat.dtype: {mat.dtype}")

            # formar mensaje de radar
            msg = RadarData()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg() # timestamp actual
            msg.header.frame_id = 'radar_sensor'
            msg.pan_deg = pan
            msg.tilt_deg = tilt
            msg.rows = mat.shape[0]
            msg.cols = mat.shape[1]
            # tipo de dato predefinido como float64 separado en reales e imaginarios
            msg.dtype = "float64"
            real64 = np.ascontiguousarray(np.real(mat), dtype=np.float64).ravel(order="C")
            imag64 = np.ascontiguousarray(np.imag(mat), dtype=np.float64).ravel(order="C")
            msg.data_real = real64.tolist()
            msg.data_imag = imag64.tolist()

            #self.get_logger().info(f"len real: {len(real64.tolist())}, len img: {len(imag64.tolist())} tipo: {msg.dtype}")

            feedback.status = "OK, devolviendo resultado"
            goal_handle.publish_feedback(feedback)

            result = RadarBeamform.Result()
            result.success = True
            result.message = f"Beamforming OK (pan={pan}°, tilt={tilt}°)"
            result.radar_data = msg
            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f"Error en beamforming: {e}")
            result = RadarBeamform.Result()
            result.success = False # faltara reconexion ?
            result.message = f"Error: {e}"
            # result.radar_data vacío
            goal_handle.abort()
            return result

    # hace un barrido del PTU y del phaser por varios angulos
    def do_sweep(self):
        radar_data_matriz = [] # matriz de data con tamaño (len_angles, len_data)
        for theta in self.angles: # 160 grados desde -80 a 80, debe tener concordancia con el PTU
            # 1) direccion del haz
            # Fórmula:
            # Phase delta = 2*Pi*d*sin(theta)/lambda = 2*Pi*d*sin(theta)*f/c
            # Se usan: f_signal_freq = 10.25 GHz, d = 0.014 m, c = 3e8 m/s
            # se utiliza f_signal_freq = 10.25 GHz ya que el phaser escucha entre 10GHz y 10.5GHz
            # se utiliza desplazamiento de fase en un rango pequeño en base a la frecuencia central
            # introduce un error el cual es pequeño 
            phase_delta = (2*np.pi * self.signal_freq_phaser_rx_hz * self.element_spacing_m * np.sin(np.radians(theta))) / self.speed_of_light
            self.my_phaser.set_beam_phase_diff(np.degrees(phase_delta))
            
            #time.sleep(0.05) # esperar tiempo de configuracion

            self.my_phaser._gpios.gpio_burst = 0
            self.my_phaser._gpios.gpio_burst = 1
            self.my_phaser._gpios.gpio_burst = 0
            # 2) Recepción y FFT
            data = self.my_sdr.rx()
            sum_data = data[0] + data[1] # canal 1 y canal 2 datos imaginarios

            #self.get_logger().info(f"shape sum_data: {sum_data.shape}")

            #rx_bursts = np.zeros((NUM_CHIRPS, GOOD_RAMP_SAMPLES), dtype=np.complex128) # rx_bursts limpio
            time_data = np.ones((self.num_chirps, self.fft_size), dtype=np.complex128)*1e-10 # fft_data limpio
            # se reemplazan los valores por los recibidos dejando un margen inicial en valores pequeños para solo considerar los GOOD_RAMP_SAMPLES

            for burst in range(self.num_chirps): # para cada chirrido individual
                # indicie inicial y final dentro del arreglo sum_data
                start_index = self.start_offset_samples + burst*self.num_samples_frame
                stop_index = start_index + self.good_ramp_samples
                #burst_data = np.ones(self.fft_size, dtype=np.complex128)*1e-10 # arreglo con tamaño fft_size complejo con valores pequeños
                #burst_slice = sum_data[start_index:stop_index] * self.win_funct
                burst_slice = sum_data[start_index:stop_index]
                #self.get_logger().info(f"start_index: {start_index} stop_index: {stop_index}")

                # Se coloca el chirp extraído en una posición dentro de burst_data, multiplicado por la ventana.
                #burst_data[self.start_offset_samples:(self.start_offset_samples+GOOD_RAMP_SAMPLES)] = burst_slice
                time_data[burst,self.start_offset_samples:(self.start_offset_samples+self.good_ramp_samples)] = burst_slice

            avg_time = np.mean(time_data[:,:], axis=0) 
            #sp = np.fft.fftshift(np.abs(np.fft.fft(avg_time))) # fft y shift a centro
            # redundancia en valor absoluto
            #s_mag = np.abs(sp) / self.sum_win_funct # calcula valor absoluto y aplica una normalización por la suma de los coeficientes de la ventana
            #s_mag = sp / self.sum_win_funct
            #s_mag = np.maximum(s_mag, 10 ** (-15)) # pone un piso minimo para evitar valores pequeños en s_mag y posteriormente -inf con el logaritmo
            # espectro en magnitud lineal (valor absoluto de la FFT, ya normalizado por la ventana)
            # normalizando respecto al valor máximo posible de la ADC (full-scale) 12 bits con signo
            # Por convención, la magnitud en decibeles de una señal se calcula como
            # 20 * log_10 (A/A_ref)
            # Si un bin de FFT tiene s_mag = 2048 -> 0dBFS
            # decibeles referidos al máximo teórico de la ADC
            #s_dbfs = 20 * np.log10(s_mag / (2 ** 11))
            # se obtiene un rango desde -366.22 db a 0 aprox

            #radar_data_matriz.append(s_dbfs)
            radar_data_matriz.append(avg_time)

        mat = np.vstack(radar_data_matriz) # shape (barrido en angulos, tamaño fft)
        
        # GUARDAR DATA .npy
        #save_dir = '/home/dammr/Desktop/UC_SmartFarmRadar/capturas_radar' # Carpeta donde guardar
        #os.makedirs(save_dir, exist_ok=True) # crea la carpeta si no existe
        #existing_files = [f for f in os.listdir(save_dir) if f.endswith('.npy')]
        #numbers = [int(f.replace('.npy', '')) for f in existing_files if f.replace('.npy', '').isdigit()]
        #next_number = max(numbers) + 1 if numbers else 0
        #save_path = os.path.join(save_dir, f"{next_number}.npy")
        #np.save(save_path, mat)
        #self.get_logger().info(f'Datos guardados en {save_path}')

        return mat

    # hace un barrido del phaser sin mover el PTU
    def do_chirp(self):
        theta = 0
        # 1) direccion del haz
        # Fórmula:
        # Phase delta = 2*Pi*d*sin(theta)/lambda = 2*Pi*d*sin(theta)*f/c
        # Se usan: f_signal_freq = 10.25 GHz, d = 0.014 m, c = 3e8 m/s
        # se utiliza f_signal_freq = 10.25 GHz ya que el phaser escucha entre 10GHz y 10.5GHz
        # se utiliza desplazamiento de fase en un rango pequeño en base a la frecuencia central
        # introduce un error el cual es pequeño 
        phase_delta = (2*np.pi * self.signal_freq_phaser_rx_hz * self.element_spacing_m * np.sin(np.radians(theta))) / self.speed_of_light
        self.my_phaser.set_beam_phase_diff(np.degrees(phase_delta))

        #time.sleep(0.05)

        self.my_phaser._gpios.gpio_burst = 0
        self.my_phaser._gpios.gpio_burst = 1
        self.my_phaser._gpios.gpio_burst = 0
        # 2) Recepción y FFT
        data = self.my_sdr.rx()
        sum_data = data[0] + data[1] # canal 1 y canal 2

        #self.get_logger().info(f"shape sum_data: {sum_data.shape}")

        #rx_bursts = np.zeros((NUM_CHIRPS, GOOD_RAMP_SAMPLES), dtype=complex)
        time_data = np.ones((self.num_chirps, self.fft_size), dtype=np.complex128) # fft_data limpio
        # se reemplazan los valores por los recibidos dejando un margen inicial en valores pequeños para solo considerar los GOOD_RAMP_SAMPLES

        for burst in range(self.num_chirps): # para cada chirrido individual
            # indicie inicial y final dentro del arreglo sum_data
            start_index = self.start_offset_samples + burst*self.num_samples_frame
            stop_index = start_index + self.good_ramp_samples
            #burst_data = np.ones(self.fft_size, dtype=complex)*1e-10 # arreglo con tamaño fft_size complejo con valores pequeños
            #burst_slice = sum_data[start_index:stop_index] * self.win_funct
            burst_slice = sum_data[start_index:stop_index]

            # Se coloca el chirp extraído en una posición dentro de burst_data, multiplicado por la ventana.
            #burst_data[self.start_offset_samples:(self.start_offset_samples+GOOD_RAMP_SAMPLES)] = rx_bursts[burst]*win_funct
            time_data[burst,self.start_offset_samples:(self.start_offset_samples+self.good_ramp_samples)] = burst_slice
        
        avg_time = np.mean(time_data[:,:], axis=0) # promediar entre filas, matiene las columnas
        #sp = np.fft.fftshift(np.abs(np.fft.fft(avg_time)))
        # redundancia en valor absoluto
        #s_mag = np.abs(sp) / self.sum_win_funct
        #s_mag = np.maximum(s_mag, 10 ** (-15))
        #s_dbfs = 20 * np.log10(s_mag / (2 ** 11))

        mat = np.vstack(avg_time) # shape (1, tamaño fft)
        
        # GUARDAR DATA .npy
        #save_dir = '/home/dammr/Desktop/UC_SmartFarmRadar/capturas_radar' # Carpeta donde guardar
        #os.makedirs(save_dir, exist_ok=True) # crea la carpeta si no existe
        #existing_files = [f for f in os.listdir(save_dir) if f.endswith('.npy')]
        #numbers = [int(f.replace('.npy', '')) for f in existing_files if f.replace('.npy', '').isdigit()]
        #next_number = max(numbers) + 1 if numbers else 0
        #save_path = os.path.join(save_dir, f"{next_number}.npy")
        #np.save(save_path, mat)
        #self.get_logger().info(f'Datos guardados en {save_path}')

        return mat

def main(args=None):
    rclpy.init(args=args)
    node = RadarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Nodo interrumpido por el usuario.")
    except Exception as e:
        print(f"Excepción no controlada: {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()