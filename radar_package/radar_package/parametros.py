SDR_URI = "ip:phaser.local:50901"
PHASER_URI = "ip:phaser.local"
ELEMENT_SPACING = 0.014 # espaciamiento de los parches en el phaser en metros

SAMPLE_RATE = 0.6e6 # frecuencia de muestreo

# frecuencias
CENTER_FREQ = 2.2e9 # 2.2 GHz frecuencia de operacion PlutoSDR
SIGNAL_FREQ = 100e3 # frecuencia de la señal transmitida 
OUTPUT_FREQ = 10e9 
BANDWIDTH = 500e6 # ancho de banda del chirp
NUM_STEPS = 500 # numero de pasos en que se dividira el chirp
RAMP_TIME = 500 # duracion de cada chirp en us, se aconseja tener un paso por us
RAMP_TIME_S = int(RAMP_TIME)/1e6
C = 3*10e8 # velocidad de la luz m/s
WAVELENGTH = C / OUTPUT_FREQ # longitud de onda
SLOPE = BANDWIDTH / RAMP_TIME_S # variación de la frecuencia por unidad de tiempo (ancho de banda total) / (duración de la rampa)s

RX_GAIN_CHAN0 = 30 # ganancia en la recepcion valor entre -3 y 70 CANAL 0
RX_GAIN_CHAN1 = 30 # ganancia en la recepcion valor entre -3 y 70 CANAL 1

TX_GAIN_CHAN0 = -88 # ganancia en la recepcion valor entre 0 y -88 CANAL 0 -> Tx1
TX_GAIN_CHAN1 = 0 # ganancia en la recepcion valor entre 0 y -88 CANAL 1 -> Tx2

# Intervalo de Repetición de Pulso (PRI) -> tiempo entre chirridos
# cada cuánto ocurrirá el ciclo TX+RX+espera -> cada cuadro coincide con un chirrido
PRI = RAMP_TIME/1e3 + 1.0 # se define en milisegundos y se le suma 1 ms como margen de separación entre chirridos
NUM_CHIRPS = 1 # solo un chirrido por cuadro

# se define en segundos y se ignora el 10%
PERCENT_IGNORE_SAMPLES = 0.1
BEGIN_OFFSET_TIME = PERCENT_IGNORE_SAMPLES * RAMP_TIME_S # tiempo offset para el comienzo de la captura de datos para evitar no linealidades, transitorio o desfase
GOOD_RAMP_SAMPLES = int((RAMP_TIME_S-BEGIN_OFFSET_TIME) * SAMPLE_RATE) # cuántas muestras "buenas" se pueden obtener después del offset}

# se utiliza desplazamiento de fase en un rango pequeño en base a la frecuencia central
# introduce un error el cual es pequeño 
SIGNAL_FREQ_PHASER_RECEPTION = 10.25e9 # 10.25 GHz el phaser escucha entre 10GHz y 10.5GHz
