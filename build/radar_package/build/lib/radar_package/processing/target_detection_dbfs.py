'''
   target_detection_dbfs.py
   Original code from Marshall Bruner, Colorado State University
   https://github.com/brunerm99/ADI_Radar_DSP
   Modified by Jon Kraft to use dBFS values
'''

'''
    corrección a archivo de Jon Kraft
    Modificada/Adaptada: se corrige la comparacion de valores sobre el umbral para no considerar los valores absolutos
'''

import numpy as np
from scipy.interpolate import interp1d

# fa_rate: probabilidad de falsa alarma, tasa de falsa alarma va entre 0 y 1
# define qué tan dispuesto estás a tolerar que el ruido de fondo sea detectado por error como un blanco real
# valor mas alto implica: Umbral más bajo, aceptas que estadísticamente un 20% del ruido cruce la barrera
# valor mas bajo: Exiges mayor certeza matemática antes de confirmar un objetivo
# Probabilidad de detectar un blanco cuando en realidad solo hay ruido
# en aplicaciones reales se utilizan valores de aproximadamente
# 10**(-4) o 10**(-6) para evitar saturar la pantalla con falsas alarmas
# un valor mas alto es mas flexible facilitando detectar objetos de baja potencia

# target_detection_dbfs
# Detección de Objetivos expresada en Decibelios respecto a la Escala Completa


def cfar(X_k, num_guard_cells, num_ref_cells, bias=1, cfar_method='average', fa_rate=0.2):
    N = X_k.size # tamaño de los datos en rango
    cfar_values = np.ma.masked_all(X_k.shape) # cada casilla esta enmascarada
    noise_variance = None # se define solo si cfar_method == 'false_alarm'

    # considera solo el sub espacio donde existe simetria en el numero de celdas de guarda y referencia a ambos lados
    # omite detecciones en los bordes
    for center_index in range(num_guard_cells + num_ref_cells, N - (num_guard_cells + num_ref_cells)):
        min_index = center_index - (num_guard_cells + num_ref_cells) # inicio de la ventana de referencia izquierda
        min_guard = center_index - num_guard_cells # inicio de la ventana de guarda a la izquierda
        max_index = center_index + (num_guard_cells + num_ref_cells) + 1 # fin de la ventana de referencia a la derecha
        max_guard = center_index + num_guard_cells + 1 # fin de la ventana de guarda a la derecha

        # para obtener el promedio solo se consideran las celdas de referencia
        lower_nearby = X_k[min_index:min_guard] # vecinos a la izquierda
        upper_nearby = X_k[max_guard:max_index] # vecinos a la derecha

        # calculo de promedios
        lower_mean = np.mean(lower_nearby)
        upper_mean = np.mean(upper_nearby)

        # segun el metodo se hace un calculo diferente
        if (cfar_method == 'average'):
            mean = np.mean(np.concatenate((lower_nearby, upper_nearby))) # concatena las celdas de referencia superior e inferior
            output = mean + bias # aplica un promedio y le suma el bias
        elif (cfar_method == 'greatest'):
            mean = max(lower_mean, upper_mean)
            output = mean + bias # obtiene el valor maximo y le suma el bias
        elif (cfar_method == 'smallest'):
            mean = min(lower_mean, upper_mean)
            output = mean + bias # obtiene el valor minimo y le suma el bias
        elif (cfar_method == 'false_alarm'): # tasa de falsas alarmas
            refs = np.concatenate((lower_nearby, upper_nearby)) # training cells
            noise_variance = np.sum(refs**2 / refs.size) # varianza estimada del ruido
            # modelo de ruido Gaussiano (o Rayleigh para amplitudes)
            output = (noise_variance * -2 * np.log(fa_rate))**0.5
        else:
            raise Exception('No CFAR method received')

        cfar_values[center_index] = output # aplica el valor central a la mascara para obtener un arreglo de comparacion con el umbral

    # mascara ajustada al tamaño de cfar_values, el resto de valores que no se rellenaron anteriormente se les pone el minimo
    cfar_values[np.where(cfar_values == np.ma.masked)] = np.min(cfar_values)

    # crea una mascara del mismo tamaño de los datos de entrada
    targets_only = np.ma.masked_array(np.copy(X_k))

    # hace la comparacion en base a valores absolutos
    # targets_only[np.where(abs(X_k) > abs(cfar_values))] = np.ma.masked
    # oculta en el arreglo targets_only las posiciones donde la señal supera el umbral CFAR
    # se hace lo contrario a lo esperado
    targets_only[np.where(X_k > cfar_values)] = np.ma.masked

    # retorno de valores segun metodo de calculo
    if (cfar_method == 'false_alarm'):
        return cfar_values, targets_only, noise_variance
    else:
        return cfar_values, targets_only
    
# considera las celdas de referencia de forma asimetrica cuando se encuentra con un borde
def cfar_adaptive_edge(X_k, num_guard_cells, num_ref_cells, bias=1, cfar_method='average', fa_rate=0.2, logger=None):
    N = X_k.size # tamaño de los datos en rango
    cfar_values = np.ma.masked_all(X_k.shape) # cada casilla esta enmascarada
    noise_variance = None # se define solo si cfar_method == 'false_alarm'

    # Antes: range(num_guard_cells + num_ref_cells, N - (num_guard_cells + num_ref_cells))
    # dejaba sin evaluar las celdas de borde. Ahora se recorre TODO el vector,
    # y la ventana de referencia se recorta a lo que exista de cada lado.
    for center_index in range(N):
        min_index = max(0, center_index - (num_guard_cells + num_ref_cells)) # si el resultado es negativo se sale por la izquierda
        min_guard = max(0, center_index - num_guard_cells) # se limita a 0 para evitar indices negativos
        max_guard = min(N, center_index + num_guard_cells + 1) # evita que el índice supere el tamaño total de la señal N
        max_index = min(N, center_index + (num_guard_cells + num_ref_cells) + 1)

        # para obtener el promedio solo se consideran las celdas de referencia
        lower_nearby = X_k[min_index:min_guard] # vecinos a la izquierda
        upper_nearby = X_k[max_guard:max_index] # vecinos a la derecha

        # sin celdas de referencia disponibles de ningún lado -> no se evalúa esta celda
        #if lower_nearby.size == 0 and upper_nearby.size == 0:
        #    continue

        valid_refs = [arr for arr in (lower_nearby, upper_nearby) if arr.size > 0]
        if not valid_refs:
            continue

        refs = np.concatenate(valid_refs) # training celñs

        # calculo de promedios si la celda contiene valores
        #lower_mean = np.mean(lower_nearby) if lower_nearby.size > 0 else None
        #upper_mean = np.mean(upper_nearby) if upper_nearby.size > 0 else None

        # segun el metodo se hace un calculo diferente
        if (cfar_method == 'average'):
            mean = np.mean(refs)
            output = mean + bias # aplica un promedio y le suma el bias
        elif (cfar_method == 'greatest'):
            means = [np.mean(arr) for arr in valid_refs]
            mean = max(means)
            output = mean + bias # obtiene el valor maximo y le suma el bias
        elif (cfar_method == 'smallest'):
            means = [np.mean(arr) for arr in valid_refs]
            mean = min(means)
            output = mean + bias # obtiene el valor minimo y le suma el bias
        elif (cfar_method == 'false_alarm'): # tasa de falsas alarmas
            noise_variance = np.sum(refs**2 / refs.size) # varianza estimada del ruido
            # modelo de ruido Gaussiano (o Rayleigh para amplitudes)
            output = (noise_variance * -2 * np.log(fa_rate))**0.5
        else:
            raise Exception('No CFAR method received')

        cfar_values[center_index] = output # aplica el valor central a la mascara para obtener un arreglo de comparacion con el umbral

    #logger.info(f"{cfar_values.shape}")  

    # se cubre todo el vector para compararlo con el umbral
    # verifica si el arreglo del umbral tiene celdas ocultas o sin evaluar
    if np.ma.is_masked(cfar_values) and cfar_values.mask.any():
        if not cfar_values.mask.all(): # Comprueba que al menos una celda sí se haya podido calcular correctamente
            cfar_values[np.where(cfar_values.mask)] = np.min(cfar_values) # toma los huecos no evaluados y los rellena con el valor de umbral más bajo
        else:
            raise Exception('Ventana CFAR (guard+ref) mayor que el largo del vector: no hay ninguna celda evaluable')

     # crea una mascara del mismo tamaño de los datos de entrada
    targets_only = np.ma.masked_array(np.copy(X_k))

    # oculta en el arreglo targets_only las posiciones donde la señal supera el umbral CFAR
    # se hace lo contrario a lo esperado
    targets_only[np.where(X_k > cfar_values)] = np.ma.masked

    # retorno de valores segun metodo de calculo
    if (cfar_method == 'false_alarm'):
        return cfar_values, targets_only, noise_variance
    else:
        return cfar_values, targets_only