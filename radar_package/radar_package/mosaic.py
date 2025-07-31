#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from radar_msg.msg import RadarData
from radar_package.target_detection_dbfs import cfar # objetivos de deteccion
from radar_package.parametros import *
import time


path_mapa_acumulado = "/home/dammr/Desktop/magister_ws/UC_SmartFarmRadar/datos/mapa_acumulado_20250731_155335.npy"
mapa_acumulado = np.load(path_mapa_acumulado)


def mosaico(mapa_acumulado, angle0: float = -90.0, fov: float = 160.0):
        total_rows, n_freq = mapa_acumulado.shape # 2093, 322 -> 1024/2 = 512 -> 585 step_freq (100000 + 11000)/585 = 189 -> 512 - 189 = 323 
        segment_size = 161 # 161
        n_segments = total_rows // segment_size # 13
        half_fov = fov / 2.0 # 80

        segmentos = mapa_acumulado.reshape((n_segments, segment_size, n_freq)) # separacion por segmentos
        
         # centros de cada barrido
        centros = angle0 + STEP_DEG_PTU * np.arange(N_MAPS) # lista de angulos del pantilt

        angulos_seg = [np.linspace(c - half_fov, c + half_fov, segment_size) for c in centros]

         # ángulos globales unificados (sin repeticiones)
        all_angles = np.concatenate(angulos_seg)
        global_angles = np.unique(all_angles)

        mapa_mosaico = np.zeros((GLOBAL_ANGLES, n_freq))

        # para cada frecuencia, interpolar y combinar
        for j in range(n_freq): # de 0 a 321
            for k, ga in enumerate(global_angles): # de -170 a 170
                pesos = []
                vals  = []
                # revisar cada segmento
                for i in range(n_segments): # de 0 a 12
                    ai = angulos_seg[i] # extrae los angulos de un segmento 
                    if ai[0] <= ga <= ai[-1]: # revisa si el angulo global esta dentro de los espacios del segmento
                        # interpolar valor del segmento en ga
                        idx = np.argmax(ai == ga)
                        val = segmentos[i, idx, j]
                        # peso según cercanía al centro
                        dist_rel = abs(ga - centros[i]) / half_fov
                        w = max(0.0, 1.0 - dist_rel)
                        pesos.append(w)
                        vals.append(val)
                if ga == 0 and j == 30:
                    print(pesos, vals)

                # caso 1: un solo valor → lo devolvemos tal cual
                if len(vals) == 1:
                    mapa_mosaico[k, j] = vals[0]

                # caso 2: varios valores → media ponderada con pesos normalizados
                elif len(vals) > 1:
                    pesos = np.array(pesos)
                    vals  = np.array(vals)
                    pesos_norm = pesos / pesos.sum()                # suma = 1
                    mapa_mosaico[k, j] = np.dot(pesos_norm, vals)

                # caso 3: ningún segmento cubre el ángulo
                else:
                    mapa_mosaico[k, j] = 0
            
        return mapa_mosaico


mapa_mosaico = mosaico(mapa_acumulado)

print(mapa_mosaico[170, 30])
print(max(0.0, 1.0 - abs(0 - -75) / 80), max(0.0, 1.0 - abs(0 - -60) / 80), max(0.0, 1.0 - abs(0 - -45) / 80), max(0.0, 1.0 - abs(0 - -30) / 80), max(0.0, 1.0 - abs(0 - -15) / 80), max(0.0, 1.0 - abs(0 - 0) / 80), max(0.0, 1.0 - abs(0 - 15) / 80), max(0.0, 1.0 - abs(0 - 30) / 80), max(0.0, 1.0 - abs(0 - 45) / 80), max(0.0, 1.0 - abs(0 - 60) / 80), max(0.0, 1.0 - abs(0 - 75) / 80))
print(mapa_acumulado[0+161+155, 30], mapa_acumulado[0+161+161+140, 30], mapa_acumulado[0+161+161+161+125, 30], mapa_acumulado[0+161+161+161+161+110, 30], mapa_acumulado[0+161+161+161+161+161+95, 30], mapa_acumulado[0+161+161+161+161+161+161+80, 30], mapa_acumulado[0+161+161+161+161+161+161+161+65, 30], mapa_acumulado[0+161+161+161+161+161+161+161+161+50, 30], mapa_acumulado[0+161+161+161+161+161+161+161+161+161+35, 30], mapa_acumulado[0+161+161+161+161+161+161+161+161+161+161+20, 30], mapa_acumulado[0+161+161+161+161+161+161+161+161+161+161+161+5, 30])

lista1 = [max(0.0, 1.0 - abs(0 - -75) / 80), max(0.0, 1.0 - abs(0 - -60) / 80), max(0.0, 1.0 - abs(0 - -45) / 80), max(0.0, 1.0 - abs(0 - -30) / 80), max(0.0, 1.0 - abs(0 - -15) / 80), max(0.0, 1.0 - abs(0 - 0) / 80), max(0.0, 1.0 - abs(0 - 15) / 80), max(0.0, 1.0 - abs(0 - 30) / 80), max(0.0, 1.0 - abs(0 - 45) / 80), max(0.0, 1.0 - abs(0 - 60) / 80), max(0.0, 1.0 - abs(0 - 75) / 80)]
lista2 = [mapa_acumulado[0+161+155, 30], mapa_acumulado[0+161+161+140, 30], mapa_acumulado[0+161+161+161+125, 30], mapa_acumulado[0+161+161+161+161+110, 30], mapa_acumulado[0+161+161+161+161+161+95, 30], mapa_acumulado[0+161+161+161+161+161+161+80, 30], mapa_acumulado[0+161+161+161+161+161+161+161+65, 30], mapa_acumulado[0+161+161+161+161+161+161+161+161+50, 30], mapa_acumulado[0+161+161+161+161+161+161+161+161+161+35, 30], mapa_acumulado[0+161+161+161+161+161+161+161+161+161+161+20, 30], mapa_acumulado[0+161+161+161+161+161+161+161+161+161+161+161+5, 30]]
lista1 = np.array(lista1)
lista2 = np.array(lista2)
lista1 = lista1 / lista1.sum()

print(np.dot(lista1, lista2))

def main(args=None):
    pass
if __name__ == '__main__':
    main()