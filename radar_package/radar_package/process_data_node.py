#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from radar_msg.msg import RadarData
from radar_package.target_detection_dbfs import cfar

path_base_data = "/home/dammr/Desktop/magister_ws/UC_SmartFarmRadar/datos/infinito1.npy"
class RadarDataSubscriber(Node):
    def __init__(self):
        super().__init__('subscribe_radar_data')
        self.create_subscription(
            RadarData,
            'radar_data',
            self.listener_callback,
            10
        )
        # Inicializar ventana de Matplotlib
        plt.ion()  # Modo interactivo
        self.fig, (self.ax_mag, self.ax_det) = plt.subplots(1, 2, figsize=(12, 5))
        self.im = None
        self.cbar = None

        self.det_plot = None
        
        self.base_data = np.load(path_base_data) 

        # Nueva figura para CFAR vs señal
        self.fig2, self.ax2 = plt.subplots(figsize=(6, 4))
        self.ax2.set_xlabel("Range [m]")
        self.ax2.set_ylabel("Magnitud [dBFS]")
        self.ax2.set_title("Datos vs Umbral CFAR")
        self.ax2.grid(True)
        self.line_sig, = self.ax2.plot([], [], label="Magnitud FFT")
        self.line_thr, = self.ax2.plot([], [], label="Umbral CFAR")
        self.scatter_det = self.ax2.scatter([], [], c='r', s=20, label="Detecciones")
        self.ax2.legend(loc="upper right")      


    def listener_callback(self, msg: RadarData):
        # Mostrar Header
        self.get_logger().info(
            f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, id="{msg.header.frame_id}"'
        )
        # Reconstruir numpy array
        arr = np.array(msg.data, dtype=msg.dtype)
        arr = arr.reshape((msg.rows, msg.cols))
        # Mostrar info
        #self.get_logger().info(
        #    f'Recibido RadarData: {arr.shape}, dtype={arr.dtype}'
        #)
        # Muestra los primeros 5 valores
        flat = arr.flatten()
        preview = ', '.join(f'{v:.2f}' for v in flat[:5])
        #self.get_logger().info(f'Primeros 5 valores: [{preview}]')

        arr = arr - self.base_data
        arr_min = np.min(arr)
        arr_max = np.max(arr)

        if arr_max > arr_min:
            arr = (arr - arr_min) / (arr_max - arr_min)
        else:
            arr = np.zeros_like(arr)

        # ------- PARAMETROS -------
        fft_size = 1024 * 4
        sample_rate = 0.6e6   # Tasa de muestreo (600 kHz)
        plot_freq = 100e3      # Rango total de frecuencia a trazar (Hz)
        signal_freq = 100e3    # Frecuencia de referencia (beat center) en Hz
        # ------- ---------- -------
        offset = 10.76e3
        freq = np.linspace(-sample_rate / 2, sample_rate / 2, int(fft_size))
        freq_step = freq[1] - freq[0]
        freq_offset_index = int(offset / freq_step)

        # Convertir el eje de frecuencias a rango
        # --------------------------
        c = 3e8                  # Velocidad de la luz (m/s)
        BW = 500e6               # Ancho de banda del chirp (Hz)
        ramp_time = 500e-6       # Tiempo de rampa en segundos (500 us)
        slope = BW / ramp_time   # Pendiente de la rampa (Hz/s)

        # Calcular el beat como diferencia con signal_freq y convertirlo a rango
        beat_freq = freq - signal_freq - offset
        range_axis = (beat_freq * c) / (2 * slope)

        valid_indices = np.where((range_axis >= 0))[0]

        filtered_freq = freq[valid_indices]
        filtered_range_axis = range_axis[valid_indices]
        filtered_fft_data = arr[:, valid_indices]

        angles = np.arange(-80, 81, 1)

        distances = np.full((len(angles), len(filtered_range_axis)), np.nan)
        #all_detected_angles = []
        #all_detected_distances = []

        for i, mag in enumerate(filtered_fft_data):
            thresh, targets = cfar(mag, num_guard_cells=5, num_ref_cells=20, bias=0.1, cfar_method='average')
            det_indices = np.where(targets.mask)[0] # posiciones no enmascaradas
            #print(det_indices)
            #all_detected_angles.append(angles[i])
            #all_detected_distances.append(mag[det_indices])
            
            distances[i, det_indices] = mag[det_indices]
            #for idx in det_indices:
                #all_detected_angles.append(angles[i])  # El ángulo correspondiente
                #all_detected_distances.append(mag[idx])  # La distancia detectada
            #    distances[i, idx] = mag[idx]
                #print(i, idx, mag[idx])

            if i == 80:
                # 3) Actualiza las curvas en la figura 2
                # Señal original X_k vs distancia
                self.line_sig.set_data(filtered_range_axis, mag)
                # Umbral CFAR vs distancia
                self.line_thr.set_data(filtered_range_axis, thresh)
                # Puntos de detección
                self.scatter_det.set_offsets(
                    np.c_[filtered_range_axis[det_indices], mag[det_indices]]
                )

                # 4) Ajusta límites si han cambiado
                self.ax2.set_xlim(filtered_range_axis[0], filtered_range_axis[-1])
                # opcional: fijar ylim fijo o basado en min/max de X_k y cfar
                ymin = min(np.min(mag), np.min(thresh))
                ymax = max(np.max(mag), np.max(thresh))
                self.ax2.set_ylim(ymin, ymax)

                # 5) Refresca la figura 2
                self.fig2.canvas.draw_idle()

        self.ax_det.clear()
        # Actualizar el gráfico de detecciones
        self.im_det = self.ax_det.imshow(
            distances.T,   # TRANSPOSE: porque primero es rango (eje y), luego ángulo (eje x)
            aspect='auto',
            origin='lower',
            extent=[angles[0], angles[-1], filtered_range_axis[0], filtered_range_axis[-1]],
            cmap='jet'
        )

        # Etiquetas
        self.ax_det.set_xlabel("Angle [°]")
        self.ax_det.set_ylabel("Range [m]")
        self.ax_det.set_title("CFAR Detections Map")
        #self.ax_det.grid(True)

        # grafico
        data = filtered_fft_data.T   # ahora shape [n_rng, n_ang]
        extent = [
            angles[0], angles[-1],              # xmin, xmax
            filtered_range_axis[0],             # ymin
            filtered_range_axis[-1]             # ymax
        ]

        if self.im is None:
            self.im = self.ax_mag.imshow(
                data,
                aspect='auto',
                extent=extent,
                origin='lower',
                cmap='jet'
            )
            # etiquetas y colorbar solo una vez
            self.ax_mag.set_xlabel("Angle [°]")
            self.ax_mag .set_ylabel("Range [m]")
            self.cbar = self.fig.colorbar(self.im, ax=self.ax_mag, label="Magnitude [dBFS]")
        else:
            # 3) en las siguientes iteraciones actualizamos
            self.im.set_data(data)
            self.im.set_extent(extent)
            # actualizar el rango de la escala de color
            self.im.set_clim(vmin=np.min(data), vmax=np.max(data))

        # 4) ajustamos límites de ejes si es necesario
        #self.ax.set_xlim(angles[0], angles[-1])
        #self.ax.set_ylim(filtered_range_axis[0], filtered_range_axis[-1])

        # 5) refrescamos
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = RadarDataSubscriber()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # Escucha un poco y sigue
            plt.pause(0.001)  # Permite actualizar la ventana matplotlib
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()