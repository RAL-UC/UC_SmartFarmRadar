import numpy as np

# Ruta al archivo .npy
ruta_archivo = 'radar_package/pos_0_0_angle_-30_0.npy'

# Cargar el archivo .npy
datos = np.load(ruta_archivo)

# Imprimir los datos
print("Datos cargados:")
print(datos)

# Información adicional (opcional)
print("\nInformación de la matriz:")
print(f"Shape: {datos.shape}")
print(f"Dtype: {datos.dtype}")