# Importar el tipo Header
std_msgs/Header header

# RadarData.msg
uint32 rows       # número de filas (por ejemplo 161)
uint32 cols       # número de columnas (por ejemplo 4096)
string dtype      # tipo de dato "float64" 
#float64[] data    # datos aplanados fila‑por‑fila

float64[] data_real 
float64[] data_imag

# pose del PTU al capturar
int32 pan_deg
int32 tilt_deg

uint32 bunker_pose_id

#uint32 start_offset_time 
#uint32 start_offset_samples  
