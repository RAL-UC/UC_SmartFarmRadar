# Importar el tipo Header
std_msgs/Header header

# RadarData.msg
uint32 rows       # número de filas (por ejemplo 161)
uint32 cols       # número de columnas (por ejemplo 4096)
string dtype      # tipo de dato, p.ej. "float64"
float64[] data    # datos aplanados fila‑por‑fila
