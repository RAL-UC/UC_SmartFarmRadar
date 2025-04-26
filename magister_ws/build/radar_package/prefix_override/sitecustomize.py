import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dammr/Desktop/UC_SmartFarmRadar/magister_ws/install/radar_package'
