import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dammr/Desktop/magister_ws/UC_SmartFarmRadar/install/ptu_routine'
