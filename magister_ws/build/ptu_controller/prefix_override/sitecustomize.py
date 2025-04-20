import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/diego/Desktop/magister/UC_SmartFarmRadar/magister_ws/install/ptu_controller'
