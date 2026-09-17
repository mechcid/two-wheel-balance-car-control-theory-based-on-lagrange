import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mechcid/lagarain/ros2_ws/install/balance_yaw'
