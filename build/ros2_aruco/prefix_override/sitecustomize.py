import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lolsondre/dat160_semesterproject/install/ros2_aruco'
