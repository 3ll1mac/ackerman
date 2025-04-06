import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/halimatou-cisse/ros2_demos/ros2_control_demos/install/ros2_control_demo_testing'
