import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ellimac-backup/Documents/halimatou/ros2_demos/servo/serial_motor_demo/install/serial_motor_demo'
