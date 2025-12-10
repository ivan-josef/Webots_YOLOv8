import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ivan/Webots_YOLOv8/install/Webots_YOLOv8'
