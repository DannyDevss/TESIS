import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dannydevss/Escritorio/TESIS/install/ugv_policy'
