import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/autonomous_explorer_ws_new/install/autonomous_explorer'
