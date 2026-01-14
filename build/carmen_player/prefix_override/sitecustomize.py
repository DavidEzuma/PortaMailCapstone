import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/c/Users/David/onedrive/Documents/Projects/PortaMailCapstone/install/carmen_player'
