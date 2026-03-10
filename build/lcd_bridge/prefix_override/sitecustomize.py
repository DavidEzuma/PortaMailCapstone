import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/davidezuma/PortaMailCapstone/install/lcd_bridge'
