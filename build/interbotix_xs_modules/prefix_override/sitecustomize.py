import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/install/interbotix_xs_modules'
