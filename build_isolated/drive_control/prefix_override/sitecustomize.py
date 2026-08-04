import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hailab/osy/260711/ai-autonomous-driving-competition-2026/install_isolated/drive_control'
