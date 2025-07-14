import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/himel/Recuitment_repo/Md-Ashfaqul-awal-22201868-autonomous/task2/src/turtle_control/install/turtle_control'
