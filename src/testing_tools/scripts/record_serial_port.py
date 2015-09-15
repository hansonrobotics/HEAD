import os
from testing_tools import PololuSerialReader

HR_WORKSPACE = os.environ.get('HR_WORKSPACE', os.path.expanduser('~/hansonrobotics'))
CWD = os.path.abspath(os.path.dirname(__file__))

device = os.path.expanduser('~/workspace/hansonrobotics/scripts/pololu1')
reader = PololuSerialReader(device, timeout=None)

try:
    ofile = os.path.join(CWD, 'yawn-1-serial-commands.csv')
    f = open(ofile, 'w')
    f.write('MotorID,Command,Value\n')
    while True:
        id, cmd, value = reader.read()
        f.write('%s,%s,%s\n' % (id, cmd, value))
        f.flush()
finally:
    f.close()
