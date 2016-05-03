#!/usr/bin/env python
#
# bin/maestro_motor_command_test.py
#

import sys, os, logging

BASE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, BASE_PATH)
#print sys.path

from pololu.motors import Maestro


def setupLogger(fullpath=None, level=logging.INFO):
    FORMAT = ("%(asctime)s %(levelname)s %(module)s %(funcName)s "
              "[line:%(lineno)d] %(message)s")
    logging.basicConfig(filename=fullpath, format=FORMAT, level=level)
    return logging.getLogger()


class MotorCommandTest(Maestro):
    _DEFAULT_TTY = '/dev/ttyACM0'

    def __init__(self, tty=_DEFAULT_TTY, readTimeout=5, log=None):
        super(MotorCommandTest, self).__init__(tty, readTimeout=readTimeout,
                                               log=log)

    def start(self):
        # Simple test cases. Can be checked with pololu maestro control center.
        self.setCompactProtocol()
        self.getHome()
        # Protocol error
        self.setMultipleTargets(100,[4000,4100,4200,4300,4400,8500,5000,4580,7878,6895])
        e = self.getError()
        log.info(e)
        self.setMultipleTargets(5,[4000,4100,4200,4300,4400,8500,5000,4580,7878,4895])
        self.setAcceleration(0,1)
        self.setSpeed(0,100)
        self.setTarget(0,8000)
        self.setSpeed(1,1)
        self.setTarget(1,4000)

if __name__ == '__main__':
    import traceback
    from datetime import datetime

    logPath = os.path.join(BASE_PATH, 'logs')
    logFilePath = os.path.join(logPath, 'maestro_command_test.log')
    log = setupLogger(fullpath=logFilePath)
    startTime = datetime.now()

    try:
        log.info("Motor command test starting at %s", startTime)
        mct = MotorCommandTest(log=log)
        mct.start()
        endTime = datetime.now()
        log.info("Motor command test finished at %s elapsed time %s",
                 endTime, endTime - startTime)
    except Exception as e:
        tb = sys.exc_info()[2]
        traceback.print_tb(tb)
        print "%s: %s\n" % (sys.exc_info()[0], sys.exc_info()[1])
        sys.exit(1)

    sys.exit(0)
