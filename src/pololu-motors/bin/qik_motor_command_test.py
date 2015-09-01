#!/usr/bin/env python
#
# bin/qik_motor_command_test.py
#

import sys, os, logging

BASE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, BASE_PATH)
#print sys.path

from pololu.motors import Qik2s9v1


def setupLogger(fullpath=None, level=logging.INFO):
    FORMAT = ("%(asctime)s %(levelname)s %(module)s %(funcName)s "
              "[line:%(lineno)d] %(message)s")
    logging.basicConfig(filename=fullpath, format=FORMAT, level=level)
    return logging.getLogger()


class MotorCommandTest(Qik2s9v1):
    _DEFAULT_TTY = '/dev/ttyUSB0'

    def __init__(self, tty=_DEFAULT_TTY, readTimeout=5, log=None):
        super(MotorCommandTest, self).__init__(tty, readTimeout=readTimeout,
                                               log=log)

    def start(self):
        self.setCompactProtocol()

        for cmd in range(0x80, 0x100):
            self._writeData(cmd, self.DEFAULT_DEVICE_ID)
            self._log.info("Command %s, results: %s",
                           hex(cmd | 0x80), self.getError())


if __name__ == '__main__':
    import traceback
    from datetime import datetime

    logPath = os.path.join(BASE_PATH, 'logs')
    logFilePath = os.path.join(logPath, 'qik2s9v1_command_test.log')
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
