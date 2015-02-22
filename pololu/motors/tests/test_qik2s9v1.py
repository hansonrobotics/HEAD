#!/usr/bin/env python
#
# motors/pololu/test_qik2s9v1.py
#

import os
import unittest
import time
import logging

from pololu.motors import Qik2s9v1


BASE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                         '..', '..', '..', 'logs'))

def setupLogger(fullpath=None, level=logging.DEBUG):
    FORMAT = ("%(asctime)s %(levelname)s %(module)s %(funcName)s "
              "[line:%(lineno)d] %(message)s")
    logging.basicConfig(filename=fullpath, format=FORMAT, level=level)
    return logging.getLogger()


class TestQik2s9v1(unittest.TestCase):
    """
    Unit tests for the Qik 2s9v1 motor controller.

    Most tests have a for loop which first tests the pololu protocol then tests
    the compact protocol.
    """
    _DEFAULT_TTY = '/dev/ttyUSB0'
    _PROTOCOL_MAP = {0: "Pololu Protocol", 1: "Compact Protocol"}

    def __init__(self, name):
        super(TestQik2s9v1, self).__init__(name)
        # Timeouts [(0.0, 0), (0.262, 1), ...]
        logFilePath = os.path.join(BASE_PATH, 'pololu_qik2s9v1.log')
        self._log = setupLogger(fullpath=logFilePath)

    def setUp(self):
        self._log.debug("Processing")
        self._qik = Qik2s9v1(self._DEFAULT_TTY, readTimeout=5, log=self._log)

    def tearDown(self):
        self._log.debug("Processing")

        if self._qik.isOpen():
            dConfig = self._qik._deviceConfig.copy()

            for d in dConfig:
                self._qik.setDeviceID(self._qik.DEFAULT_DEVICE_ID, device=d)

            self._qik.getError()
            self._qik.setPWMFrequency(31500)
            self._qik.setMotorShutdown(True)
            self._qik.setSerialTimeout(0.0)
            self._qik.setM0Speed(0)
            self._qik.setM0Coast()
            self._qik.setM1Speed(0)
            self._qik.setM1Coast()
            self._qik.close()
            self._qik = None

    def test_getConfigForDevice(self):
        self._log.debug("Processing")

        for device in self._qik._deviceConfig:
            config = self._qik.getConfigForDevice(device)
            self.assertTrue(config['version'] in (1, 2))
            self.assertTrue(config['pwm'] == 0)
            self.assertTrue(config['shutdown'] == 1)
            self.assertTrue(config['timeout'] == 0)

    def test_close(self):
        self._log.debug("Processing")
        self.assertTrue(self._qik.isOpen() == True)
        self._qik.close()
        self.assertTrue(self._qik.isOpen() == False)

    def test_setCompactProtocol(self):
        """
        The compact protocol is not the default.
        """
        self._log.debug("Processing")
        self.assertTrue(self._qik.isCompactProtocol() == False)
        self._qik.setCompactProtocol()
        self.assertTrue(self._qik.isCompactProtocol() == True)

    def test_setPololuProtocol(self):
        """
        The pololu portocol is the default.
        """
        self._log.debug("Processing")
        self.assertTrue(self._qik.isPololuProtocol() == True)
        self._qik.setCompactProtocol()
        self.assertTrue(self._qik.isPololuProtocol() == False)

    def test_getFirmwareVersion(self):
        self._log.debug("Processing")
        self.assertTrue(self._qik.getFirmwareVersion() in (1, 2))
        self._qik.setCompactProtocol()
        self.assertTrue(self._qik.getFirmwareVersion() in (1, 2))

    @unittest.skip("Temporarily skipped")
    def test_getError_DataOverrunError(self):
        self._log.debug("Processing")

    @unittest.skip("Temporarily skipped")
    def test_getError_FrameError(self):
        self._log.debug("Processing")

    @unittest.skip("Temporarily skipped")
    def test_getError_CRCError(self):
        self._log.debug("Processing")

    def test_getError_FormatError(self):
        self._log.debug("Processing")
        command = 0x70 # Bad command breaks in both protocols
        num = 64
        error = self._qik._ERRORS.get(num)

        for i in range(2):
            protocol = self._PROTOCOL_MAP.get(i)
            self._qik._writeData(command, self._qik.DEFAULT_DEVICE_ID)
            result = self._qik.getError(message=False)
            msg = "{}: Invalid error '{}' should be '{}'.".format(
                protocol, result, [num])
            self.assertTrue(num in result and len(result) == 1, msg=msg)
            self._qik._writeData(command, self._qik.DEFAULT_DEVICE_ID)
            result = self._qik.getError()
            msg = "{}: Invalid error '{}' should be '{}'.".format(
                protocol, result, [error])
            self.assertTrue(error in result and len(result) == 1, msg=msg)
            self._qik.setCompactProtocol()

    def test_getError_TimeoutError(self):
        self._log.debug("Processing")
        timeout = 0.262
        num = 128
        error = self._qik._ERRORS.get(num)

        for i in range(2):
            protocol = self._PROTOCOL_MAP.get(i)
            self._qik.setSerialTimeout(timeout)
            time.sleep(0.275)
            result = self._qik.getError(message=False)
            msg = "{}: Invalid error '{}' should be '{}'.".format(
                protocol, result, [num])
            self.assertTrue(num in result and len(result) == 1, msg=msg)
            self._qik.setSerialTimeout(timeout)
            time.sleep(0.275)
            result = self._qik.getError()
            msg = "{}: Invalid error '{}' should be '{}'.".format(
                protocol, result, [error])
            self.assertTrue(error in result and len(result) == 1, msg=msg)
            self._qik.setCompactProtocol()

    def test_getDeviceID(self):
        """
        Only test Pololu protocol, it's the only protocol that uses the device
        ID.
        """
        protocol = self._PROTOCOL_MAP.get(0)
        self._log.debug("Processing")
        devices = (self._qik.DEFAULT_DEVICE_ID, 127)
        # Default device
        result = self._qik.getDeviceID(device=devices[0])
        msg = "Invalid device '{}' should be '{}'.".format(
            protocol, result, devices[0])
        self.assertTrue(result == devices[0], msg=msg)
        # Change device
        self._qik.setDeviceID(devices[1], device=devices[0])
        result = self._qik.getDeviceID(device=devices[1])
        msg = "{}: Invalid device '{}' should be '{}'.".format(
            protocol, result, devices[1])
        self.assertTrue(result == devices[1], msg=msg)

    def test_getPWMFrequency(self):
        self._log.debug("Processing")

        for i in range(2):
            protocol = self._PROTOCOL_MAP.get(i)
            result = self._qik.getPWMFrequency()
            msg = "{}: Invalid PWM frequency '{}' should be '{}'.".format(
                protocol, result, self._qik._CONFIG_PWM.get(0)[1])
            self.assertTrue(result == self._qik._CONFIG_PWM.get(0)[1], msg=msg)
            result = self._qik.getPWMFrequency(message=False)
            msg = "{}: Invalid PWM frequency '{}' should be '{}'.".format(
                protocol, result, self._qik._CONFIG_PWM.get(0)[0])
            self.assertTrue(result == self._qik._CONFIG_PWM.get(0)[0], msg=msg)
            self._qik.setCompactProtocol()

    def test_getMotorShutdown(self):
        self._log.debug("Processing")

        for i in range(2):
            protocol = self._PROTOCOL_MAP.get(i)
            tf = self._qik.getMotorShutdown()
            msg = ("{}: Invalid motor shutdown value '{}' should be '{}'."
                   ).format(protocol, tf, True)
            self.assertTrue(tf == True, msg=msg)
            self._qik.setCompactProtocol()

    def test_getSerialTimeout(self):
        self._log.debug("Processing")

        for i in range(2):
            protocol = self._PROTOCOL_MAP.get(i)
            result = self._qik.getSerialTimeout()
            timeout = self._qik._valueToTimeout.get(0)
            msg = ("{}: Invalid serial timeout value '{}' should be '{}'."
                   ).format(protocol, result, timeout)
            self.assertTrue(result == timeout, msg=msg)
            self._qik.setSerialTimeout(200.0)
            result = self._qik.getSerialTimeout()
            timeout = self._qik._valueToTimeout.get(108)
            msg = ("{}: Invalid serial timeout value '{}' should be '{}'."
                   ).format(protocol, result, timeout)
            self.assertTrue(result == timeout, msg=msg)
            self._qik.setCompactProtocol()
            self._qik.setSerialTimeout(0.0)

    def test_setDeviceID(self):
        self._log.debug("Processing")
        devices = (self._qik.DEFAULT_DEVICE_ID, 127)

        for i in range(2):
            protocol = self._PROTOCOL_MAP.get(i)
            result = self._qik.setDeviceID(devices[1], devices[0])
            msg = ("{}: Invalid device ID '{}' should be '{}'.").format(
                protocol, result, 'OK')
            self.assertTrue(result == 'OK', msg=msg)
            result = self._qik.setDeviceID(devices[0], devices[1],
                                           message=False)
            msg = ("{}: Invalid device ID '{}' should be '{}'.").format(
                protocol, result, devices[0])
            self.assertTrue(result == 0, msg=msg)
            # Test that device has been properly changes in the stored config.
            msg = ("{}: Set device '{}' is not in stored device config "
                   ).format(protocol, result)
            self.assertTrue(devices[0] in self._qik._deviceConfig, msg=msg)
            self._qik.setCompactProtocol()

    def test_setPWMFrequency(self):
        self._log.debug("Processing")
        pwms = [v[0] for v in self._qik._CONFIG_PWM.values()]
        nums = dict([(v[0], k) for k, v in self._qik._CONFIG_PWM.items()])

        for i in range(2):
            protocol = self._PROTOCOL_MAP.get(i)

            for pwm in pwms:
                result = self._qik.setPWMFrequency(pwm)
                rtn = self._qik._CONFIG_RETURN.get(0)
                msg = ("{}: Invalid PM return text '{}' for PWM '{}', "
                       "should be '{}'.").format(protocol, result, pwm, rtn)
                self.assertTrue(result == rtn, msg=msg)
                num = self._qik.setPWMFrequency(pwm, message=False)
                msg = ("{}: Invalid PWM number '{}' for PWM '{}', "
                       "should be '{}'.").format(protocol, num, pwm, 0)
                self.assertTrue(num == 0, msg=msg)
                # Test the stored device config
                freq = self._qik.getPWMFrequency(message=False)
                num = self._qik._CONFIG_PWM_TO_VALUE.get(freq)
                config = self._qik.getConfigForDevice(
                    self._qik.DEFAULT_DEVICE_ID)
                cnum = config.get('pwm')
                msg = ("{}: Invalid PWM number '{}' for PWM '{}', "
                       "in stored config, should be '{}'").format(
                    protocol, num, pwm, cnum)
                self.assertTrue(num == cnum, msg=msg)

            self._qik.setCompactProtocol()

    def test_setMotorShutdown(self):
        self._log.debug("Processing")
        command = 0x70 # Bad command breaks in both protocols
        error = self._qik._ERRORS.get(64) # Format error
        rtn = self._qik._CONFIG_RETURN.get(0)

        for i in range(2):
            protocol = self._PROTOCOL_MAP.get(i)
            # Start with default motor stop on errors
            # Start up motor M0
            self._qik.setM0Speed(50)
            tf = self._qik.getMotorShutdown()
            msg = ("{}: Invalid motor shutdown value '{}' should be '{}'."
                   ).format(protocol, tf, True)
            self.assertTrue(tf == True, msg=msg)
            time.sleep(0.5)
            # Create an error condition that will stop the motors.
            self._qik._writeData(command, self._qik.DEFAULT_DEVICE_ID)
            result = self._qik.getError()
            msg = ("{}: Invalid response error text '{}' should be '{}'."
                   ).format(protocol, result, [error])
            self.assertTrue(error in result and len(result) == 1, msg=msg)

            # Switch to non-stopping motors
            text = self._qik.setMotorShutdown(False)
            msg = "{}: Invalid response '{}' should be '{}'.".format(
                protocol, text, rtn)
            self.assertTrue(text == rtn, msg=msg)
            tf = self._qik.getMotorShutdown()
            msg = ("{}: Invalid motor shutdown value '{}' should be '{}'."
                   ).format(protocol, text, False)
            self.assertTrue(text == rtn, msg=msg)

            # Test the stored device config
            config = self._qik.getConfigForDevice(self._qik.DEFAULT_DEVICE_ID)
            shutdown = config.get('shutdown')
            msg = ("{}: Invalid motor shutdown value '{}' in stored config, "
                   "should be '{}'.").format(protocol, tf, bool(shutdown))
            self.assertTrue(tf == bool(shutdown), msg=msg)

            # Start up motor M0, If change motor shutdown, need to come to
            # full stop. HAVING TO DO THIS MAY BE BECAUSE OF A BUG SOMEWHERE.
            self._qik.setM0Speed(0)
            self._qik.setM0Speed(-50)
            tf = self._qik.getMotorShutdown()
            msg = ("{}: Invalid motor shutdown value '{}' should be '{}'."
                   ).format(protocol, tf, False)
            self.assertTrue(tf == False, msg=msg)
            time.sleep(0.5)
            # Create an error condition that will not stop the motors.
            self._qik._writeData(command, self._qik.DEFAULT_DEVICE_ID)
            result = self._qik.getError()
            msg = ("{}: Invalid response error text '{}' should be '{}'."
                   ).format(protocol, result, [error])
            self.assertTrue(error in result and len(result) == 1, msg=msg)
            self._qik.setM0Speed(0)
            self._qik.setM0Coast()

            # Test the stored device config
            config = self._qik.getConfigForDevice(self._qik.DEFAULT_DEVICE_ID)
            shutdown = config.get('shutdown')
            msg = ("{}: Invalid motor shutdown value '{}' in stored config, "
                   "should be '{}'.").format(protocol, bool(shutdown), tf)
            self.assertTrue(tf == bool(shutdown), msg=msg)

            # Switch back to stopping motor.
            result = self._qik.setMotorShutdown(True)
            msg = "{}: Invalid response '{}' should be '{}'.".format(
                protocol, result, rtn)
            self.assertTrue(result == rtn, msg=msg)
            self._qik.setCompactProtocol()

    def test_setSerialTimeout(self):
        self._log.debug("Processing")
        rtn = self._qik._CONFIG_RETURN.get(0)
        shortDelay = self._qik._valueToTimeout.get(1)
        longDelay = self._qik._valueToTimeout.get(127)

        for i in range(2):
            protocol = self._PROTOCOL_MAP.get(i)
            # Test short timeout, will be 0.262 on the Qik
            result = self._qik.setSerialTimeout(0.3)
            msg = ("{}: Invalid serial timeout '{}' should be '{}'."
                   ).format(protocol, result, rtn)
            self.assertTrue(result == rtn, msg=msg)
            result = self._qik.getSerialTimeout()
            msg = ("{}: Invalid serial timeout value '{}' should be '{}'."
                   ).format(protocol, result, shortDelay)
            self.assertTrue(result == shortDelay, msg=msg)

            # Test the stored device config
            config = self._qik.getConfigForDevice(self._qik.DEFAULT_DEVICE_ID)
            timeout = config.get('timeout')
            msg = ("{}: Invalid motor timeout value '{}' in stored config, "
                   "should be '{}'.").format(protocol, result, timeout)
            self.assertTrue(result == timeout, msg=msg)

            # Test long timeout, will be 503.04 on the Qik
            result = self._qik.setSerialTimeout(500.0)
            msg = ("{}: Invalid serial timeout '{}' should be '{}'."
                   ).format(protocol, result, rtn)
            self.assertTrue(result == rtn, msg=msg)
            result = self._qik.getSerialTimeout()
            msg = ("{}: Invalid serial timeout value '{}' should be '{}'."
                   ).format(protocol, result, longDelay)
            self.assertTrue(result == longDelay, msg=msg)

            # Test the stored device config
            config = self._qik.getConfigForDevice(self._qik.DEFAULT_DEVICE_ID)
            timeout = config.get('timeout')
            msg = ("{}: Invalid motor timeout value '{}' in stored config, "
                   "should be '{}'.").format(protocol, result, timeout)
            self.assertTrue(result == timeout, msg=msg)

            self._qik.setCompactProtocol()

    @unittest.skip("Skipped, no return values.")
    def test_setM0Coast(self):
        self._log.debug("Processing")

        for i in range(2):
            self._qik.setM0Speed(50)
            time.sleep(0.5)
            self._qik.setM0Coast()

    @unittest.skip("Skipped, no return values.")
    def test_setM1Coast(self):
        self._log.debug("Processing")

        for i in range(2):
            self._qik.setM1Speed(50)
            time.sleep(0.5)
            self._qik.setM1Coast()

    @unittest.skip("Skipped, no return values and no get speed command.")
    def test_setM0Speed(self):
        self._log.debug("Processing")

        for i in range(2):
            self._qik.setM0Speed(50)
            time.sleep(0.5)

    @unittest.skip("Skipped, no return values and no get speed command.")
    def test_setM1Speed(self):
        self._log.debug("Processing")

        for i in range(2):
            self._qik.setM1Speed(50)
            time.sleep(0.5)


if __name__ == '__main__':
    unittest.main()
