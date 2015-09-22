#
# pololu/motors/maestro.py
#
# Usual device on Linux: /dev/ttyACM0
#

"""
This code was written to work with the Pololu Maestro motor controllers.

by Carl J. Nobile

THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
__docformat__ = "reStructuredText en"


import serial

from .crc7 import crc7


class Maestro(object):
    """
    Pololu Maestro controller for single device attached on USB Dual Port mode.
    """
    _BAUD_DETECT = 0xAA

    _COMMAND = {
        'set-target': 0x04,
        'set-multiple-targets': 0x1F,
        'set-speed': 0x07,
        'set-acceleration': 0x09,
        'get-position': 0x10,
        'get-errors': 0x21,
        'get-home':  0x22,
    }
    _ERRORS = {
        0: 'OK',
        1: "Serial Signal Error",
        2: "Serial Overrun Error",
        4: "Serial RX buffer full",
        8: "Serial CRC error (bit 3)",
        16: "Serial protocol error",
        32: "Serial timeout error",
        64: "Script stack error",
        128: "Script call stack error",
        256: "Script program counter error"
    }
    DEFAULT_DEVICE_ID = 0x0B

    def __init__(self, device, baud=115200, readTimeout=None, writeTimeout=None,
                 log=None):
        self._log = log
        self._device_numbers = []
        self._serial = serial.Serial(port=device, baudrate=baud,
                                     bytesize=serial.EIGHTBITS,
                                     parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE,
                                     timeout=readTimeout,
                                     writeTimeout=writeTimeout)
        self.setCompactProtocol()
        self._deviceConfig = {}
        self._crc = False



    def close(self):
        """
        Closes the serial connection.
        """
        if self._serial:
            self._serial.close()

    def isOpen(self):
        """
        Check if the serial connection is open.

        :Returns:
          If `True` the serial connction is open else if `False` it is closed.
        """
        return self._serial.isOpen()

    def setCompactProtocol(self):
        """
        Set the compact protocol.
        """
        self._compact = True
        # BAUD_DETECT only required on UART, detect baud rate mode.
        # self._serial.write(bytes(self._BAUD_DETECT))
        self._log and self._log.debug("Compact protocol has been set.")

    def isCompactProtocol(self):
        """
        Check if currently using the compact protocol.

        :Returns:
          if `True` the compact protocol is currently being used else if
          `False` it is not currently being used.
        """
        return self._compact == True

    def setPololuProtocol(self):
        """
        Set the pololu protocol.
        """
        self._compact = False
        self._log and self._log.debug("Pololu protocol has been set.")

    def isPololuProtocol(self):
        """
        Check if currently using the pololu protocol.

        :Returns:
          if `True` the pololu protocol is currently being used else if
          `False` it is not currently being used.
         """
        return self._compact == False

    def setCRC(self, value):
        """
        Enable or disable cyclic redundancy check.

        :Parameters:
          value : `bool`
            If `True` CRC is enabled else if `False` CRC is disabled.
        """
        self._crc = value

    def isCRC(self):
        """
        Check if CRC is enabled.

        :Returns:
          If `True` CRC is enabled else if `False` CRC is disabled.
        """
        return self._crc == True

    def _writeData(self, command, device, params=()):
        """
        Write the data to the device.

        :Parameters:
          command : `int`
            The command to write to the device.
          device : `int`
            The device is the integer number of the hardware devices ID and
            is only used with the Pololu Protocol.
          params : `tuple`
            Sequence of bytes to write.

        :Exceptions:
          * `SerialTimeoutException`
            If the low level serial package times out.
          * `SerialException`
            IO error when the port is not open.
        """
        sequence = []

        if self._compact:
            sequence.append(command | 0x80)
        else:
            sequence.append(self._BAUD_DETECT)
            sequence.append(device)
            sequence.append(command)

        for param in params:
            sequence.append(param)

        if self._crc:
            sequence.append(crc7(sequence))

        self._serial.write(bytearray(sequence))
        self._log and self._log.debug("Wrote byte sequence: %s",
                                      [hex(num) for num in sequence])

    def getError(self, message=True, device=DEFAULT_DEVICE_ID):
        """
        Get the error message or value stored in the Maestro hardware.

        :Parameters:
          message : `bool`
            If set to `True` a text message will be returned, if set to `False`
            the integer stored in the Maestro will be returned.
          device : `int`
            The device is the integer number of the hardware devices ID and
            is only used with the Pololu Protocol.

        :Returns:
          A list of text messages, integers, or and empty list. See the
          `message` parameter above.
        """
        cmd = self._COMMAND.get('get-errors')
        self._writeData(cmd, device)
        result = []
        bits = []

        try:
            num = self._serial.read(size=2)
            num = ord(num[1])*256+ord(num[0])
        except serial.SerialException as e:
            self._log and self._log.error("Error: %s", e, exc_info=True)
            raise e
        except TypeError as e:
            num = 0

        for i in range(8, -1, -1):
            bit = num & (1 << i)

            if bit:
                if message:
                    result.append(self._ERRORS.get(bit))
                else:
                    result.append(bit)

        return result

    def _intToLowHigh(self, val):
        """
        Converts the 2 byte value to 7 lower bits followed by high bits
        :parameters:
          val: : `int`
            Pololu data value. i.e motor value, speed
        :returns:
            2 bytes with 7 lower bits plus 7 high bits
        """
        return ((val & 0x7f), ((val >> 7) & 0x7f),)

    def setSpeed(self, motor, speed, device=DEFAULT_DEVICE_ID):
        """
        This command limits the speed at which a servo channel's output value changes.

        :Parameters:
          motor : `int`
            Servo Id on the board
          speed : `int`
            Motor speed as an integer. The speed limit is given in units of (0.25 micro s)/(10 ms)
          device : `int`
            The device is the integer number of the hardware devices ID and
            is only used with the Pololu Protocol.
        """
        self._writeData(
            self._COMMAND.get('set-speed'),
            device,
            (motor,) + self._intToLowHigh(speed)
        )

    def setAcceleration(self, motor, acceleration, device=DEFAULT_DEVICE_ID):
        """
        This command limits the speed at which a servo channel's output value changes.

        :Parameters:
          motor : `int`
            Servo Id on the board
          acceleration : `int`
            Motor speed as an integer. The speed limit is given in units of (0.25 micro s)/(10 ms)
          device : `int`
            The device is the integer number of the hardware devices ID and
            is only used with the Pololu Protocol.
        """
        # max acceleration is 256 units
        acceleration = min(256, acceleration)
        self._writeData(
            self._COMMAND.get('set-acceleration'),
            device,
            (motor,) + self._intToLowHigh(acceleration)
        )

    def setTarget(self,motor, target, device=DEFAULT_DEVICE_ID):
        """
        Sets motor to the target
        :Parameters:
          motor: : `int`
            Servo id
          target: `int`
            pulse target in 1/4 micro micro s.
          device: `int`
            device number, only for pololu protocol
        """

        self._writeData(
            self._COMMAND.get('set-target'),
            device,
            (motor,) + self._intToLowHigh(target)
        )

    def setMultipleTargets(self,motor, targets, device=DEFAULT_DEVICE_ID):
        """
        Sets motor to the target
        :Parameters:
          motor: : `int`
            Servo id
          targets: `array`
            pulses for  target motors, pulses start with the first motor_id passed by parameter
          device: `int`
            device number, only for pololu protocol
        """
        # format data
        data = (len(targets), motor,)
        for v in targets:
            data = data + self._intToLowHigh(v)

        self._writeData(
            self._COMMAND.get('set-multiple-targets'),
            device,
            data
        )

    def getHome(self, device=DEFAULT_DEVICE_ID):
        """
        Resets servos to home position
        :Parameters:
          device: `int`
            device number, only for pololu protocol
        """
        # forma
        self._writeData(
            self._COMMAND.get('get-home'),
            device,
            ()
        )

    def clean(self):
        self._serial.flushInput()

    @staticmethod
    def calculateSpeed(p1, p2, time, period=0.02):
        """
        Returns speed needed between two pulses based on servo channel setting
        more info https://www.pololu.com/docs/0J40/all#4.e
        :param p1: first pulse
        :param p2: second pulse
        :param time: time between pulses
        :param period: period in s of the servos, by default 0.02 (50Hz)
        :return: returns speed for maestro board
        """
        if period >= 0.02:
            speed = abs((p1-p2)/time)*(period/2)
        else:
            speed = abs((p1-p2)/time)*period
        return speed
