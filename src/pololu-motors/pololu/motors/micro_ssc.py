#
# pololu/motors/microSSC.py
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

class MicroSSC(object):
    """
    Pololu Micro Serial Servo Controller
    """
    _COMMAND = {
        'set-parameters': 0x00,
        'set-speed': 0x01,
        'set-position-7': 0x02,
        'set-position-8': 0x03,
        'set-target': 0x04,
        'set-neutral-home': 0x05,
    }
    DEFAULT_DEVICE_ID = 0x01

    def __init__(self, device, baud=38400, readTimeout=None, writeTimeout=0.1,
                 log=None):
        self._log = log
        self._device_numbers = []
        self._serial = serial.Serial(port=device, baudrate=baud,
                                     bytesize=serial.EIGHTBITS,
                                     parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE,
                                     timeout=readTimeout,
                                     writeTimeout=writeTimeout)

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

    def _writeData(self, command, device, servo, params=()):
        """
        Write the data to the device.

        :Parameters:
          command : `int`
            The command to write to the device.
          device : `int`
            The device is the integer number of the hardware device ID. Defaults to 0x01.
          servo : `int`
            Servo number. Pololu Micro SSC alllows only commands to specific servos
          params : `tuple`
            Sequence of bytes to write.

        :Exceptions:
          * `SerialTimeoutException`
            If the low level serial package times out.
          * `SerialException`
            IO error when the port is not open.
        """
        sequence = []

        sequence.append(0x80)
        sequence.append(device)
        sequence.append(command)
        sequence.append(servo)
        for param in params:
            sequence.append(param)

        self._serial.write(bytearray(sequence))
        self._log and self._log.debug("Wrote byte sequence: %s",
                                      [hex(num) for num in sequence])

    def _intToHighLow(self, val):
        """
        Converts the 2 byte value to 7 higher bits followed by low bits
        :parameters:
          val: : `int`
            2-byte number for position
        :returns:
            2 bytes with 7 upper bits plus 7 lower bits
        """
        return (((val >> 7) & 0x7f),(val & 0x7f),)

    def setSpeed(self, motor, speed, device=DEFAULT_DEVICE_ID):
        """
        This command limits the speed at which a servo channel's output value changes.

        :Parameters:
          motor : `int`
            Servo Id on the board
          speed : `int`
            Motor speed as an integer. 1 unit is 25 micro s/s. Max speed 6.35 ms/s
          device : `int`
            The device is the integer number of the hardware devices ID and
            is only used with the Pololu Protocol.
        """

        # Need to convert to the MicroSCC
        speed = int(min(speed/2, 127))
        self._writeData(
            self._COMMAND.get('set-speed'),
            device,
            motor,
            (speed,)
        )


    def setParameters(self, motor, on=1, reverse=0, range=15, device=DEFAULT_DEVICE_ID):
        """
        This sets parameters of the servo

        :Parameters:
          motor : `int`
            Servo Id on the board
          on : `int`
            Servo on/off
          reverse : `int`
            if reverse is 1 the pulse will be decreased with values sent
          range : `int`
            4 bits multiplier for 7 and 8 bits set-target command.
          device : `int`
            device number
        """
        data  =  ((on<<6)|(reverse<<5)|range)&0x7F
        self._writeData(
            self._COMMAND.get('set-parameters'),
            device,
            motor,
            data
        )

    def setTarget(self,motor, target, device=DEFAULT_DEVICE_ID):
        """
        Sets motor to the target
        :Parameters:
          motor: : `int`
            Servo id
          target: `int`
            pulse target in 1/4 micro s.
          device: `int`
            device number
        """
        # Micro SSC accepts target position in 1/2 micro s
        self._writeData(
            self._COMMAND.get('set-target'),
            device,
            motor,
            self._intToHighLow(target/2)
        )

    def setPosition7(self,motor, position, device=DEFAULT_DEVICE_ID):
        """
        Sets motor to the position with 5 byte command
        :Parameters:
          motor: : `int`
            Servo id
          position: `int`
            position for servo based on servo settings. Maximum value 127.
          device: `int`
            device number
        """

        self._writeData(
            self._COMMAND.get('set-position-7'),
            device,
            motor,
            min(127, position)
        )

    def setPosition8(self,motor, position, device=DEFAULT_DEVICE_ID):
        """
        Sets motor to the position with 6 byte command
        :Parameters:
          motor: : `int`
            Servo id
          position: `int`
            position for servo based on servo settings. Maximum value 255.
          device: `int`
            device number
        """

        self._writeData(
            self._COMMAND.get('set-position-8'),
            device,
            motor,
            self._intToHighLow(min(255, position))
        )

    def setNeutral(self,motor, position, device=DEFAULT_DEVICE_ID):
        """
        Sets neutral motor position. Will move servo to this position.
        :Parameters:
          motor: : `int`
            Servo id
          position: `int`
            pulse in 1/4 micro s
          device: `int`
            device number
        """

        self._writeData(
            self._COMMAND.get('set-neutral'),
            device,
            motor,
            self._intToHighLow(position)
        )

    def clean(self):
        self._serial.flushInput()