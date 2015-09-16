import os
import time
from testing_tools import SerialPortRecorder, MessageQueue
from testing_tools.protocol.pololu import CompactProtocal
import rospy
import rostopic
from collections import OrderedDict
from blender_api_msgs.srv import GetParam, SetParam
from ros_pololu.msg import MotorCommand
from blender_api_msgs.msg import SetGesture
from pau2motors.msg import pau
from datetime import datetime
import pandas as pd

HR_WORKSPACE = os.environ.get('HR_WORKSPACE', os.path.expanduser('~/hansonrobotics'))
CWD = os.path.abspath(os.path.dirname(__file__))

device = os.path.expanduser('~/workspace/hansonrobotics/scripts/pololu1')
raw_data_file = os.path.join(CWD, 'out.raw')
recorder = SerialPortRecorder(device, raw_data_file)

rospy.init_node("record")

rospy.wait_for_service('/blender_api/get_param')
rospy.wait_for_service('/blender_api/set_param')
blender_get_param = rospy.ServiceProxy('/blender_api/get_param', GetParam)
shapekeys = eval(blender_get_param("self.getFaceData()").value)

blender_set_param = rospy.ServiceProxy('/blender_api/set_param', SetParam)
blender_set_param("bpy.context.scene['commandListenerActive']", "False")
time.sleep(2)
recorder.ser.flushInput()
recorder.ser.flushInput()

blender_set_param("bpy.context.scene['commandListenerActive']", "True")
recorder.start()
print "Start recording"
queue = MessageQueue()
queue.subscribe('/blender_api/get_pau', pau)
pub = rospy.Publisher('/blender_api/set_gesture', SetGesture, latch=True)
pub.publish(SetGesture('yawn-1', 1, 1, 1))
timestamps = OrderedDict()
timestamps['publish_set_gesture_msg'] = time.time()

current_gestures = None
while current_gestures is None:
    current_gestures = eval(blender_get_param("self.getGestures()").value)
    time.sleep(0.01)
timestamps['blender_play_gestures'] = time.time()

motor_msg = rospy.wait_for_message('/sophia/safe/head/command', MotorCommand)
timestamps['motor_command_arrive'] = time.time()


time.sleep(9)
blender_set_param("bpy.context.scene['commandListenerActive']", "False")
recorder.stop()
print "Stop recording"
pau_messages = queue.tolist()
pau_df = pd.DataFrame(columns = shapekeys.keys())
for pau_message in pau_messages:
    pau_df.loc[len(pau_df)] = pau_message.m_coeffs
pau_file = os.path.join(CWD, 'pau.csv')
pau_df.to_csv(pau_file, index=False)
print "Write pau message to %s" % pau_file

timestamps['first_serial_port_data'] = recorder.start_time
timestamps['last_serial_port_data'] = recorder.stop_time
for k, v in timestamps.items():
    print k, datetime.fromtimestamp(v)

parser = CompactProtocal()
with open(raw_data_file) as f:
    data = f.read()
    instructions = parser.parse(data)

ofile = os.path.join(CWD, 'yawn-1-serial-commands.csv')
with open(ofile, 'w') as f:
    f.write('MotorID,Command,Value\n')
    for i in instructions:
        cmd, id, value = i
        f.write('%s,%s,%s\n' % (id, cmd, value))
print "Write serial data to %s" % ofile

os.remove(raw_data_file)
