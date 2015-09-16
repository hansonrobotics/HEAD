import os
import time
from testing_tools import SerialPortRecorder
from testing_tools.protocol.pololu import CompactProtocal
import rospy
import rostopic
from blender_api_msgs.srv import SetParam

HR_WORKSPACE = os.environ.get('HR_WORKSPACE', os.path.expanduser('~/hansonrobotics'))
CWD = os.path.abspath(os.path.dirname(__file__))

device = os.path.expanduser('~/workspace/hansonrobotics/scripts/pololu1')
raw_data_file = os.path.join(CWD, 'out.raw')
recorder = SerialPortRecorder(device, raw_data_file)

rospy.wait_for_service('/blender_api/set_param')
blender_set_param = rospy.ServiceProxy('/blender_api/set_param', SetParam)
blender_set_param("bpy.context.scene['commandListenerActive']", "False")
time.sleep(2)
os.system('cat < %s >/dev/null' % device) # clear buffer
blender_set_param("bpy.context.scene['commandListenerActive']", "True")
recorder.start()
print "Start recording"
pub, msg_class = rostopic.create_publisher(
    '/blender_api/set_gesture',
    'blender_api_msgs/SetGesture', True)
pub.publish(msg_class('yawn-1', 1, 1, 1))
print "Start animation"

time.sleep(9)
blender_set_param("bpy.context.scene['commandListenerActive']", "False")
recorder.stop()
print "Stop recording"

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

os.remove(raw_data_file)
