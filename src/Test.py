import rospy
import roslib
from eva_behavior.msg import event
from std_msgs.msg import String

action_pub = rospy.Publisher("tracking_event", event, queue_size=1)
rospy.init_node("Test")
while True:
    event_name = event()
    e = raw_input("Event: ")
    if e == "new":
        event_name.event = "new_face"
    else:
        event_name.event = "exit"
    event_name.param = raw_input("ID: ")
    action_pub.publish(event_name)