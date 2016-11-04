import logging
import functools

import rospy
import dynamic_reconfigure
import dynamic_reconfigure.client

logger = logging.getLogger('hr.tools.common.control')

def update_parameter(node, param):
    client = dynamic_reconfigure.client.Client(node)
    try:
        client.update_configuration(param)
    except dynamic_reconfigure.DynamicReconfigureParameterException as ex:
        logger.error("Updating {} parameter: {}".format(node, ex))
        return False
    return True

def chatbot(enable):
    node = 'chatbot'
    param = {'enable': enable}
    return update_parameter(node, param)

chatbot_on = functools.partial(chatbot, True)
chatbot_off = functools.partial(chatbot, False)

if __name__ == '__main__':
    rospy.init_node('test')
    print chatbot_on()
    rospy.sleep(1)
    print chatbot_off()
