#!/usr/bin/env python
import rospy
import logging
import webui.srv as srv
import dynamic_reconfigure.client
import json

logger = logging.getLogger('hr.webui.node_configuration')


class NodeConfiguration:
    def __init__(self):
        logger.info('Starting configuration node')
        rospy.init_node('node_configuration')
        rospy.Service('get_configurable_nodes', srv.ConfigurableNodes, self.get_configurable_nodes)
        rospy.Service('get_description', srv.NodeDescription, self.get_description)
        rospy.Service('get_configuration', srv.NodeConfiguration, self.get_configuration)
        rospy.spin()

    def get_configurable_nodes(self, request):
        """
        Returns an array of running configurable nodes in json

        :param request:
        :return:
        """
        logger.info('get_configurable_nodes service call received')
        return srv.ConfigurableNodesResponse(self.configurable_nodes)

    def get_description(self, request):
        """
        Returns description of node properties with types in json

        :param request:
        :return:
        """
        node = request.node
        configurable_nodes = self.configurable_nodes

        if node in configurable_nodes:
            logger.info('get_node_description service call received for node: ' + node)
            client = dynamic_reconfigure.client.Client(node)
            return srv.NodeDescriptionResponse(json.dumps(client.get_parameter_descriptions()))
        else:
            logger.info('get_node_description service: Invalid node name')
            return srv.NodeDescriptionResponse(json.dumps({}))

    def get_configuration(self, request):
        """
        Returns node configuration with values in json

        :param request:
        :return:
        """
        node = request.node
        configurable_nodes = self.configurable_nodes

        if node in configurable_nodes:
            logger.info('get_node_configuration service call received for node: ' + node)
            client = dynamic_reconfigure.client.Client(node)
            return srv.NodeDescriptionResponse(json.dumps(client.get_configuration()))
        else:
            logger.info('get_node_configuration service: Invalid node name')
            return srv.NodeDescriptionResponse(json.dumps({}))

    @property
    def configurable_nodes(self):
        return dynamic_reconfigure.find_reconfigure_services()


if __name__ == '__main__':
    talker = NodeConfiguration()
