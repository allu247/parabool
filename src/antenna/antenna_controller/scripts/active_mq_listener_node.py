#!/usr/bin/env python
__author__ = 'Mart Hytt'

from utils.validations import *
from utils.ros_exception import RosNodeException
from constants.action_constants import ActionConstants
from constants.topic_constants import TopicConstants
from constants.antenna_constants import AntennaConstants
from stomp import Connection, ConnectionListener
from antenna_msgs.msg import Flyover_request
from antenna_msgs.msg import Communication_package
from antenna_msgs.msg import UTF_antenna_command
from antenna_msgs.msg import Satellite_manager_command
from antenna_msgs.msg import Response
import rospy
import json
import time
import datetime


class ActiveMQConnectionListener(ConnectionListener):

    def __init__(self, node):
        self.node = node

    def on_message(self, headers, message):
        request = json.loads(message)
        rospy.loginfo(str(datetime.datetime.now()) + ': ActiveMQConnectionListener received request: ' + request)
        try:
            if request['node'] == ActionConstants.NODE_SATELLITE_MANAGE:
                validate_satellite_manager(request)
                self.node.publish_to_satellite_manager(request)
                return

            if request['node'] == ActionConstants.NODE_PUBLIC_CALC:
                validate_public_calc(request)
                self.node.publish_to_public_calc(request)
                return

            if request['node'] == ActionConstants.NODE_COMMUNICATION:
                validate_communication(request)
                self.node.publish_to_communication(request)
                return

            if request['node'] == ActionConstants.NODE_UTF_CONTROLLER:
                validate_utf_controller(request)
                self.node.publish_to_public_utf_antenna(request)
                return

            raise RosNodeException('Invalid node', request['request_id'])
        except RosNodeException as ex:
            rospy.logerr(str(datetime.datetime.now()) + ': request ID: ' + ex.request_id + ' ' + ex.message)
            self.node.publish_exception(ex.request_id, ex.message)

    def on_disconnected(self):
        super(ActiveMQConnectionListener, self).on_disconnected()
        print('ActiveMQConnectionListener:\n{}\n'.format(self))


class ActiveMQListener:
    def __init__(self):
        self.topic_public_calc = None
        self.topic_public_utf_antenna = None
        self.topic_communication = None
        self.topic_satellite_manager = None
        self.topic_active_mq_writer = None
        self.connection = None

    def __delete__(self, instance):
        if self.connection is not None:
            print('Closing MQ in connection')
            self.connection.disconnect()

    def publish_to_public_calc(self, message):
        try:
            request = Flyover_request()
            request.request_id = message['request_id']
            request.action = message['action']
            request.satellite_ids = message['data']['satellite_ids']
            request.long = message['data']['lon']
            request.lat = message['data']['lat']
            request.elevation = message['data']['elevation']

            rospy.loginfo(str(datetime.datetime.now()) + ': ActiveMQListener publish to public calc: ' + str(request))
            self.topic_public_calc.publish(request)
        except Exception as ex:
            rospy.logerr(str(datetime.datetime.now()) + ': request ID: ' + message['request_id'] + ' ' + str(ex))
            self.publish_exception(message['request_id'], str(ex))

    def publish_to_public_utf_antenna(self, message):
        try:
            request = UTF_antenna_command()
            request.request_id = message['request_id']
            request.action = message['action']

            if message['elevation'] is not None:
                request.elevation = message['elevation']

            if message['azimuth'] is not None:
                request.azimuth = message['azimuth']

            rospy.loginfo(str(datetime.datetime.now()) + ': ActiveMQListener publish to utf antenna: ' + str(request))
            self.topic_public_utf_antenna.publish(request)
        except Exception as ex:
            rospy.logerr(str(datetime.datetime.now()) + ': request ID: ' + message['request_id'] + ' ' + str(ex))
            self.publish_exception(message['request_id'], str(ex))

    def publish_to_communication(self, message):
        try:
            request = Communication_package()
            request.request_id = message['request_id']
            request.data = message['data']

            rospy.loginfo(
                str(datetime.datetime.now()) + ': ActiveMQListener publish to satellite communication: ' + str(request))
            self.topic_communication.publish(request)
        except Exception as ex:
            rospy.logerr(str(datetime.datetime.now()) + ': request ID: ' + message['request_id'] + ' ' + str(ex))
            self.publish_exception(message['request_id'], str(ex))

    def publish_to_satellite_manager(self, message):
        try:
            request = Satellite_manager_command()
            request.request_id = message['request_id']
            request.action = message['action']

            if message['satellite_ids'] is not None:
                request.satellite_ids = message['satellite_ids']

            rospy.loginfo(
                str(datetime.datetime.now()) + ': ActiveMQListener publish to satellite manager: ' + str(request))
            self.topic_satellite_manager.publish(request)
        except Exception as ex:
            rospy.logerr(str(datetime.datetime.now()) + ': request ID: ' + message['request_id'] + ' ' + str(ex))
            self.publish_exception(message['request_id'], [str(ex)])

    def publish_exception(self, request_id, message):
        response = Response()
        response.request_id = request_id
        response.ok = False
        response.data = [message]

        rospy.loginfo(str(datetime.datetime.now()) + ': ActiveMQListener publish exception: ' + str(response))
        self.topic_active_mq_writer.publish(response)

    def connect_to_mq(self):
        rospy.loginfo(str(datetime.datetime.now()) + ': Connecting to ActiveMQ')
        self.connection = Connection(
            host_and_ports=[(AntennaConstants.ACTIVE_MQ_HOST, AntennaConstants.ACTIVE_MQ_PORT)])
        self.connection.set_listener('my_listener', ActiveMQConnectionListener(self))
        self.connection.start()
        self.connection.connect(AntennaConstants.ACTIVE_MQ_USER, AntennaConstants.ACTIVE_MQ_PASS, wait=True)
        self.connection.subscribe(destination=AntennaConstants.ACTIVE_MQ_TOPIC_IN, id=1, ack='auto')

    def start(self):
        print('Starting ActiveMQListener')
        self.topic_public_calc = rospy.Publisher(TopicConstants.TOPIC_PUBLIC_CALC, Flyover_request,
                                                 queue_size=TopicConstants.QUEUE_SIZE)
        self.topic_public_utf_antenna = rospy.Publisher(TopicConstants.TOPIC_PUBLIC_UTF_ANTENNA, UTF_antenna_command,
                                                        queue_size=TopicConstants.QUEUE_SIZE)
        self.topic_communication = rospy.Publisher(TopicConstants.TOPIC_COMMUNICATION, Communication_package,
                                                   queue_size=TopicConstants.QUEUE_SIZE)
        self.topic_satellite_manager = rospy.Publisher(TopicConstants.TOPIC_SATELLITE_MANAGER,
                                                       Satellite_manager_command, queue_size=TopicConstants.QUEUE_SIZE)
        self.topic_active_mq_writer = rospy.Publisher(TopicConstants.TOPIC_ACTIVE_MQ_WRITER, Response,
                                                      queue_size=TopicConstants.QUEUE_SIZE)
        rospy.init_node(TopicConstants.TOPIC_ACTIVE_MQ_LISTENER, anonymous=False)
        self.connect_to_mq()


if __name__ == '__main__':
    ActiveMQListener().start()
    while True:
        time.sleep(1)
