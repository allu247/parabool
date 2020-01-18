#!/usr/bin/env python
__author__ = 'Mart Hytt'

from antenna_msgs.msg import UTF_antenna_command
from antenna_msgs.msg import Response
from utils.utf_antenna import UTFAntenna
from constants.antenna_constants import AntennaConstants
from constants.action_constants import ActionConstants
from constants.topic_constants import TopicConstants
from utils.ros_exception import RosNodeException
import rospy
import datetime


class UTFAntennaMover:

    def __init__(self):
        self.topic_active_mq_writer = None
        self.utf_antenna = UTFAntenna(AntennaConstants.UTF_ANTENNA_PORT)

    def __delete__(self):
        self.utf_antenna = None

    def send_response(self, request_id, ok, data):
        response = Response()
        response.request_id = request_id
        response.ok = ok
        response.data = data
        self.topic_active_mq_writer.publish(response)

    def move(self, azimuth, elevation):
        rospy.loginfo(str(datetime.datetime.now()) + ': RadioAntennaController calculated azimute: ' + azimuth)
        rospy.loginfo(str(datetime.datetime.now()) + ': RadioAntennaController calculated elevation: ' + elevation)

        if azimuth < 0 or azimuth > 360:
            rospy.logwarn(str(datetime.datetime.now()) + ': Azimuth outside of range: ' + str(azimuth))
            raise RosNodeException('Azimuth outside of range: ' + str(azimuth), 0)

        if elevation < 0 or elevation > 90:
            rospy.logwarn(str(datetime.datetime.now()) + ': Elevation outside of range: ' + str(elevation))
            raise RosNodeException('Elevation outside of range: ' + str(azimuth), 0)

        self.utf_antenna.rotate(azimuth, elevation)
        rospy.loginfo(str(datetime.datetime.now()) + ': RadioAntennaController command executed: ' + elevation)

    def callback(self, msg):
        try:
            if msg.action == ActionConstants.ACTION_MOVE:
                self.move(msg.azimuth, msg.elevation)

            if msg.action == ActionConstants.ACTION_REMOTE_MOVE:
                self.move(msg.azimuth, msg.elevation)
                data = [str(self.utf_antenna.get_azimuth()), str(self.utf_antenna.get_elevation())]
                self.send_response(msg.request_id, True, data)

            if msg.action == ActionConstants.ACTION_GET_ORIENTATION:
                data = [str(self.utf_antenna.get_azimuth()), str(self.utf_antenna.get_elevation())]
                self.send_response(msg.request_id, True, data)

            self.send_response(msg.request_id, False, ['UTFAntennaMover invalid action: ' + msg.action])
        except RosNodeException as rex:
            self.send_response(msg.request_id, False, [rex.message])
        except Exception as ex:
            self.send_response(msg.request_id, False, [str(ex)])

    def subscribe(self):
        rospy.Subscriber(TopicConstants.TOPIC_PUBLIC_UTF_ANTENNA, UTF_antenna_command, self.callback)
        rospy.spin()

    def start(self):
        print('Starting UTFAntennaMover')
        self.topic_active_mq_writer = rospy.Publisher(TopicConstants.TOPIC_ACTIVE_MQ_WRITER, Response,
                                                      queue_size=TopicConstants.QUEUE_SIZE)
        rospy.init_node(TopicConstants.TOPIC_PUBLIC_UTF_ANTENNA, anonymous=False)
        self.subscribe()


if __name__ == '__main__':
    UTFAntennaMover().start()
