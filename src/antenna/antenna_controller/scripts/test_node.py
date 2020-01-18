#!/usr/bin/env python
__author__ = "Mart Hytt"

from constants.action_constants import ActionConstants
from constants.topic_constants import TopicConstants
from antenna_msgs.msg import Flyover_request
from antenna_msgs.msg import Communication_package
from antenna_msgs.msg import UTF_antenna_command
from antenna_msgs.msg import Satellite_manager_command
from antenna_msgs.msg import Response
import rospy
import time


class Test:
    def __init__(self):
        rospy.init_node('test', anonymous=False)
        self.public_calc_topic = None
        self.public_utf_antenna_topic = None
        self.communication_out_topic = None
        self.satellite_manager_topic = None

    def callback(self, data):
        print(data)

    def send_flyover_request(self):
        request = Flyover_request()
        request.request_id = 1
        request.satellite_ids = [40024]
        request.long = 24.661399
        request.lat = 59.394870
        request.elevation = 1
        request.action = ActionConstants.ACTION_NEXT_FLYOVER
        print('sending.....')
        print(request)
        self.public_calc_topic.publish(request)

    def add_satellite_ids(self):
        request = Satellite_manager_command()
        request.request_id = 2
        request.action = ActionConstants.ACTION_REMOVE_ID
        request.satellite_ids = ["999999", "666666"]
        self.satellite_manager_topic.publish(request)

    def subscribe(self):
        rospy.Subscriber(TopicConstants.TOPIC_ACTIVE_MQ_WRITER, Response, self.callback)
        rospy.spin()

    def start(self):
        print("Starting Test")
        self.public_calc_topic = rospy.Publisher(TopicConstants.TOPIC_PUBLIC_CALC, Flyover_request, queue_size=10)
        self.public_utf_antenna_topic = rospy.Publisher(TopicConstants.TOPIC_PUBLIC_UTF_ANTENNA, UTF_antenna_command, queue_size=10)
        self.communication_out_topic = rospy.Publisher(TopicConstants.TOPIC_COMMUNICATION, Communication_package, queue_size=10)
        self.satellite_manager_topic = rospy.Publisher(TopicConstants.TOPIC_SATELLITE_MANAGER, Satellite_manager_command, queue_size=10)
        time.sleep(1)
        self.add_satellite_ids()
        self.subscribe()


if __name__ == '__main__':
    Test().start()
