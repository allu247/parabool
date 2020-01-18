#!/usr/bin/env python
__author__ = 'Mart Hytt'

import rospy
from antenna_msgs.msg import UTF_antenna_command
from antenna_msgs.msg import Signal_frequency
from constants.action_constants import ActionConstants
from constants.topic_constants import TopicConstants


class UTFAntennaMain:
    def __init__(self):
        self.topic_kenwood_radio = None
        self.topic_utf_antenna = None

    def publish_message(self, publisher, msg):
        publisher.publish(msg)

    def test_mode(self):
        while True:
            rotation = UTF_antenna_command()
            rotation.action = 'MOVE'
            rotation.azimuth = int(input("Please enter azimuth: "))
            rotation.elevation = int(input("Please enter elevation: "))
            self.publish_message(self.topic_utf_antenna, rotation)

    def calculate_doppler_shift(self, msg):
        frequency = Signal_frequency()
        frequency.frequency = 435.463
        self.publish_message.publish(self.topic_kenwood_radio, frequency)

    def calculate_rotation(self, msg):
        rotation = UTF_antenna_command()
        rotation.action = ActionConstants.ACTION_MOVE
        rotation.azimuth = 140
        rotation.elevation = 40
        self.publish_message.publish(self.topic_utf_antenna, rotation)

    def callback(self, msg):
        self.calculate_doppler_shift(msg)
        self.calculate_rotation(msg)

    def subscribe(self):
        rospy.Subscriber(TopicConstants.TOPIC_UTF_ANTENNA_MAIN, UTF_antenna_command, self.callback)
        rospy.spin()

    def start(self):
        print("Starting RadioAntennaMain")
        rospy.init_node(TopicConstants.TOPIC_UTF_ANTENNA_MAIN, anonymous=False)
        self.topic_utf_antenna = rospy.Publisher(TopicConstants.TOPIC_PUBLIC_UTF_ANTENNA, UTF_antenna_command,
                                                 queue_size=TopicConstants.QUEUE_SIZE)
        self.topic_kenwood_radio = rospy.Publisher(TopicConstants.TOPIC_KENWOOD_RADIO, Signal_frequency,
                                                   queue_size=TopicConstants.QUEUE_SIZE)


if __name__ == '__main__':
    UTFAntennaMain().start()
