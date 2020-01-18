#!/usr/bin/env python
__author__ = "Mart Hytt"

import rospy
import datetime
from antenna_msgs.msg import Signal_frequency
from constants.action_constants import ActionConstants
from constants.topic_constants import TopicConstants


class KenwoodRadio:

    def __init__(self):
        rospy.init_node(TopicConstants.TOPIC_KENWOOD_RADIO, anonymous=False)
        self.sending = False

    def switch_mode(self):
        self.sending = not self.sending
        rospy.loginfo(str(datetime.datetime.now()) + ': Switching listening mode to : ' + str(self.sending))

    def set_frequency(self, frequency):
        rospy.loginfo(str(datetime.datetime.now()) + ': Setting  frequency to : ' + str(frequency))

    def callback(self, msg):
        rospy.loginfo(str(datetime.datetime.now()) + ': KenwoodRadio callback: ' + str(msg))
        if msg.action == ActionConstants.ACTION_SET_FREQUENCY:
            self.set_frequency(msg.frequency)
        if msg.action == ActionConstants.ACTION_SWITCH:
            self.switch_mode()

    def subscribe(self):
        rospy.Subscriber(TopicConstants.TOPIC_KENWOOD_RADIO, Signal_frequency, self.callback)
        rospy.spin()

    def start(self):
        print("Starting KenwoodRadio")
        self.subscribe()


if __name__ == '__main__':
    KenwoodRadio().start()
