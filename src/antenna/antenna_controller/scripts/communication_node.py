#!/usr/bin/env python
__author__ = "Mart Hytt"

import rospy
import datetime
from constants.topic_constants import TopicConstants
from antenna_msgs.msg import Communication_package
from antenna_msgs.msg import Response


class Communication:

    def __init__(self):
        self.topic_active_mq_writer = None
        rospy.init_node(TopicConstants.TOPIC_COMMUNICATION, anonymous=False)

    def send_response(self, request_id, ok, data):
        response = Response()
        response.request_id = request_id
        response.ok = ok
        response.data = data

        rospy.loginfo(str(datetime.datetime.now()) + ': Communication sending  response: ' + str(response))
        self.topic_active_mq_writer.publish(response)

    def callback(self, data):
        try:
            rospy.loginfo(str(datetime.datetime.now()) + ': Communication node: ' + str(data))
            self.send_response(data.request_id, True, [str(data)])
        except Exception as ex:
            rospy.logerr(str(datetime.datetime.now()) + ': Request ID: ' + data.request_id + ' ' + str(ex))
            self.send_response(data.request_id, False, [str(ex)])

    def subscribe(self):
        rospy.Subscriber(TopicConstants.TOPIC_COMMUNICATION, Communication_package, self.callback)
        rospy.spin()

    def start(self):
        print("Starting Communication")
        self.topic_active_mq_writer = rospy.Publisher(TopicConstants.TOPIC_ACTIVE_MQ_WRITER, Response,
                                                      queue_size=TopicConstants.QUEUE_SIZE)
        self.subscribe()


if __name__ == '__main__':
    Communication().start()
