#!/usr/bin/env python
__author__ = "Mart Hytt"

from constants.topic_constants import TopicConstants
from constants.antenna_constants import AntennaConstants
from antenna_msgs.msg import Response
from stomp import Connection
import rospy
import json
import datetime


class ActiveMQWriter:
    def write_to_active_mq(self, message):
        rospy.loginfo(str(datetime.datetime.now()) + ': Posting to ActiveMQ ' + str(message))
        conn = Connection(host_and_ports=[(AntennaConstants.ACTIVE_MQ_HOST, AntennaConstants.ACTIVE_MQ_PORT)])
        conn.start()
        conn.connect(AntennaConstants.ACTIVE_MQ_USER, AntennaConstants.ACTIVE_MQ_PASS, wait=True)
        conn.send(body=json.dumps(message), destination=AntennaConstants.ACTIVE_MQ_TOPIC_OUT)
        conn.disconnect()

    def callback(self, data):
        self.write_to_active_mq({'request_id': data.request_id, 'ok': data.ok, 'data': data.data})

    def subscribe(self):
        rospy.Subscriber(TopicConstants.TOPIC_ACTIVE_MQ_WRITER, Response, self.callback)
        rospy.spin()

    def start(self):
        print("Starting ActiveMQWriter")
        rospy.init_node(TopicConstants.TOPIC_ACTIVE_MQ_LISTENER, anonymous=False)
        self.subscribe()


if __name__ == '__main__':
    ActiveMQWriter().start()
