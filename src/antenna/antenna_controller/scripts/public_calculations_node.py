#!/usr/bin/env python
__author__ = 'Mart Hytt'

import rospy
import datetime
from antenna_msgs.msg import Flyover_request
from antenna_msgs.msg import Response
from constants.topic_constants import TopicConstants
from utils.tle_utils import get_tles_for_catalog_numbers, find_next_pass, read_catalog_numbers_from_config
from constants.action_constants import ActionConstants


class PublicCalculations:

    def __init__(self):
        self.topic_active_mq_writer = None
        rospy.init_node(TopicConstants.TOPIC_PUBLIC_CALC, anonymous=False)

    def send_response(self, request_id, ok, data):
        response = Response()
        response.request_id = request_id
        response.ok = ok
        response.data = data

        rospy.loginfo(str(datetime.datetime.now()) + ': PublicCalculations send response: ' + str(response))
        self.topic_active_mq_writer.publish(response)

    def get_next_flyover(self, data):
        tle = []

        if data.satellite_ids is None or len(data.satellite_ids) == 0:
            tle = get_tles_for_catalog_numbers(read_catalog_numbers_from_config())
        else:
            tle = get_tles_for_catalog_numbers(data.satellite_ids)

        rospy.loginfo(str(datetime.datetime.now()) + ': Flyover tle: ' + str(tle))
        self.send_response(data.request_id, True, find_next_pass(tle, data.long, data.lat, data.elevation))

    def calculate_values(self, data):
        tle = []

        if data.satellite_ids is None or len(data.satellite_ids) == 0:
            tle = get_tles_for_catalog_numbers(read_catalog_numbers_from_config())
        else:
            tle = get_tles_for_catalog_numbers(data.satellite_ids)

        next_flyover = find_next_pass(tle, data.long, data.lat, data.elevation)
        rospy.loginfo(str(datetime.datetime.now()) + ': Flyover calculate values: ' + str(next_flyover))
        self.send_response(data.request_id, True, next_flyover)

    def callback(self, data):
        if data.action == ActionConstants.ACTION_NEXT_FLYOVER:
            self.get_next_flyover(data)
            return
        if data.action == ActionConstants.ACTION_CALCULATE_VALUES:
            self.calculate_values(data)
            return

        self.send_response(data.request_id, False, ['PublicCalculations invalid ACTION: ' + data.action])

    def subscribe(self):
        rospy.Subscriber(TopicConstants.TOPIC_PUBLIC_CALC, Flyover_request, self.callback)
        rospy.spin()

    def start(self):
        print('Starting PublicCalculations')
        self.topic_active_mq_writer = rospy.Publisher(TopicConstants.TOPIC_ACTIVE_MQ_WRITER, Response,
                                                      queue_size=TopicConstants.QUEUE_SIZE)
        self.subscribe()


if __name__ == '__main__':
    PublicCalculations().start()
