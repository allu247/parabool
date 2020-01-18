#!/usr/bin/env python
__author__ = "Mart Hytt"

import rospy
import datetime
from antenna_msgs.msg import Satellite_manager_command
from antenna_msgs.msg import Response
from utils.tle_utils import get_tle_file_path, read_catalog_numbers_from_config
from constants.action_constants import ActionConstants
from constants.topic_constants import TopicConstants


class SatelliteManager:

    def __init__(self):
        self.topic_active_mq_writer = None
        rospy.init_node(TopicConstants.TOPIC_SATELLITE_MANAGER, anonymous=False)

    def add_ids(self, ids):
        rospy.loginfo(str(datetime.datetime.now()) + ': SatelliteManager adding IDs: ' + str(ids))
        with open(get_tle_file_path("all_cubesats.txt"), "a") as myfile:
            for sat_id in ids:
                myfile.write(sat_id)
                myfile.write('\n')

    def remove(self, ids):
        old_ids = read_catalog_numbers_from_config()
        rospy.loginfo(str(datetime.datetime.now()) + ': SatelliteManager remove IDs: ' + str(ids))
        with open(get_tle_file_path("all_cubesats.txt"), "w") as myfile:
            for old in old_ids:
                if str(old) not in ids:
                    myfile.write(str(old))
                    myfile.write('\n')

    def remove_all(self):
        rospy.loginfo(str(datetime.datetime.now()) + ': SatelliteManager remove all IDs')
        with open(get_tle_file_path("all_cubesats.txt"), "w") as myfile:
            myfile.write('')

    def respond(self, request_id, ok, data):
        response = Response()
        response.request_id = request_id
        response.ok = ok
        response.data = data

        rospy.loginfo(str(datetime.datetime.now()) + ': SatelliteManager sending  response: ' + str(response))
        self.topic_active_mq_writer.publish(response)

    def callback(self, data):
        try:
            if data.action == ActionConstants.ACTION_ADD_ID:
                self.add_ids(data.satellite_ids)

            if data.action == ActionConstants.ACTION_REMOVE_ID:
                self.remove(data.satellite_ids)

            if data.action == ActionConstants.ACTION_REMOVE_ALL_IDS:
                self.remove_all()

            self.respond(data.request_id, True, read_catalog_numbers_from_config())
        except Exception as ex:
            rospy.loginfo(
                str(datetime.datetime.now()) + ': SatelliteManager request ID: ' + data.request_id + ' ' + str(ex))
            self.respond(data.request_id, False, [str(ex)])

    def subscribe(self):
        rospy.Subscriber(TopicConstants.TOPIC_SATELLITE_MANAGER, Satellite_manager_command, self.callback)
        rospy.spin()

    def start(self):
        print("Starting SatelliteManager")
        self.topic_active_mq_writer = rospy.Publisher(TopicConstants.TOPIC_ACTIVE_MQ_WRITER, Response,
                                                      queue_size=TopicConstants.QUEUE_SIZE)
        self.subscribe()


if __name__ == '__main__':
    SatelliteManager().start()
