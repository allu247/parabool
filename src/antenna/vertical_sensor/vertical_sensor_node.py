#! /usr/bin/env python

import json
import rospy

from std_msgs.msg import String

rospy.init_node('vertical_sensors')

vertical_sensor_publisher = rospy.Publisher('/vertical_sensors', String, queue_size=10)


class SensorService:

    def listener(self):
        print('SENSORS RUNNING')
        rospy.spin()


if __name__ == '__main__':
    SensorService().listener()