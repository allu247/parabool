#!/usr/bin/env python
__author__ = 'Mart Hytt'

import math


class AntennaConstants(object):
    SECONDS_PER_MOVE = 5
    GROUND_STATION_LATITUDE = 59.394870
    GROUND_STATION_LONGITUDE = 24.661399
    MAX_LEG_JOINT_ANGULAR_VELOCITY = math.radians(1.2)
    MAX_DISH_JOINT_ANGULAR_VELOCITY = math.radians(1.2)
    GROUND_STATION_ELEVATION = 1
    MIN_SATELLITE_ELEVATION = 0

    ACTIVE_MQ_HOST = 'localhost'
    ACTIVE_MQ_PORT = 61613
    ACTIVE_MQ_USER = 'admin'
    ACTIVE_MQ_PASS = 'admin'
    ACTIVE_MQ_TOPIC_IN = 'ros_in'
    ACTIVE_MQ_TOPIC_OUT = 'ros_out'
