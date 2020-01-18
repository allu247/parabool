#!/usr/bin/env python
__author__ = 'Rasmus Tomsen'

import rospy
from std_msgs.msg import Int64
from antenna_controller_utils import read_satellite_id_from_config


# publish_command(int(earliest_satellite_id))
def publish_command(catalog_number):
    """
    Publishes the correct catalog number to satellite chooser topic
    """
    publish_new_catalog_number = rospy.Publisher("/satellite_chooser", Int64, queue_size=10)
    rospy.init_node('pub_catalog_num', anonymous=False)
    publish_new_catalog_number.publish(catalog_number)


if __name__ == '__main__':
    read_satellite_id_from_config()
