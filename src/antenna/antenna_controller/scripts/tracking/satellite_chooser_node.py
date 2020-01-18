#!/usr/bin/env python
__author__ = 'Rasmus Tomsen'

import rospy
from spacetrack import SpaceTrackClient
from std_msgs.msg import Int64
import tle_getter_node
import ephem


class SatelliteChooser:
    """docstring for SatelliteChooser"""

    def __init__(self):
        self.GROUND_STATION_LATITUDE = '59.394870'
        self.GROUND_STATION_LONGITUDE = '24.661399'
        self.MIN_SATELLITE_ELEVATION = '0'

    def does_satellite_pass(self, tle, catalog_number):
        """
        Checks whether the asked satellite ever enters the field of view of the antenna
        :param tle: Satellite Two-Line Element
        :param catalog_number: Satellite NORAD catalog number
        """
        tle_lines = tle[2:].split('\n')
        obs = ephem.Observer()
        obs.lat = self.GROUND_STATION_LATITUDE
        obs.long = self.GROUND_STATION_LONGITUDE
        obs.horizon = self.MIN_SATELLITE_ELEVATION
        satellite = ephem.readtle(tle_lines[0], tle_lines[1], tle_lines[2])
        try:
            obs.next_pass(satellite)
        except ValueError:
            rospy.loginfo("Satellite with catalog number {} is never visible".format(catalog_number))
            return False
        else:
            return True

    def add_satellite_to_track(self, catalog_number):
        """
        Replaces the current satellite
        :param catalog_number: Satellite NORAD catalog number
        """
        current_satellite = 0
        for line in open('/home/rasmus/catkin_ws/src/antenna/antenna_controller/tle/current_satellite.txt',
                         'r').readlines():
            current_satellite = line
        if not int(current_satellite) == catalog_number:
            with open('/home/rasmus/catkin_ws/src/antenna/antenna_controller/tle/current_satellite.txt', 'w') as f:
                f.write("{}".format(catalog_number))

    def callback(self, data):
        catalog_number = data.data
        st = SpaceTrackClient('rasmustomsen@hotmail.com', '!kK!Ft3-W6sKGa8X')
        tle = st.tle_latest(norad_cat_id=catalog_number, ordinal=1, format='3le').encode("utf-8").strip()
        if not (tle == ""):
            if self.does_satellite_pass(tle, catalog_number):
                self.add_satellite_to_track(catalog_number)
                tle_getter_node.format_tle(tle)
        else:
            rospy.loginfo("{}: Wrong Catalog Number".format(catalog_number))

    def choose_satellite(self):
        rospy.init_node('satellite_chooser')
        rospy.Subscriber('satellite_chooser', Int64, self.callback)
        rospy.spin()


if __name__ == '__main__':
    SatelliteChooser().choose_satellite()
