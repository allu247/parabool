#!/usr/bin/env python
__author__ = 'Rasmus Tomsen'

import rospy
from utils.tle_utils import get_tle_file_path, format_tle
from spacetrack import SpaceTrackClient
import time
import schedule


class TLEGetter:
    def __init__(self):
        pass

    def tle_schedule(self):
        """
        Asks for a new TLE every four hours.
        """
        time.sleep(1)
        self.get_tle()

        schedule.every(4).hours.do(self.get_tle)
        while True:
            schedule.run_pending()
            time.sleep(60)

    def get_tle(self):
        """
        Replaces the current satellite TLE when needed.
        """

        for line in open(get_tle_file_path('current_satellite.txt'), 'r').readlines():
            catalog_number = int(line)
            st = SpaceTrackClient('rasmustomsen@hotmail.com', '!kK!Ft3-W6sKGa8X')
            tle = st.tle_latest(norad_cat_id=catalog_number, ordinal=1, format='3le').encode("utf-8").strip()
            rospy.loginfo(tle)
            format_tle(tle)


if __name__ == '__main__':
    TLEGetter().tle_schedule()
