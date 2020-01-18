#!/usr/bin/env python
__author__ = 'Mart Hytt'

from spacetrack import SpaceTrackClient
import os
import ephem


def get_tle_file_path(file_name):
    return str(os.path.dirname(__file__) + '/../../tle/' + file_name)


def format_tle(tle):
    tle_string = tle[2:]
    with open(get_tle_file_path('tle.txt'), 'a') as f:
        f.write(tle_string)


def read_catalog_numbers_from_config():
    """
    Read satellite id from file.
    """
    catalog_numbers = []

    for line in open(get_tle_file_path('all_cubesats.txt'), 'r').readlines():
        catalog_numbers.append(int(line))

    return catalog_numbers


def get_tles_for_catalog_numbers(catalog_numbers):
    """
    Gets TLE for every cube satellite.
    """
    space_tracker = SpaceTrackClient('rasmustomsen@hotmail.com', '!kK!Ft3-W6sKGa8X')
    data = space_tracker.tle_latest(iter_lines=True, norad_cat_id=catalog_numbers, ordinal=1, format='3le')
    tles = []
    current_tle = []

    for line in data:
        current_tle.append(line.encode("utf-8"))
        if len(current_tle) >= 3:
            tles.append(current_tle)
            current_tle = []

    return tles


def find_next_pass(tle, long, lat, elevation):
    """
    Finds next satellite that enters the field of view of the antenna
    """
    obs = ephem.Observer()
    obs.lat = long
    obs.long = lat
    obs.elevation = elevation

    passes = []

    for entry in tle:
        satellite = ephem.readtle(entry[0], entry[1], entry[2])
        satellite.compute(obs)

        try:
            obs.next_pass(satellite)
        except ValueError:
            print("ValueError")
        else:
            passes.append(str(obs.next_pass(satellite)).replace('(', '').replace(')', ''))

    return passes

def get_current_satellite():
    """
    Gets selected cubesate id from file
    :return: current satellite id or rises exception
    """
    current = []
    for line in open(get_tle_file_path('current_satellite.txt'), 'r').readline():
        current.append(line)

    if len(current) is 1:
        return current[0]

    raise Exception('Current cubesat id error')
