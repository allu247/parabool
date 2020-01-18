#!/usr/bin/env python
__author__ = 'Mart Hytt'

import math


def convert_to_correct_radians(radians):
    """
    Converts the angle to a range of 0 to pi.
    :param radians: Initial angle
    """
    number_of_turns = math.floor(math.fabs(radians / (2 * math.pi)))
    if radians < 0:
        return radians + ((number_of_turns + 1) * 2 * math.pi)
    else:
        return radians - (number_of_turns * 2 * math.pi)


def calculate_velocity(initial_velocity, displacement, acceleration, radius=1):
    """
    Calculates the final velocity for the asked movement.
    """
    return math.sqrt(math.pow(initial_velocity, 2) + (2 * acceleration * displacement)) * radius


def calculate_acceleration(displacement, initial_velocity, seconds, radius=1):
    """
    Calculates the acceleration for the asked movement.
    """
    return (((2 * displacement) - (2 * initial_velocity * seconds)) / math.pow(seconds, 2)) * radius


def find_min_time_between_two_points(initial_velocity, final_velocity, displacement):
    """
    Calculates the min turning time between two points.
    """
    if initial_velocity == final_velocity:
        return displacement / final_velocity
    if displacement >= 0:
        return 2 * displacement / (final_velocity + initial_velocity)
    else:
        return -2 * displacement / (final_velocity - initial_velocity)


def get_displacement_az(az1, az2):
    """
    Returns the displacement of two leg joint positions.
    """
    displacement = math.fabs(az1 - az2)
    if displacement > math.pi:
        return 2 * math.pi - displacement
    return displacement
