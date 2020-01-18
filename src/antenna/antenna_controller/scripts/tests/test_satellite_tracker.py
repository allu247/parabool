#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Rasmus Tomsen'

import math
import unittest
import satellite_tracker_node


class TestConvertToCorrectRadians(unittest.TestCase):
    def test_one(self):
        self.assertEqual(1, 1, "1!=1")

    def test_radians_less_than_zero_more_than_minus_two_pi(self):
        initial = -math.pi
        ans = satellite_tracker_node.convert_to_correct_radians(initial)
        self.assertEqual(ans, math.pi)

    def test_radians_less_than_minus_two_pi(self):
        initial = -3 * math.pi
        ans = satellite_tracker_node.convert_to_correct_radians(initial)
        self.assertEqual(ans, math.pi)

    def test_radians_within_limits(self):
        initial = math.pi
        ans = satellite_tracker_node.convert_to_correct_radians(initial)
        self.assertEqual(ans, math.pi)

    def test_radians_more_than_two_pi(self):
        initial = 5 * math.pi
        ans = satellite_tracker_node.convert_to_correct_radians(initial)
        self.assertEqual(ans, math.pi)

    def test_radians_full_circle(self):
        initial = 4 * math.pi
        ans = satellite_tracker_node.convert_to_correct_radians(initial)
        self.assertEqual(ans, 0)


class TestCalculateVelocity(unittest.TestCase):

    def test_velocity_all_over_zero(self):
        initial_vel = 5
        acceleration = 2
        displacement = 50
        final_vel = satellite_tracker_node.calculate_velocity(initial_vel, displacement, acceleration)
        self.assertEqual(final_vel, 15)

    def test_velocity_initial_velocity_zero(self):
        initial_vel = 0
        acceleration = 2
        displacement = 16
        final_vel = satellite_tracker_node.calculate_velocity(initial_vel, displacement, acceleration)
        self.assertEqual(final_vel, 8)

    def test_velocity_acceleration_zero(self):
        initial_vel = 5
        acceleration = 0
        displacement = 50
        final_vel = satellite_tracker_node.calculate_velocity(initial_vel, displacement, acceleration)
        self.assertEqual(final_vel, initial_vel)

    def test_velocity_displacement_zero(self):
        initial_vel = 5
        acceleration = 2
        displacement = 0
        final_vel = satellite_tracker_node.calculate_velocity(initial_vel, displacement, acceleration)
        self.assertEqual(final_vel, initial_vel)

    def test_velocity_negative_acceleration(self):
        initial_vel = 15
        acceleration = -2
        displacement = 50
        final_vel = satellite_tracker_node.calculate_velocity(initial_vel, displacement, acceleration)
        self.assertEqual(final_vel, 5)


class TestCalculateAcceleration(unittest.TestCase):

    def test_acceleration_all_over_zero(self):
        displacement = 50
        initial_velocity = 5
        time = 5
        acceleration = satellite_tracker_node.calculate_acceleration(displacement, initial_velocity, time)
        self.assertEqual(acceleration, 2)

    def test_acceleration_displacement_zero(self):
        displacement = 0
        initial_velocity = 5
        time = 10
        acceleration = satellite_tracker_node.calculate_acceleration(displacement, initial_velocity, time)
        self.assertEqual(acceleration, -1)

    def test_acceleration_initial_velocity_zero(self):
        displacement = 50
        initial_velocity = 0
        time = 5
        acceleration = satellite_tracker_node.calculate_acceleration(displacement, initial_velocity, time)
        self.assertEqual(acceleration, 4)

    def test_acceleration_negative_displacement(self):
        displacement = -10
        initial_velocity = 5
        time = 10
        acceleration = satellite_tracker_node.calculate_acceleration(displacement, initial_velocity, time)
        self.assertEqual(acceleration, -1.2)


class TestFindMinTimeBetweenTwoPoints(unittest.TestCase):

    def test_min_time_velocity_not_changing(self):
        initial_velocity = 5
        final_velocity = 5
        displacement = 25
        time = satellite_tracker_node.find_min_time_between_two_points(initial_velocity, final_velocity, displacement)
        self.assertEqual(time, 5)

    def test_min_time_positive_displacement(self):
        initial_velocity = 5
        final_velocity = 20
        displacement = 50
        time = satellite_tracker_node.find_min_time_between_two_points(initial_velocity, final_velocity, displacement)
        self.assertEqual(time, 4)

    def test_min_time_negative_displacement(self):
        initial_velocity = 5
        final_velocity = 25
        displacement = -50
        time = satellite_tracker_node.find_min_time_between_two_points(initial_velocity, final_velocity, displacement)
        self.assertEqual(time, 5)


class TestGetDisplcementAz(unittest.TestCase):

    def test_displacement_az_pass_zero_point(self):
        az1 = math.pi * 0.25
        az2 = math.pi * 1.75
        displacement = satellite_tracker_node.get_displacement_az(az1, az2)
        self.assertEqual(displacement, math.pi * 0.5)

    def test_displacement_az_does_not_pass_zero_point(self):
        az1 = math.pi * 0.25
        az2 = math.pi * 0.75
        displacement = satellite_tracker_node.get_displacement_az(az1, az2)
        self.assertEqual(displacement, math.pi * 0.5)


if __name__ == '__main__':
    unittest.main()
