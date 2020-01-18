#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Rasmus Tomsen'

import rospy
from antenna_msgs.msg import Antenna_command
from antenna_msgs.msg import Antenna_position
from datetime import datetime
from constants import AntennaConstants
from src.antenna.antenna_controller.scripts.utils import tle_utils
import math
import ephem
import time
import schedule
import os


# control_msgs/JointControllerState
def write_pass_parameters_to_file(time, sat_az, sat_elev, antenna_az, antenna_elev, turning_time, displacement_az,
                                  displacement_elev, initial_leg_velocity, initial_dish_velocity, final_leg_velocity,
                                  final_dish_velocity, acceleration_leg, acceleration_dish):
    """
    Writes different necessary parameters for testing to a csv file
    """
    with open(os.path.dirname(__file__) + '/../output/antenna_movement_data.csv', 'a') as file:
        file.write("{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(time,
                                                                        sat_az,
                                                                        sat_elev,
                                                                        antenna_az,
                                                                        antenna_elev,
                                                                        turning_time,
                                                                        displacement_az,
                                                                        displacement_elev,
                                                                        initial_leg_velocity,
                                                                        initial_dish_velocity,
                                                                        final_leg_velocity,
                                                                        final_dish_velocity,
                                                                        acceleration_leg,
                                                                        acceleration_dish))


class SatelliteTracker:
    def __init__(self):
        # Finals:
        self.SECONDS_PER_MOVE = AntennaConstants.SECONDS_PER_MOVE
        self.MAX_LEG_JOINT_ANGULAR_VELOCITY = AntennaConstants.MAX_LEG_JOINT_ANGULAR_VELOCITY
        self.MAX_DISH_JOINT_ANGULAR_VELOCITY = AntennaConstants.MAX_DISH_JOINT_ANGULAR_VELOCITY
        self.GROUND_STATION_LATITUDE = AntennaConstants.GROUND_STATION_LATITUDE
        self.GROUND_STATION_LONGITUDE = AntennaConstants.GROUND_STATION_LONGITUDE
        self.GROUND_STATION_ELEVATION = AntennaConstants.GROUND_STATION_ELEVATION
        self.MIN_SATELLITE_ELEVATION = AntennaConstants.MIN_SATELLITE_ELEVATION

        # Variables:
        self.next_pass = []
        self.antenna_command_publisher = ""
        self.last_move_direction = 0
        self.is_stopped = False
        self.is_zenith = False
        self.max_elev_time = ''
        self.max_elev = 0
        self.last_leg_joint_velocity = 0
        self.last_dish_joint_velocity = 0

    def run_program(self):
        """
        Starts the program.
        """
        open(tle_utils.get_tle_file_path('tle.txt'), 'w').close()
        self.antenna_command_publisher = rospy.Publisher("/antenna/antenna_command", Antenna_command, queue_size=10)
        rospy.init_node('satellite_tracker', anonymous=True)
        with open(os.path.dirname(__file__) + '/../output/antenna_movement_data.csv', 'w') as f:
            f.write('Satellite time,Satellite azimuth,Satellite elevation,Antenna azimuth,Antenna elevation,'
                    'Turning time,Displacement azimuth,Displacement elevation,Initial velocity azimuth,'
                    'Initial velocity elevation,Final velocity azimuth,Final velocity elevation,Acceleration azimuth,'
                    'Acceleration elevation\n')

        while not (os.stat(tle_utils.get_tle_file_path('tle.txt')).st_size > 0):
            print("No tle yet")
            time.sleep(0.5)

        self.get_next_passes()

        self.move_antenna_once(self.next_pass[0])

        schedule.every(0.5).seconds.do(self.check_time)
        while True:
            schedule.run_pending()
            time.sleep(0.5)

    def check_time(self):
        """
        Compares the current time to the next satellite pass time.
        If satellite comes into the field of view, program starts tracking it.
        """
        self.get_weather_info()
        if self.is_stopped:
            return
        if len(self.next_pass) > 0:
            self.check_for_new_satellite()

            satellite_time = ephem.localtime(self.next_pass[0][0])
            current_time = datetime.now()
            satellite_time_string = satellite_time.strftime("%d-%m-%Y %H:%M:%S")
            current_time_string = current_time.strftime("%d-%m-%Y %H:%M:%S")

            print("Time until LOS: {}s".format(len(self.next_pass) * self.SECONDS_PER_MOVE))
            print("{}\n{} {}".format(current_time_string, satellite_time_string, self.next_pass[0][3]))

            if current_time_string == satellite_time_string:
                pass_second = self.next_pass.pop(0)
                self.set_last_move_direction(pass_second[1])
                move_leg = self.get_leg_move(pass_second)
                move_dish = self.get_dish_move(pass_second)
                self.make_movement(move_leg, move_dish, self.SECONDS_PER_MOVE, pass_second[0])
            elif current_time > satellite_time:
                self.last_leg_joint_velocity = 0
                self.last_dish_joint_velocity = 0
                self.get_next_passes()
                self.move_antenna_once(self.next_pass[0])
        else:
            self.last_leg_joint_velocity = 0
            self.last_dish_joint_velocity = 0
            time.sleep(10)
            self.get_next_passes()
            self.move_antenna_once(self.next_pass[0])

    def get_weather_info(self):
        """
        If wind is too strong then stops the satellite tracking and parks the antenna.
        """
        current_weather_data = rospy.get_param('weather_information')
        if current_weather_data["should_stop_antenna"]:
            if not self.is_stopped:
                move = ["", current_weather_data["antenna_el"], current_weather_data["antenna_az"]]
                self.move_antenna_once(move)
                self.change_antenna_state()
        else:
            if self.is_stopped:
                self.change_antenna_state()
        print("Current wind speed: {}".format(current_weather_data['wind_speed']))

    def check_for_new_satellite(self):
        """
        Checks which satellite the program has to track at the moment.
        """
        for line in open('/home/rasmus/catkin_ws/src/antenna/antenna_controller/tle/current_satellite.txt',
                         'r').readlines():
            if not (int(line) == self.next_pass[0][4]):
                print("SATELLITE HAS CHANGED: {} -> {}".format(self.next_pass[0][4], line))
                self.get_next_passes()
                self.move_antenna_once(self.next_pass[0])
            break

    def set_last_move_direction(self, pass_az):
        """
        Checks which way the satellite is moving
        :param pass_az: Satellite azimuth
        """
        if len(self.next_pass) > 0:
            if pass_az > self.next_pass[0][1]:
                self.last_move_direction = -1
            else:
                self.last_move_direction = 1

    def get_leg_move(self, move):
        """
        If satellite is zenith then isn't going to move the leg joint.
        :param move: Array of next satellite pass part, [time, azimuth, elevation]
        """
        leg_move = move[2]
        if math.degrees(self.max_elev) > 89:
            leg_move = self.get_current_position()[0]
        return leg_move

    def get_dish_move(self, move):
        """
        If satellite is zenith then moves the dish full 180 degrees.
        :param move: Array of next satellite pass part, [time, azimuth, elevation]
        """
        dish_move = move[1]
        if math.degrees(self.max_elev) > 89:
            if self.max_elev_time < move[0]:
                dish_move = math.pi - move[2]
        return dish_move

    def move_antenna_once(self, move):
        """
        Used for steering the antenna when the antenna doesn't need to move too accurately.
        Mostly used when the antenna needs to move to the starting position of the next satellite flyover.
        :param move: Array of next satellite pass part, [time, azimuth, elevation]
        """
        current_position_data = self.get_current_position()
        start_pos_az = current_position_data[0]
        start_pos_elev = current_position_data[1]
        new_current_pos_az = start_pos_az
        new_current_pos_elev = start_pos_elev

        from src.antenna.antenna_controller.scripts.utils.math_utils import convert_to_correct_radians
        end_pos_az = convert_to_correct_radians(move[2])
        end_pos_elev = move[1]

        for _ in xrange(10):
            new_current_pos_az = self.get_next_move_az_pos(start_pos_az, new_current_pos_az, end_pos_az)
            new_current_pos_elev = self.get_next_move_elev_pos(start_pos_elev, new_current_pos_elev, end_pos_elev)
            from src.antenna.antenna_controller.scripts.utils.math_utils import convert_to_correct_radians
            new_current_pos_az = convert_to_correct_radians(new_current_pos_az)
            turning_time = self.get_antenna_turning_time(new_current_pos_az, new_current_pos_elev)
            self.make_movement(new_current_pos_az, new_current_pos_elev, turning_time, move[0])
            time.sleep(turning_time)
        self.stop_antenna()
        self.adjust_antenna(end_pos_az, end_pos_elev)
        self.last_leg_joint_velocity = 0
        self.last_dish_joint_velocity = 0

    def get_next_move_az_pos(self, start_pos_az, new_current_pos_az, end_pos_az):
        """
        Checks which way the antenna has to move and returns the next move step azimuth
        :param start_pos_az: Starting position azimuth of the antenna
        :param new_current_pos_az: Half way position azimuth of the antenna
        :param end_pos_az: Ending position azimuth of the antenna
        """
        displacement = math.fabs(start_pos_az - end_pos_az)
        az_step_alpha = displacement / 10.0
        az_step_beta = (2 * math.pi - displacement) / 10.0
        if start_pos_az < end_pos_az:
            if self.last_move_direction >= 0:
                new_current_pos_az -= az_step_beta
            else:
                new_current_pos_az += az_step_alpha
        else:
            if self.last_move_direction >= 0:
                new_current_pos_az -= az_step_alpha
            else:
                new_current_pos_az += az_step_beta
        return new_current_pos_az

    def get_next_move_elev_pos(self, start_pos_elev, new_current_pos_elev, end_pos_elev):
        """
        Checks which way the antenna has to move and returns the next step elevation
        :param start_pos_elev: Starting position elevation of the antenna
        :param new_current_pos_elev: Half way position elevation of the antenna
        :param end_pos_elev: Ending position elevation of the antenna
        """
        elev_step = math.fabs(start_pos_elev - end_pos_elev) / 10.0
        if start_pos_elev < end_pos_elev:
            new_current_pos_elev += elev_step
        else:
            new_current_pos_elev -= elev_step
        return new_current_pos_elev

    def adjust_antenna(self, sat_az, sat_elev):
        """
        Adjusts the antenna position to get the necessary precision.
        :param sat_az: Satellite azimuth
        :param sat_elev: Satellite elevation
        """
        current_pos_az = self.get_current_position()[0]
        while math.fabs(current_pos_az - sat_az) > math.radians(0.5):
            self.make_movement(sat_az, sat_elev, 1, 0)
            print("ADJUSTING ANTENNA: sat: {}, cur: {}".format(sat_az, current_pos_az))
            current_pos_az = self.get_current_position()[0]

    def make_movement(self, az, elev, seconds, sat_time):
        """
        Calculates the necessary parameters to move the antenna.
        :param az: Azimuth to turn the antenna to
        :param elev: Elevation to turn the antenna to
        :param seconds: How long does the turning take
        """
        current_position_data = self.get_current_position()
        current_pos_az = current_position_data[0]
        current_pos_elev = current_position_data[1]

        from src.antenna.antenna_controller.scripts.utils.math_utils import get_displacement_az
        displacement_az = get_displacement_az(az, current_pos_az)
        displacement_elev = math.fabs(elev - current_pos_elev)

        from src.antenna.antenna_controller.scripts.utils.math_utils import calculate_acceleration
        leg_acceleration = calculate_acceleration(displacement_az, self.last_leg_joint_velocity, seconds, 1)
        from src.antenna.antenna_controller.scripts.utils.math_utils import calculate_acceleration
        dish_acceleration = calculate_acceleration(displacement_elev, self.last_dish_joint_velocity, seconds, 1)
        from src.antenna.antenna_controller.scripts.utils.math_utils import calculate_velocity
        final_leg_joint_velocity = calculate_velocity(self.last_leg_joint_velocity, displacement_az,
                                                      leg_acceleration, 1)
        from src.antenna.antenna_controller.scripts.utils.math_utils import calculate_velocity
        final_dish_joint_velocity = calculate_velocity(self.last_dish_joint_velocity, displacement_elev,
                                                       dish_acceleration, 1)

        write_pass_parameters_to_file(sat_time, az, elev, current_pos_az, current_pos_elev, seconds, displacement_az,
                                      displacement_elev, self.last_leg_joint_velocity, self.last_dish_joint_velocity,
                                      final_leg_joint_velocity, final_dish_joint_velocity, leg_acceleration,
                                      dish_acceleration)

        self.last_leg_joint_velocity = final_leg_joint_velocity
        self.last_dish_joint_velocity = final_dish_joint_velocity

        self.publish_command(self.last_leg_joint_velocity, self.last_dish_joint_velocity, az, elev, leg_acceleration,
                             dish_acceleration)

    def publish_command(self, leg_linear_velocity, dish_linear_velocity, leg_angle, dish_angle, leg_acceleration=0,
                        dish_acceleration=0):
        """
        Puts together a message for the antenna_controller and publishes it.
        """
        msg = Antenna_command()
        msg.leg_linear_velocity = leg_linear_velocity
        msg.dish_linear_velocity = dish_linear_velocity
        msg.leg_angle = leg_angle
        msg.dish_angle = dish_angle
        msg.leg_linear_acceleration = leg_acceleration
        msg.dish_linear_acceleration = dish_acceleration
        self.antenna_command_publisher.publish(msg)

    def get_antenna_turning_time(self, sat_az, sat_elev):
        """
        Calculates the min turning time of the satellite.
        :param sat_az: Satellite azimuth
        :param sat_elev: Satellite elevation
        """
        current_position_data = self.get_current_position()
        current_pos_az = current_position_data[0]
        current_pos_elev = current_position_data[1]

        from src.antenna.antenna_controller.scripts.utils.math_utils import get_displacement_az
        displacement = get_displacement_az(current_pos_az, sat_az)

        from src.antenna.antenna_controller.scripts.utils.math_utils import find_min_time_between_two_points
        dish_joint_time = find_min_time_between_two_points(self.last_dish_joint_velocity,
                                                           self.MAX_DISH_JOINT_ANGULAR_VELOCITY,
                                                           math.fabs(sat_elev - current_pos_elev))
        from src.antenna.antenna_controller.scripts.utils.math_utils import find_min_time_between_two_points
        leg_joint_time = find_min_time_between_two_points(self.last_leg_joint_velocity,
                                                          self.MAX_LEG_JOINT_ANGULAR_VELOCITY,
                                                          displacement)
        print("===============================================================")
        print("\nMOVE ANTENNA ONCE: \n\tCurrent: {}, {}\n\tNew:{}, {}\n\tSeconds: {}".format(current_pos_az,
                                                                                             current_pos_elev,
                                                                                             sat_az,
                                                                                             sat_elev,
                                                                                             leg_joint_time))

        if dish_joint_time > leg_joint_time:
            return dish_joint_time
        else:
            return leg_joint_time

    def change_antenna_state(self):
        """
        Stops the tracking if the antenna is working and continues to track if the antenna has been stopped.
        """
        if self.is_stopped:
            self.is_stopped = False
            self.stop_antenna()
            rospy.loginfo("Shut down antenna")
        else:
            self.is_stopped = True
            rospy.loginfo("Continue tracking")

    def stop_antenna(self):
        """
        Brakes the antenna.
        """
        stopping_time = 1
        initial_leg_velocity = self.last_leg_joint_velocity
        initial_dish_velocity = self.last_dish_joint_velocity
        current_position_data = self.get_current_position()
        current_pos_az = current_position_data[0]
        current_pos_elev = current_position_data[1]

        leg_acceleration = initial_dish_velocity * -1 / stopping_time
        dish_acceleration = initial_dish_velocity * -1 / stopping_time
        from src.antenna.antenna_controller.scripts.utils.math_utils import convert_to_correct_radians
        leg_angle = convert_to_correct_radians(current_pos_az + (initial_leg_velocity * stopping_time / 2))
        dish_angle = current_pos_elev + (initial_dish_velocity * stopping_time / 2)

        self.publish_command(0, 0, leg_angle, dish_angle, leg_acceleration, dish_acceleration)

    def get_current_position(self):
        """
        Returns the antennas current position.
        """
        current_position_data = rospy.wait_for_message('/antenna/antenna_position', Antenna_position)
        from src.antenna.antenna_controller.scripts.utils.math_utils import convert_to_correct_radians
        current_pos_az = convert_to_correct_radians(current_position_data.leg_angle)
        current_pos_elev = current_position_data.dish_angle
        return (current_pos_az, current_pos_elev)

    def get_next_passes(self):
        """
        Finds information about the next satellite flyover.
        """
        final_max_elev_time = ""
        final_max_elev = 0
        tle_lines = []
        for line in open('/home/rasmus/catkin_ws/src/antenna/antenna_controller/tle/tle.txt', 'r').readlines():
            tle_lines.append(line)
        print("Next satellite TLE: \n{}".format(tle_lines))

        pass_parts = []
        obs = ephem.Observer()
        obs.lat = self.GROUND_STATION_LATITUDE
        obs.long = self.GROUND_STATION_LONGITUDE
        obs.elevation = self.GROUND_STATION_ELEVATION

        satellite = ephem.readtle(tle_lines[0], tle_lines[1], tle_lines[2])
        satellite.compute(obs)

        while len(pass_parts) == 0:
            pass_parts = []
            rise_time, rise_az, max_elev_time, max_elev, set_time, set_az = obs.next_pass(satellite)
            final_max_elev_time = max_elev_time
            final_max_elev = max_elev
            if math.degrees(max_elev) < self.MIN_SATELLITE_ELEVATION:
                obs.date = set_time + ephem.minute
                continue

            if rise_time > set_time:
                turning_time = self.get_antenna_turning_time(math.radians(math.degrees(satellite.az)),
                                                             math.radians(math.degrees(satellite.alt)))
                rise_time = ephem.Date(ephem.Date(datetime.utcnow()) + ephem.second * (turning_time + 30))

                if rise_time > set_time:
                    obs.date = set_time + ephem.minute
                    continue

            while rise_time < set_time:
                obs.date = rise_time
                satellite.compute(obs)
                pass_parts.append((rise_time, math.radians(math.degrees(satellite.alt)),
                                   math.radians(math.degrees(satellite.az)), tle_lines[0],
                                   int(tle_lines[2].split(" ")[1])))
                rise_time = ephem.Date(rise_time + ephem.second * self.SECONDS_PER_MOVE)

        self.max_elev_time = final_max_elev_time
        self.max_elev = final_max_elev
        self.next_pass = pass_parts


def main():
    controller = SatelliteTracker()
    controller.run_program()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
