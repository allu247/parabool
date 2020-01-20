#! /usr/bin/env python

import rospy
import csv
import json
import time
import threading

from datetime import datetime
from datetime import timedelta
from std_msgs.msg import String

rospy.init_node('main_controller')

vertical_publisher = rospy.Publisher('/vertical_motor_commands', String, queue_size=10)
horizontal_publisher = rospy.Publisher('/horizontal_motor_commands', String, queue_size=10)


class ControllerService:

    def __init__(self):
        self.mission = []
        self.execution_times=[]
        self.vertical_motor_commands = []
        self.horizontal_motor_commands = []
        self.mission_started = False
        self.mission_initialized = False
        self.pre_checked = False
        self.error = False
        self.error_code = ''
        self.sender_thread = None
        self.start_time = None

    def check_commands(self):
        for command in self.vertical_motor_commands:
            destination = float(command['destination'])
            if 0 > destination > 360:
                self.error = True
                self.error_code = 'Mission not valid'
                return

        for command in self.horizontal_motor_commands:
            destination = float(command['destination'])
            if 0 > destination > 180:
                self.error = True
                self.error_code = 'Mission not valid'
                return

        self.pre_checked = True
        self.error = False
        self.error_code = ''

    def initialize_antenna(self):
        self.mission_initialized = True

        if self.vertical_motor_commands.__len__() == 0 or self.horizontal_motor_commands.__len__() == 0 or \
                self.execution_times.__len__() == 0:
            self.error = True
            self.error_code = 'No valid commands'
            return

        first_azimuth_command = self.vertical_motor_commands[0]
        first_elevation_command = self.horizontal_motor_commands[0]
        self.start_time = self.execution_times[0]

        azimuth_initialization_command = [{ 'duration': 30, 'destination': first_azimuth_command['destination']}]
        elevation_initialization_command = [{'duration': 30, 'destination': first_elevation_command['destination']}]

        print('INITIAL_COMMAND_SENT')
        print(azimuth_initialization_command)
        print(elevation_initialization_command)

        vertical_publisher.publish(json.dumps(azimuth_initialization_command))
        horizontal_publisher.publish(json.dumps(elevation_initialization_command))


        time.sleep(1)

        self.mission_initialized = True

    def send_all_command(self):
        print('COMMANDS SENT')
        self.mission_started = False

        vertical_publisher.publish(json.dumps(self.vertical_motor_commands))
        horizontal_publisher.publish(json.dumps(self.horizontal_motor_commands))

    def start_mission(self):
        self.mission_started = True
        now = datetime.now()

        # FOR TESTING DELAY IS SET 10 seconds in future
        # Replace run_at with self.start_time in production

        run_at = now + timedelta(seconds=1)
        delay = (run_at - now).total_seconds()

        threading.Timer(delay, self.send_all_command).start()

        return

    def initialize_mission(self):
        self.error_code = ''
        self.sender_thread = None

        self.check_commands()

        print('COMMANDS CHECKED')

        if self.error:
            return

        self.initialize_antenna()

        print('ANTENNA INITIALIZED')

        if not self.mission_initialized:
            return

        print('START_TIME')
        print(self.start_time)

        self.start_mission()

    def add_new_mission_by_file(self, filename):
        with open(filename) as csv_file:
            reader = csv.reader(csv_file, delimiter=',', quotechar='"')

            for row in reader:
                azimuth = {'destination': row[1], 'duration': row[5]}
                elevation = {'destination': row[2], 'duration': row[5]}

                self.vertical_motor_commands.append(azimuth)
                self.horizontal_motor_commands.append(elevation)
                self.execution_times.append(row[0])

        self.initialize_mission()

    def callback(self, payload):
        data = json.loads(payload.data)

        for row in data:
            azimuth = {'destination': row[1], 'duration': row[5]}
            elevation = {'destination': row[2], 'duration': row[5]}

            self.vertical_motor_commands.append(azimuth)
            self.horizontal_motor_commands.append(elevation)
            self.execution_times.append(row[0])

        self.initialize_mission()

    def listener(self):
        rospy.Subscriber("tracking_station", String, self.callback)
        rospy.spin()


if __name__ == '__main__':
    ControllerService().add_new_mission_by_file('input.csv')
    # In production switch to listener
