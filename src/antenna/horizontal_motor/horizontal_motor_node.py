#! /usr/bin/env python

import json
import time
import rospy

import sys
import inspect, os

BASE_LOCATION = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

sys.path.append(BASE_LOCATION + '/service')
sys.path.append(BASE_LOCATION + '/domain')

from std_msgs.msg import String


from RegisterService import RegisterService
from PID import PID

rospy.init_node('horizontal_motor_controller')


class RotationService:

    def __init__(self, port, slave):
        self.port = port
        self.slave = slave
        self.command_queue = []
        self.execution_started = False
        self.stop_mission = False
        self.register_service = RegisterService(port, slave)
        self.motor_rpm = 935
        self.transmission_constant = 3132
        self.max_turn_range = 180
        self.error = False
        self.error_code = ''
        self.pre_checked = False
        self.pid = PID(0.2, 0.0, 0.0)
        self.current_pos = 0.0

    def save_new_configuration(self, port, slave):
        if port.__len__() != 0:
            self.port = port
        if slave.__len__() != 0:
            self.slave = int(slave)

        self.command_queue = []
        self.execution_started = False
        self.register_service = RegisterService(port, slave)

        return self.get_configuration()

    def get_configuration(self):
        return {"port": self.port, "slave": self.slave}

    def check_commands(self):
        if self.command_queue.__len__() is 0:
            self.error = True
            self.error_code = 'Commands are missing'
            self.stop_mission = True
            return

        for command in self.command_queue:
            destination = float(command.get('destination'))
            if 0 > destination > self.max_turn_range:
                self.error = True
                self.error_code = 'Mission not valid'
                self.stop_mission = True
                return

        self.pre_checked = True
        self.error = False
        self.error_code = ''

        print('CHECK COMPLETED')

    def add_mission(self, commands):
        while self.execution_started:
            print("WAITING TO EXECUTE")
            time.sleep(1)

        self.command_queue = commands
        self.check_commands()

        if self.command_queue.__len__() is 1:
            self.initalize_antenna()
        else:
            self.execute()

        return {"status": "ADDED"}

    def stop(self):
        self.stop_mission = True

    def initalize_antenna(self):
        current_command = self.command_queue.pop(0)
        self.initialize_inverter()
        self.execution_started = True

        destination = float(current_command.get("destination"))
        angle = destination - self.current_pos

        self.change_rotation_direction(angle)

    def initialize_inverter(self):
        # Speed
        self.register_service.write_to_register("0002", "0")
        # Turn device on
        self.register_service.write_to_coil("0001", "1")
        # Reset rotation
        self.register_service.write_to_coil("0002", "0")

    def change_rotation_direction(self, angle):
        if angle >= 0:
            self.register_service.write_to_coil("0002", "0")
        else:
            self.register_service.write_to_coil("0002", "1")

    def check_rotation_direction(self, current_command, next_command):
        current_destination = float(current_command.get("destination"))
        next_destination = float(next_command.get("destination"))

        angle = next_destination - current_destination
        self.change_rotation_direction(angle)

    def get_frequency(self, command):
        if not command:
            return 0

        duration = float(command.get("duration"))
        destination = float(command.get("destination"))

        self.pid.SetPoint(destination)
        self.pid.update(self.current_pos)

        t1 = duration / 60
        a = abs(destination - self.current_pos + self.pid.output)

        if a <= 0 or t1 <= 0:
            return 0

        # TODO: REMOVE THIS LINE IN PRODUCTION
        self.current_pos = destination

        return ((self.transmission_constant / t1) * (1 / (360 / a)) * 50) / self.motor_rpm

    def execute(self):
        print('EXECUTE STARTED')

        while len(self.command_queue) > 0 or not self.stop_mission:
            if not self.execution_started:
                self.initialize_inverter()
                self.execution_started = True

            next_command = None
            current_command = self.command_queue.pop(0)

            if self.command_queue.__len__() > 0:
                next_command = self.command_queue[0]

            if next_command is not None:
                self.check_rotation_direction(current_command, next_command)

            current_calculated_speed = self.get_frequency(current_command)

            print('SPEED')
            print(current_calculated_speed)

            duration = float(current_command.get("duration"))

            self.register_service.write_to_register("0002", current_calculated_speed)
            time.sleep(duration)


        self.execution_started = False
        self.stop_mission = False
        self.pre_checked = False
        self.register_service.write_to_register("0002", "0")
        self.register_service.write_to_coil("0001", "0")

        print('EXECUTE FINISHED')

    def callback(self, payload):
        commands = json.loads(payload.data)
        self.add_mission(commands)

    def update_current_pos(self, payload):
        self.current_pos = float(payload.data)
        print('CURRENT_POS_UPDATED')


    def listener(self):
        rospy.Subscriber("horizontal_motor_commands", String, self.callback)
        rospy.Subscriber("horizontal_sensors1", String, self.update_current_pos)
        rospy.spin()


if __name__ == '__main__':
    rotation_service = RotationService('UNKNOW PORT', 1)
    rotation_service.listener()