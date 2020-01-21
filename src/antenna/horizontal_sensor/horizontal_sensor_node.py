#! /usr/bin/env python

import json
import rospy
import socket
import sys
from time import sleep

from std_msgs.msg import String

rospy.init_node('horizontal_sensor')

horizontal_sensor_publisher = rospy.Publisher('/horizontal_sensors1', String, queue_size=10)


class SensorService:

    def listener(self):

        TCP_IP = '127.0.0.1'
        TCP_PORT = 2000
        BUFFER_SIZE = 10
        MESSAGE_OPEN_USB = '\x05\x00\x02\x67\x00'
        MESSAGE_REGULAR_PING = '\x07\x00\x02\x0e\x00\x00\x00'
        MESSAGE_GET_DEVICE_INFORMATION = '\x08\x00\x02\x02\x00\x00\x00\x00'
        MESSAGE_OPEN_DEVICE = '\x24\x00\x02\x0f\x00\x00\x00\x11\x00\x00' \
                              '\x00\x00\x01\x01\x3d\x0c\x00\x05\x00\x00\x00' \
                              '\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x20\x20'
        MESSAGE_POSITION_CAPTURE = '\x05\x00\x02\x0a\x00'



        print('Open USB Device')

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.close()
        s.connect((TCP_IP, TCP_PORT))
        s.send(bytearray(MESSAGE_OPEN_USB, 'utf-8'))
        data = s.recv(BUFFER_SIZE)

        print('Recieve data')
        print('Recieved data:', data)

        sleep(0.5)

        print('Regular Ping')
        s.send(bytearray(MESSAGE_REGULAR_PING, 'utf-8'))
        print('Recieve data')
        data = s.recv(BUFFER_SIZE)
        print('Recieved data:', data)

        sleep(0.5)

        print('Open Device Session')
        s.send(bytearray(MESSAGE_OPEN_DEVICE, 'utf-8'))
        print('Recieve data')
        data = s.recv(BUFFER_SIZE)
        print('Recieved data:', data)

        sleep(0.5)

        print('Get Device Information')
        s.send(bytearray(MESSAGE_GET_DEVICE_INFORMATION, 'utf-8'))
        print('Recieve data')
        data = s.recv(BUFFER_SIZE)
        print('Recieved data:', data)

        sleep(0.5)

        while True:

            s.send(bytearray(MESSAGE_REGULAR_PING, 'utf-8'))
            data = s.recv(BUFFER_SIZE)
            sleep(0.5)

            if data == '':
                break
            else:
                s.send(bytearray(MESSAGE_POSITION_CAPTURE, 'utf-8'))
                data = s.recv(BUFFER_SIZE)
                horizontal_sensor_publisher.publish(data)
                print('Posistion: ' + data)
                sleep(1)

        print('SENSORS RUNNING')



if __name__ == '__main__':
    sensor_service = SensorService()
    sensor_service.listener()