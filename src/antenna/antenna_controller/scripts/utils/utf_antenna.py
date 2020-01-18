#!/usr/bin/env python
__author__ = 'Mart Hytt'

import serial


class UTFAntenna:

    GET_ELEVATION = 'B'
    GET_AZIMUTH = 'C'
    MOVE_TO = 'W'
    BAUD_RATE = 9600

    def __init__(self, serial_post):
        self._port = serial.Serial(serial_post, self.BAUD_RATE, bytesize=8, stopbits=1, timeout=0.5, xonxoff=0, rtscts=0)

    def __delete__(self):
        self._port.close()

    def _send_cmd(self, command):
        self._port.write((command + "\r").encode())

    def get_azimuth(self):
        self._send_cmd(self.GET_AZIMUTH)

        azimuth = self._port.readline().strip()
        azimuth = int(azimuth[3:6])
        return azimuth

    def get_elevation(self):
        self._send_cmd(self.GET_ELEVATION)

        elevation = self._port.readline().strip()
        elevation = int(elevation[3:6])
        elevation = 90 - elevation
        return elevation

    def rotate(self, azimuth, elevation):
        if azimuth < 0 or azimuth > 360:
            return

        if elevation < 0 or elevation > 90:
            return

        elevation = 90 - int(round(elevation, 0))
        azimuth = int(round(azimuth, 0))
        command = self.MOVE_TO + ' ' + str(azimuth) + ' ' + str(elevation)

        self._send_cmd(command)


if __name__ == "__main__":
    r = UTFAntenna("/dev/ttyUSB0")

    print(r.get_azimuth())
    print(r.get_elevation())
