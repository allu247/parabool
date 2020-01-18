#!/usr/bin/env python
__author__ = 'Rasmus Tomsen'

from antenna_msgs.msg import Weather_information
import rospy
import json
import urllib2
import schedule
import time


class WeatherTracker():
    """docstring for WeatherTracker"""

    def __init__(self):
        self.API_URL = ""

    def weather_schedule(self):
        """
        Asks for new weather information every 10 seconds.
        """
        rospy.init_node('weather_information')
        self.API_URL = self.get_api_url()
        self.track_weather()

        schedule.every(10).seconds.do(self.track_weather)
        while True:
            schedule.run_pending()
            time.sleep(1)

    def track_weather(self):
        """
        Gets weather data and publishes it.
        """
        weather_data = self.get_weather_data()
        wind_direction = weather_data['wind_deg']
        antenna_direction_el = 4
        antenna_direction_az = self.calculate_antenna_direction(wind_direction)
        dt = weather_data['dt']
        wind_speed = weather_data['wind']
        should_stop_antenna = False
        if wind_speed >= 10:
            should_stop_antenna = True
        self.publish_weather_data(should_stop_antenna, wind_speed, wind_direction, antenna_direction_az,
                                  antenna_direction_el, dt)

    def publish_weather_data(self, should_stop_antenna, wind_speed, wind_direction, antenna_az, antenna_el, dt):
        """
        Publishes current weather data to parameter server.
        :param should_stop_antenna: Boolean whether the wind is too strong to continue antenna's work
        :param wind_speed: Wind speed
        :param wind_direction: Wind direction azimuth
        :param antenna_az: Azimuth to turn the antenna to
        :param antenna_el: Elevation to turn the antenna to
        :param dt: Prediction date time
        """
        data = {"should_stop_antenna": should_stop_antenna,
                "wind_speed": wind_speed,
                "wind_direction": wind_direction,
                "antenna_az": antenna_az,
                "antenna_el": antenna_el,
                "dt": dt}
        if rospy.has_param('weather_information'):
            has_stopped = rospy.get_param('weather_information')["should_stop_antenna"]
            if not has_stopped == should_stop_antenna:
                rospy.set_param('weather_information', data)
        else:
            rospy.set_param('weather_information', data)

    def calculate_antenna_direction(self, wind_direction):
        return (wind_direction + 90) % 360

    def get_weather_data(self):
        """
        Asks for the newest weather data from openweathermap API.
        """
        request = urllib2.Request(self.API_URL)
        response = urllib2.urlopen(request)
        output = response.read().decode('utf-8')
        return self.get_formatted_data(json.loads(output))

    def get_formatted_data(self, data):
        """
        Returns weather data in better format.
        """
        first_prediction = data.get('list')[0]
        formatted_data = dict(
            city=data.get('city').get('name'),
            country=data.get('city').get('country'),
            wind=first_prediction.get('wind').get('speed'),
            wind_deg=first_prediction.get('wind').get('deg'),
            dt_txt=first_prediction.get('dt_txt'),
            dt=first_prediction.get('dt')
        )
        return formatted_data

    def get_api_url(self):
        latitude = '59.394870'
        longitude = '24.661399'
        api_key = 'c82cf06f9f08cb91929635b916ee19b6'
        url = 'http://api.openweathermap.org/data/2.5/forecast?lat={}&lon={}&appid={}'.format(latitude, longitude,
                                                                                              api_key)
        return url


if __name__ == '__main__':
    WeatherTracker().weather_schedule()
