#!/usr/bin/env python
__author__ = 'Rasmus Tomsen'

import rospy
from antenna_msgs.msg import Antenna_command
from std_msgs.msg import Float64


class AntennaController:
    def __init__(self):
        self.publish_leg_joint = ""
        self.publish_dish_joint = ""

    def move_antenna_gazebo(self, new_az, new_elev):
        """
        Publishes new position to gazebo
        :param new_az: New azimuth
        :param new_elev: New elevation
        """
        self.publish_leg_joint.publish(new_az)
        self.publish_dish_joint.publish(new_elev)

    def callback(self, data):
        self.move_antenna_gazebo(data.leg_angle, data.dish_angle)

        # For testing purposes:
        open('/home/rasmus/catkin_ws/src/antenna/antenna_controller/output/acc_vel_ang.csv', 'w').close()
        with open('/home/rasmus/catkin_ws/src/antenna/antenna_controller/output/acc_vel_ang.csv', 'a') as f:
            f.write("{},{},{},{},{},{}".format(data.leg_linear_acceleration,
                                               data.leg_linear_velocity,
                                               data.dish_linear_acceleration,
                                               data.dish_linear_velocity,
                                               data.leg_angle,
                                               data.dish_angle))
            f.write("\n")
        rospy.loginfo("\nPUBLISHED DATA: \n\t{}".format(data))

    def subscribe(self):
        rospy.Subscriber('/antenna/antenna_command', Antenna_command, self.callback)
        rospy.spin()

    def main(self):
        self.publish_leg_joint = rospy.Publisher("/antenna/leg_joint_position_controller/command",
                                                 Float64, queue_size=10)
        self.publish_dish_joint = rospy.Publisher("/antenna/dish_joint_position_controller/command",
                                                  Float64, queue_size=10)
        rospy.init_node('antenna_controller', anonymous=False)
        self.subscribe()


if __name__ == '__main__':
    AntennaController().main()
