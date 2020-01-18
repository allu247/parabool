

import rospy
import json
from std_msgs.msg import String

rospy.init_node("listener_node")

class Listener:

    def __init__(self):
        self.update_current_pos_horizontal = 0.0
        self.commands_horizontal = ""
        self.update_current_pos_vertical = 0.0
        self.commands_vertical = ""



    def callback_horizontal(self, payload):
        print("Horizontal command")
        self.commands_horizontal = json.loads(payload.data)


    def update_current_pos_horizontal(self, payload):
        print("Horizontal position")
        self.update_current_pos_horizontal = float(payload.data)

    def callback_vertical(self, payload):
        print("Horizontal command")
        self.commands_vertical = json.loads(payload.data)

    def update_current_pos_vertical(self, payload):
        print("Horizontal position")
        self.update_current_pos_vertical = float(payload.data)


    def listener(self):
        rospy.Subscriber("horizontal_motor_commands", String, self.callback_horizontal)
        rospy.Subscriber("horizontal_sensors1", String, self.update_current_pos_horizontal)
        rospy.spin()


if __name__ == '__main__':

    topic_listener = Listener()
    topic_listener.listener()