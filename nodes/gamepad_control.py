#!/usr/bin/env python

import rospy
import pygame
import rospkg
import os
from bluerov_sim.msg import ActuatorCommands
from mavros_msgs.srv import CommandBool
from sensor_msgs.msg import Joy


class GamepadControlNode():
    def __init__(self, name):
        rospy.init_node(name)
        self.arm_vehicle()

        self.thrust = 0.0
        self.left_stick_vertical = 0.0
        # self.thrust_scaler = 0.5

        self.lateral_thrust = 0.0
        self.left_stick_horizontal = 0.0
        # self.lateral_thrust_scaler = 0.5
        
        self.vertical_thrust = 0.0
        self.right_stick_vertical = 0.0
        # self.vertical_thrust_scaler = 0.4
        
        self.yaw_rate = 0.0
        self.right_stick_horizontal = 0.0
        # self.yaw_rate_scaler = 0.2

        self.actuator_pub = rospy.Publisher("mixer/actuator_commands",
                                            ActuatorCommands,
                                            queue_size=1)                
        self.gamepad_sub = rospy.Subscriber("joy", Joy, self.gamepad_callback)

    def arm_vehicle(self):
        rospy.wait_for_service("mavros/cmd/arming")
        arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        while not arm(True).success:
            rospy.logwarn_throttle(1, "Could not arm vehicle. Keep trying.")
        rospy.loginfo("Armed successfully.")
    
    def disarm_vehicle(self):
        rospy.wait_for_service("mavros/cmd/arming")
        arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        arm(False)

    def run(self):
        rate = rospy.Rate(30.0)

        while not rospy.is_shutdown():
            self.handle_inputs()
            self.publish_message()
            rate.sleep()
    
    def gamepad_callback(self, msg):
        # self.a_pressed = msg.buttons[0]
        # self.b_pressed = msg.buttons[1]
        self.left_stick_vertical = msg.axes[1]
        self.left_stick_horizontal = msg.axes[0]
        self.right_stick_vertical = msg.axes[4]
        self.right_stick_horizontal = msg.axes[3]

    def handle_inputs(self):
        self.thrust = self.left_stick_vertical
        self.lateral_thrust = self.left_stick_horizontal
        self.vertical_thrust = self.right_stick_vertical
        self.yaw_rate = self.right_stick_horizontal
    
    def publish_message(self):
        msg = ActuatorCommands()
        msg.header.stamp = rospy.Time.now()
        msg.thrust = self.thrust
        msg.yaw = self.yaw_rate
        msg.lateral_thrust = self.lateral_thrust
        msg.vertical_thrust = self.vertical_thrust
        self.actuator_pub.publish(msg)


def main():
    node = GamepadControlNode("gamepad")
    node.run()


if __name__ == "__main__":
    main()