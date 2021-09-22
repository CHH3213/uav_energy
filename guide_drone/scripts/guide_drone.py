#!/usr/bin/env python

import rospy
import numpy as np
import os
import matplotlib.pyplot as plt

from std_msgs.msg import String
from mavros_msgs.msg import State
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import PositionTarget
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import RCOut
from mavros_msgs.msg import OverrideRCIn

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode


class Drone:
    def __init__(self, name): # try to add 'name' paremeter,which can be used as namespace
        #
        rospy.init_node(name, anonymous=True)

        # parameters
        self.log_dir = "../log"
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        # states
        self.state = State()
        self.battery = BatteryState()
        self.rcIn = RCIn()

        # clients
        rospy.loginfo("waiting for ROS services")
        rospy.wait_for_service('mavros/cmd/arming')     # make sure that your service is available
        rospy.wait_for_service('mavros/set_mode')

        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)       # name and srv type (rosservice info <srv_name>)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # pubs subs
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)

    # callback funcs

    def state_callback(self, data):
        # change notice
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))
        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.state.connected, data.connected))
        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, data.mode))
        self.state = data

    # helper methods

    def set_arm(self, arm):
        """arm: True to arm or False to disarm"""
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        loop_freq = 2  # 2Hz
        rate = rospy.Rate(loop_freq)
        counter = 0
        while self.state.armed != arm:
            rospy.logerr("failed to send arm command")
            self.arming_client(arm)
            counter = counter + 1
            rate.sleep()
        rospy.loginfo("set arm success | seconds: {0} ".format(counter/loop_freq))

    def set_mode(self, mode):
        """mode: APM mode string"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        loop_freq = 2  # 2Hz
        rate = rospy.Rate(loop_freq)
        counter = 0
        while self.state.mode != mode:
            print(self.state.mode)
            rospy.logerr("failed to send set mode command")
            self.set_mode_client(0, mode)
            counter = counter + 1
            rate.sleep()
        rospy.loginfo("set mode success | seconds: {0} ".format(counter/loop_freq))


if __name__ == '__main__':
    drone = Drone('drone')
    drone.set_arm(True)
    #drone.set_mode('GUIDED')    # use 'Guided' can also set mode successfully but can not make (self.state.mode != mode) True

