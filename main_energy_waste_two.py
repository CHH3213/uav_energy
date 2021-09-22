#!/usr/bin/python2.7
# -*- coding utf-8 -*-

'''
#############chh###########
优化无人机能耗问题
#########################
'''
from __future__ import print_function, absolute_import, division

# ROS packages required
import rospy
import rospkg
from std_srvs.srv import Empty
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3

from mavros_msgs.msg import ActuatorControl
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import GlobalPositionTarget
from mavros_msgs.msg import Thrust
from mavros_msgs.msg import RCOut
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetModeRequest
from mavros_msgs.srv import SetModeResponse
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import CommandBoolRequest
from mavros_msgs.srv import CommandBoolResponse
from mavros_msgs.srv import StreamRate, StreamRateRequest

from math import *
from sensor_msgs.msg import Imu
from sensor_msgs.msg import BatteryState

from std_msgs.msg import Header
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyRequest

import math

from std_srvs.srv import Empty
# from gazebo_msgs.srv import SetModelState, GetModelState  # 设置模型状态、得到模型状态
# from gazebo_msgs.msg import ModelState, ModelStates
# from gazebo_msgs.srv import *
# from gazebo_msgs.srv import SetModelStateRequest
# from gazebo_msgs.srv import GetLinkState
# from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import  Odometry

import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import os
import time
from optimizer_energy_two import minimizeForce
from compute_rotated import *

class energy_waste:
    def __init__(self):


        self.payload_pos = np.array([0.25,0,1])
        self.payload_vector = np.array([1,0,-1])
        # 绳子方向,指向等边三角形顶点：（1，0，1），（-1，0，1），（0，np.sqrt(3),1)
        self.triangle_angle_1 = np.array([0.5,0,1])
        self.triangle_angle_2 = np.array([-0.5,0,1])
        self.cable_length = 0.5  # 绳子长度
        self.k_t =5.215  # 功率系数
        self.local_position1 = PoseStamped()
        self.local_position2 = PoseStamped()
        self.local_position3 = PoseStamped()
        self.rc_states1 = RCOut()
        self.rc_states2 = RCOut()
        self.rc_states3 = RCOut()
        self.mavros_state1 = State()
        self.mavros_state2 = State()
        self.mavros_state3 = State()
        #无人机相对世界坐标系的初始点
        self.world_drone1 = np.array([1,0.,1])
        self.world_drone2 = np.array([-1,0,1])
        #无人机重力矢量
        self.gravity_drone1 = np.array([0,0,-9.8*1.888])
        self.gravity_drone2 = np.array([0,0,-9.8*1.888])
        # 初始推力矢量--竖直向上 用四元数，后面补了0
        self.init_drone_thrust = np.array([0,0,1,0])  

        ## ROS Subscribers
        self.local_pos_sub1 = rospy.Subscriber("/drone1/mavros/local_position/pose", PoseStamped, self.lp_cb1, queue_size=1)
        self.local_pos_sub2 = rospy.Subscriber("/drone2/mavros/local_position/pose", PoseStamped, self.lp_cb2, queue_size=1)
        self.rc_out1 = rospy.Subscriber("/drone1/mavros/rc/out", RCOut, self.rc_cb1, queue_size=1)
        self.rc_out2 = rospy.Subscriber("/drone2/mavros/rc/out", RCOut, self.rc_cb2, queue_size=1)
        self.state_sub1 = rospy.Subscriber('/drone1/mavros/state', State, self.state_callback1)
        self.state_sub2 = rospy.Subscriber('/drone2/mavros/state', State, self.state_callback2)


        ## ROS Publishers
        self.setpoint_raw_pub1 = rospy.Publisher("/drone1/mavros/setpoint_raw/attitude",AttitudeTarget,queue_size=1)
        self.setpoint_raw_pub2 = rospy.Publisher("/drone2/mavros/setpoint_raw/attitude",AttitudeTarget,queue_size=1)
        self.setposition_raw_pub1 = rospy.Publisher("/drone1/mavros/setpoint_raw/local",PositionTarget,queue_size=1)  # NED
        self.setposition_raw_pub2 = rospy.Publisher("/drone2/mavros/setpoint_raw/local",PositionTarget,queue_size=1)  # NED

        self.local_pos_pub1 = rospy.Publisher("/drone1/mavros/setpoint_position/local",PoseStamped,queue_size=1) #ENU
        self.local_pos_pub2 = rospy.Publisher("/drone2/mavros/setpoint_position/local",PoseStamped,queue_size=1)



        ## ROS mavros Services
        rospy.wait_for_service('drone1/mavros/cmd/arming')
        self.arming_client1 = rospy.ServiceProxy('drone1/mavros/cmd/arming', CommandBool)
        rospy.wait_for_service('drone2/mavros/cmd/arming')
        self.arming_client2 = rospy.ServiceProxy('drone2/mavros/cmd/arming', CommandBool)



        rospy.wait_for_service('drone1/mavros/cmd/takeoff')
        self.takeoff_service1 = rospy.ServiceProxy('/drone1/mavros/cmd/takeoff', CommandTOL)
        rospy.wait_for_service('drone2/mavros/cmd/takeoff')
        self.takeoff_service2 = rospy.ServiceProxy('/drone2/mavros/cmd/takeoff', CommandTOL)



        rospy.wait_for_service('drone1/mavros/set_mode')
        self.set_mode_client1 = rospy.ServiceProxy('drone1/mavros/set_mode', SetMode)
        rospy.wait_for_service('drone2/mavros/set_mode')
        self.set_mode_client2 = rospy.ServiceProxy('drone2/mavros/set_mode', SetMode)



        ### Initiate ROS node
        print('-- Connecting to mavros')
        rospy.init_node('energy',anonymous=True)
        print ('connected')


    def lp_cb1(self,data):
        self.local_position1 = data
        # print('GPSpos',self.local_position.pose.position.x,self.local_position.pose.position.y,self.local_position.pose.position.z)
    def lp_cb2(self,data):
        self.local_position2 = data
        # print('GPSpos',self.local_position.pose.position.x,self.local_position.pose.position.y,self.local_position.pose.position.z)


    def rc_cb1(self,data):
        self.rc_states1 = data
    def rc_cb2(self,data):
        self.rc_states2 = data


    def state_callback1(self, data):
        self.mavros_state1 = data
    
    def state_callback2(self, data):
        self.mavros_state2 = data
        


    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly,
        land, and disarm.
        """
        self.set_mode_client1(custom_mode="9")
        self.arming_client1(False)
        self.set_mode_client2(custom_mode="9")
        self.arming_client2(False)


    def set_arm(self, arm):
        """arm: True to arm or False to disarm"""
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        loop_freq = 2  # 2Hz
        rate = rospy.Rate(loop_freq)
        counter = 0
        while self.mavros_state1.armed != arm or self.mavros_state2.armed != arm:
            rospy.logerr("failed to send arm command")
            self.arming_client1(arm)
            self.arming_client2(arm)
            counter = counter + 1
            rate.sleep()
        rospy.loginfo("set arm success | seconds: {0} ".format(counter / loop_freq))

    def set_mode(self, mode):
        """mode: APM mode string"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        loop_freq = 2  # 2Hz
        rate = rospy.Rate(loop_freq)
        counter = 0
        while self.mavros_state1.mode != mode or self.mavros_state2.mode != mode:
            print('current mode', self.mavros_state1.mode, self.mavros_state2.mode)
            rospy.logerr("failed to send set mode command")
            self.set_mode_client1(0, mode)
            self.set_mode_client2(0, mode)
            counter = counter + 1
            rate.sleep()
        rospy.loginfo("set mode success | seconds: {0} ".format(counter / loop_freq))


    def goto_xyz_rpy(self, x, y, z, ro, pi, ya):
        '''
        Set the given pose as a the next setpoint by sending
        a SET_POSITION_TARGET_LOCAL_NED message. The copter must
        be in GUIDED mode for this to work.
        :param x:
        :param y:
        :param z:
        :param ro:
        :param pi:
        :param ya:
        :return:
        '''
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        quat = self.euler_to_quaternion(ro,pi,ya+np.pi/2)

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time()
        pose_stamped.pose = pose
        return  pose_stamped


    def get_posByMavros(self):
        '''
        使用mavros话题获取无人机状态
        :return: 返回无人机的位置和姿态
        '''
        pos_drone1 = np.array([self.local_position1.pose.position.x,self.local_position1.pose.position.y,self.local_position1.pose.position.z])
        pos_drone2 = np.array([self.local_position2.pose.position.x,self.local_position2.pose.position.y,self.local_position2.pose.position.z])
        ori_drone1 = np.array([self.local_position1.pose.orientation.x,self.local_position1.pose.orientation.y,self.local_position1.pose.orientation.z,self.local_position1.pose.orientation.w])
        ori_drone2 = np.array([self.local_position2.pose.orientation.x,self.local_position2.pose.orientation.y,self.local_position2.pose.orientation.z,self.local_position2.pose.orientation.w])

        return [pos_drone1,pos_drone2,ori_drone1,ori_drone2]

    

    def estimate_fixed_point(self,thrust1_mod,thrust2_mod,bias_pos_drone1,bias_pos_drone2):
        '''
        由无人机位置估算出固定点的位置
        '''
        # 无人机位置
        [pos_drone1,pos_drone2,ori_drone1,ori_drone2] = self.get_posByMavros()  # 相对自身原点的位置
        '''# 推力向量计算---需要使用姿态信息 旋转矩阵*旋转前向量'''  
        # q1 = np.dot(quaternion_to_rotation_matrix(ori_drone1),self.init_drone_thrust)
        # q2 = np.dot(quaternion_to_rotation_matrix(ori_drone2),self.init_drone_thrust)
        # q3 = np.dot(quaternion_to_rotation_matrix(ori_drone3),self.init_drone_thrust)
        # 四元数形式
        q1 = rotated_vector(ori_drone1,self.init_drone_thrust)
        q2 = rotated_vector(ori_drone2,self.init_drone_thrust)

        thrust1_unit = q1[0:3]/np.linalg.norm(q1[0:3])
        thrust2_unit = q2[0:3]/np.linalg.norm(q2[0:3])
        thrust1_vector = np.array(thrust1_unit)*thrust1_mod
        thrust2_vector = np.array(thrust2_unit)*thrust2_mod
        # 绳子拉力向量
        self.cable1_vector = thrust1_vector -self.gravity_drone1
        self.cable2_vector = thrust2_vector -self.gravity_drone2
        #单位向量
        cable1_u = self.cable1_vector/np.linalg.norm(self.cable1_vector)
        cable2_u = self.cable2_vector/np.linalg.norm(self.cable2_vector)

        # 固定点计算  # 公式10
        fixed_point1  = np.array(pos_drone1)+np.array(bias_pos_drone1) +self.world_drone1-  cable1_u * self.cable_length  
        fixed_point2  = np.array(pos_drone2)+np.array(bias_pos_drone2) +self.world_drone2-  cable2_u * self.cable_length
        # 返回世界坐标系下的坐标
        return fixed_point1,fixed_point2

    def estimate_payload_vector(self,cable1,cable2,fixed_point1,fix_point2,payload_point):
        '''
        已知绳子拉力向量和作用点，估计负载向量
        '''
        # patload_point_X_vector = -(np.cross(fixed_point1,np.array(cable1))+np.cross(fix_point2,np.array(cable2))+np.cross(fix_point3,np.array(cable3)))
        # payload_vector = np.array([patload_point_X_vector[2]/payload_point[1],patload_point_X_vector[0]/payload_point[2],patload_point_X_vector[1]/payload_point[0]])
        payload_vector = -(cable1 + cable2)
        return payload_vector

    def compute_optimize(self,args):
        '''
        :return: 优化后的无人机位置
        '''
        # print(np.array([0,0,-1])[2])
        # 返回优化后的绳子拉力向量
        v_max,v_sum,result_max,result_sum  = minimizeForce(args)
        # 单位化
        u1_max = np.array(v_max[0:3])/np.linalg.norm(v_max[0:3])
        u2_max = np.array(v_max[3:6])/np.linalg.norm(v_max[3:6])
        u1_sum = np.array(v_sum[0:3])/np.linalg.norm(v_sum[0:3])
        u2_sum = np.array(v_sum[3:6])/np.linalg.norm(v_sum[3:6])
        # 最大化最小化后的位置  公式15
        m_position_drone1 = u1_max * self.cable_length+self.triangle_angle_1  
        m_position_drone2 = u2_max * self.cable_length+self.triangle_angle_2
        # 求和最小化后的位置
        s_position_drone1 = u1_sum * self.cable_length+self.triangle_angle_1
        s_position_drone2 = u2_sum * self.cable_length+self.triangle_angle_2
        return m_position_drone1, m_position_drone2,s_position_drone1,s_position_drone2

    
    def run(self):
        #切换模式，解锁，起飞
        self.set_mode(mode='GUIDED')
        self.set_arm(arm=True)
        [self.init_drone1,self.init_drone2,_,_] = self.get_posByMavros()# 初始坐标
        print('^^^^^^^^^initial^^^^^^^^^^')
        print('self.init_drone1',self.init_drone1)
        print('self.init_drone2',self.init_drone2)
        flag = input('takeoff--0.5m==>continue:1, over:2')
        if flag ==2:
            self.land()
        else:
            while True:
                self.takeoff_service1(altitude=0.5+self.init_drone1[2])   # 预计飞到1.5m处，加上一开始的偏置误差
                self.takeoff_service2(altitude=0.5+self.init_drone2[2])
                [drone1,drone2,_,_] = self.get_posByMavros()
                print('self.init_drone1',drone1)
                print('self.init_drone2',drone2)
                print('==============================')
                if np.abs(drone1[2]-0.5-self.init_drone1[2]) <0.2 and np.abs(drone2[2]-0.5-self.init_drone2[2]) <0.2 :
                    break
        flag = input('takeoff--1.0m==>continue:1, over:2')
        if flag ==2:
            self.land()
        else:
            while True:
                self.takeoff_service1(altitude=1+self.init_drone1[2])   # 预计飞到1.5m处，加上一开始的偏置误差
                self.takeoff_service2(altitude=1+self.init_drone2[2])
                [drone1,drone2,_,_] = self.get_posByMavros()
                print('self.init_drone1',drone1)
                print('self.init_drone2',drone2)
                print('=========================')
                if np.abs(drone1[2]-1-self.init_drone1[2]) <0.2 and np.abs(drone2[2]-1-self.init_drone2[2]) <0.2 :
                    break
        flag = input('takeoff--1.5m==>continue:1, over:2')
        if flag ==2:
            self.land()
        else:
            while True:
                self.takeoff_service1(altitude=1.5+self.init_drone1[2])   # 预计飞到1.5m处，加上一开始的偏置误差
                self.takeoff_service2(altitude=1.5+self.init_drone2[2])
                [drone1,drone2,_,_] = self.get_posByMavros()
                print('self.init_drone1',drone1)
                print('self.init_drone2',drone2)
                print('=======================')
                if np.abs(drone1[2]-1.5-self.init_drone1[2]) <0.2 and np.abs(drone2[2]-1.5-self.init_drone2[2]) <0.2:
                    break
        while True:
            flag = input('estimate_fixed_point==>continue:1, over:2')
            if flag == 2:
                break
            else:
                if self.rc_states1.channels ==[] or self.rc_states2.channels == [] :
                    mod_thrust1 = 0
                    mod_thrust2 = 0
                # 推力大小计算  pwm映射，公式16
                mod_thrust1 = 0.008138 * np.sum(self.rc_states1.channels) -4 * 9.211
                mod_thrust2 = 0.008138 * np.sum(self.rc_states2.channels) -4 * 9.211

                # 返回平台固定点位置--实际位置
                self.triangle_angle_1, self.triangle_angle_2= self.estimate_fixed_point( thrust1_mod=mod_thrust1,thrust2_mod=mod_thrust2,bias_pos_drone1= self.init_drone1,bias_pos_drone2= self.init_drone2)
                # 一开始需要估计负载向量
                payload_vector = self.estimate_payload_vector(self.cable1_vector,self.cable2_vector,self.triangle_angle_1,self.triangle_angle_2,self.payload_pos)
                print('payload_vector',payload_vector)
            flag = input('optimize==>continue:1, over:2')
            if flag == 2:
                break
            else:
                # 优化
                args = (self.triangle_angle_1,self.triangle_angle_2, self.payload_pos, payload_vector ,self.k_t)
                m_position_drone1, m_position_drone2,s_position_drone1,s_position_drone2 = self.compute_optimize(args)  #实际位置-世界坐标系

            #下次需要飞的位置---最小化求和
            temp_drone1 = s_position_drone1 - self.init_drone1-self.world_drone1
            temp_drone2 = s_position_drone2 - self.init_drone2-self.world_drone2
            print('===========second==========')
            print('second_temp_drone1',temp_drone1)
            print('second_temp_drone2',temp_drone2)
            flag = input('second optimized takeoff==>continue:1, over:2')
            if flag == 2:
                break
            else:
                pose_stamped_1 = self.goto_xyz_rpy(temp_drone1[0], temp_drone1[1], temp_drone1[2], 0, 0, 0)
                pose_stamped_2 = self.goto_xyz_rpy(temp_drone2[0], temp_drone2[1], temp_drone2[2], 0, 0, 0)

                self.local_pos_pub1.publish(pose_stamped_1)
                # rospy.sleep(0.0005)
                self.local_pos_pub2.publish(pose_stamped_2)

        self.land()

if __name__ == '__main__':
    env = energy_waste()
    env.run()

