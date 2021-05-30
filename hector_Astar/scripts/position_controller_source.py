#!/usr/bin/env python3

"""
Author: Pengkai Ru (pengkai.ru@gmail.com)

Description: Code for quadrotor to maintain a certain position
"""

import numpy
import roslib
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty      #empty message for landing
from geometry_msgs.msg import Twist #control movement
from geometry_msgs.msg import PoseStamped
from hector_uav_msgs.msg import *
from math import *
import scipy.io as sio


try:
    # Python 2
    xrange
except NameError:
    # Python 3, xrange is now named range
    xrange = range

class PID:
    def __init__(self, _kp, _ki, _kd, _dt,):
        self.kp, self.ki, self.kd = _kp, _ki, _kd
        self.dt = _dt;
        self.enabled = True;
        self.err_prev = 0
        self.err_int  = 0

    def reset(self):
        self.err_prev = 0
        self.err_int  = 0

    def cal_output(self, err):
        self.err_int += err;
        output = -( self.kp*(err) + self.ki*(self.err_int) + self.kd*((err - self.err_prev)/self.dt) )
        self.err_prev = err;
        return output


class position_controller:
    quad_pos_x = 0
    quad_pos_y = 0
    quad_pos_z = 0


    def __init__(self,_n):
        self.n = _n
        # determine which quad to call
        s = "/quad"+str(self.n)


        # subscribe to pose of quadrotor
        self.quad_pose_sub = rospy.Subscriber(s + "/ground_truth_to_tf/pose", PoseStamped, self.pose_callback)


        self.quad_pos_x_d = self.quad_pos_x
        self.quad_pos_y_d = self.quad_pos_y
        self.quad_pos_z_d = self.quad_pos_z
        # subscribe to reference
        self.quad_pose_r_sub = rospy.Subscriber(s + "/trajectory_ref", PoseStamped, self.quad_pose_r_callback) 

        # publish to cmd_vel to power quad
        self.mov_pub  = rospy.Publisher(s + "/cmd_vel", Twist, queue_size = 1)

    def quad_pose_r_callback(self, pose_r_data):
        self.quad_pos_x_d = pose_r_data.pose.position.x
        self.quad_pos_y_d = pose_r_data.pose.position.y
        self.quad_pos_z_d = pose_r_data.pose.position.z

    def pose_callback(self, pose_data):
        self.quad_pos_x = pose_data.pose.position.x
        self.quad_pos_y = pose_data.pose.position.y
        self.quad_pos_z = pose_data.pose.position.z

        
        print ("Desired position" + str(self.n) + ": ")
        print (self.quad_pos_x_d, self.quad_pos_y_d, self.quad_pos_z_d)
        print ("Current position " + str(self.n) + ": ")
        print (self.quad_pos_x, self.quad_pos_y, self.quad_pos_z)
        # print 'Current position:'
        # print (quad_pos_x, quad_pos_y, quad_pos_z)
        twist = Twist()
        PID_ctrller = PID(1, 0, 0.001, 0.05)
        twist.linear.x = PID_ctrller.cal_output(self.quad_pos_x - self.quad_pos_x_d)
        twist.linear.y = PID_ctrller.cal_output(self.quad_pos_y - self.quad_pos_y_d)
        twist.linear.z = PID_ctrller.cal_output(self.quad_pos_z - self.quad_pos_z_d)
        self.mov_pub.publish(twist)
        # twist.linear.x = -1*(quad_pos_x - self.quad_pos_x_d)
        # twist.linear.y = -1*(quad_pos_y - self.quad_pos_y_d)
        # twist.linear.z = -1*(quad_pos_z - self.quad_pos_z_d)
        # print 'Command Velocity:'
        # print (twist.linear.x, twist.linear.y, twist.linear.z)
def pos_ctrl_6(n):
    for i in xrange(n):
        try:
            position_controller(i+1)
        except rospy.ROSInterruptException:
            print ("Quad") + str(i+1) + ("cant publish command") 



def shutdown_callback():
    print ("Shutting down position controller.")
    # for i in xrange(n):
    #     try:
    #         position_controller(i+1)

if __name__ == "__main__":
    rospy.init_node("quadcopter_position_controller")
    quad_cmd_vel_pub = rospy.Publisher("/quad1/cmd_vel", Twist, queue_size=10)
    pos_ctrl_6(6)
    rospy.on_shutdown(shutdown_callback)
    rospy.sleep(0.01)
    rospy.spin()
