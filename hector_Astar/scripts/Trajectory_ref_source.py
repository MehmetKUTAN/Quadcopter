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
import math
import scipy.io as sio
import time

try:
    # Python 2
    xrange
except NameError:
    # Python 3, xrange is now named range
    xrange = range

def quadcopter_trajectory_ref_pub(dt, x_r, k, n, X):
    rospy.init_node("quad_trajectory_ref_pub", anonymous=True)
    l_pub = []
    l_pose = []
    for j in xrange(5):
        s_topic = "/quad"+str(j+1)+"/trajectory_ref"
        l_pub.append(rospy.Publisher(s_topic, PoseStamped, queue_size=10))
        l_pose.append( PoseStamped() )

    # quad1_trajectory_ref_pub = rospy.Publisher('/quad1/trajectory_ref', PoseStamped, queue_size=10)
    # quad2_trajectory_ref_pub = rospy.Publisher('/quad2/trajectory_ref', PoseStamped, queue_size=10)

    sub_clock= rospy.Time.from_sec(time.time())
    starting_time = sub_clock.to_sec()

    # rate = rospy.Rate(1/dt)
    rate = rospy.Rate(20)
    k = 550
    i = -k
    while not rospy.is_shutdown():
        sim_time = rospy.Time.from_sec(time.time()).to_sec()
        print ("Time in secs:")
        print (sim_time - starting_time)
        i  = int(math.floor((sim_time - starting_time)/dt))
        i += 1
        print ("i = "),i
        if i >= 0:
            ii = i
            for j in xrange(5):
                l_pose[j].pose.position.x, l_pose[j].pose.position.y, l_pose[j].pose.position.z = X[0+j*12,ii], X[1+j*12,ii], X[2+j*12,ii]+8
                print ("pozitif")
                l_pub[j].publish(l_pose[j])   
        else: 
            for j in xrange(5):
                l_pose[j].pose.position.x, l_pose[j].pose.position.y, l_pose[j].pose.position.z = X[0+j*12,0], X[1+j*12,0], X[2+j*12,0]+(i+float(k))/k*8
                print ("negatif")
                print (l_pose[j].pose.position.x, l_pose[j].pose.position.y, l_pose[j].pose.position.z)
                l_pub[j].publish(l_pose[j])   
             
        # Pose_quad1.pose.position.x, Pose_quad1.pose.position.y, Pose_quad1.pose.position.z = X[0,i], X[1,i], X[2,i]+8
        # quad1_trajectory_ref_pub.publish(Pose_quad1)
        rate.sleep()



if __name__ == "__main__":
    formation_change_data = sio.loadmat("/home/mkutan/Desktop/gazebo_quadcopter/src/hector_Astar/scripts/Formation_change_data.mat")
    x_r,k,n,X = formation_change_data["x_r"],formation_change_data["k"],formation_change_data["n"][0][0],formation_change_data["X"]
    dt = 0.05
    try:
        quadcopter_trajectory_ref_pub(dt,x_r,k,n,X)
    except rospy.ROSInterruptException:
        pass

