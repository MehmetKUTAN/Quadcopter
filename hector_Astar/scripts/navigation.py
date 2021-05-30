#!/usr/bin/env python3

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
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

try:
    # Python 2
    xrange
except NameError:
    # Python 3, xrange is now named range
    xrange = range

def quadcopter_trajectory_ref_pub(X,Y,Z):
    rospy.init_node("quad_trajectory_ref_pub", anonymous=True)
    l_pub = []
    l_pose = []


    for j in xrange(10):
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
    i = k
    i = 0
    while not rospy.is_shutdown():
        sim_time = rospy.Time.from_sec(time.time()).to_sec()
        print ("Time in secs:")
        print (sim_time - starting_time)
        i  = int(math.floor((sim_time - starting_time)/dt))
        i += 1
        print ("i ="),i
        hareket=0
        #if i < 100:
        ii = i
        for j in xrange(5):
            if(j==0):
                l_pose[j].pose.position.x, l_pose[j].pose.position.y,l_pose[j].pose.position.z=0,0,10
                l_pub[j].publish(l_pose[j])
                if(sim_time - starting_time>20):
                    l_pose[j].pose.position.x, l_pose[j].pose.position.y,l_pose[j].pose.position.z=-7,5.5,10
                    l_pub[j].publish(l_pose[j])
                    if(sim_time - starting_time>35):
                        l_pose[j].pose.position.x, l_pose[j].pose.position.y,l_pose[j].pose.position.z=4,5,10
                        l_pub[j].publish(l_pose[j])
                        if(sim_time - starting_time>50):
                            l_pose[j].pose.position.x, l_pose[j].pose.position.y,l_pose[j].pose.position.z=4,-5,10
                            l_pub[j].publish(l_pose[j])
                            if(sim_time - starting_time>65):
                                l_pose[j].pose.position.x, l_pose[j].pose.position.y,l_pose[j].pose.position.z=-9.5,-5,10
                                l_pub[j].publish(l_pose[j])
                                if(sim_time - starting_time>75):
                                    l_pose[j].pose.position.x, l_pose[j].pose.position.y,l_pose[j].pose.position.z=0,0,10
                                    l_pub[j].publish(l_pose[j])
                                        
            if(j==1):
                #l_pose[j].pose.position.x, l_pose[j].pose.position.y=X[i,j*math.cos(30)],Y[i,j*math.sin(60)],10
                l_pose[j].pose.position.x, l_pose[j].pose.position.y,l_pose[j].pose.position.z =l_pose[0].pose.position.x+3 ,l_pose[0].pose.position.y+4,10
                l_pub[j].publish(l_pose[j])
                              
                
            if(j==2):
                #l_pose[j].pose.position.x, l_pose[j].pose.position.y=X[i,j*math.cos(30)],Y[i,j*math.sin(60)],10
                l_pose[j].pose.position.x, l_pose[j].pose.position.y,l_pose[j].pose.position.z =l_pose[0].pose.position.x+3,l_pose[0].pose.position.y-4,10
                print (l_pose[j].pose.position.x, l_pose[j].pose.position.y, l_pose[j].pose.position.z)
                l_pub[j].publish(l_pose[j])
    
                        
            if(j==3):
                #l_pose[j].pose.position.x, l_pose[j].pose.position.y=X[i,j*math.cos(30)],Y[i,j*math.sin(60)],10
                l_pose[j].pose.position.x, l_pose[j].pose.position.y,l_pose[j].pose.position.z = l_pose[0].pose.position.x+8.04,l_pose[0].pose.position.y+3.47,10
                print (l_pose[j].pose.position.x, l_pose[j].pose.position.y, l_pose[j].pose.position.z)
                l_pub[j].publish(l_pose[j])
            
                        
            if(j==4):
                #l_pose[j].pose.position.x, l_pose[j].pose.position.y=X[i,j*math.cos(30)],Y[i,j*math.sin(60)],10
                l_pose[j].pose.position.x, l_pose[j].pose.position.y,l_pose[j].pose.position.z = l_pose[0].pose.position.x+8.04,l_pose[0].pose.position.y-3.47,10
                print (l_pose[j].pose.position.x, l_pose[j].pose.position.y, l_pose[j].pose.position.z)
                l_pub[j].publish(l_pose[j])
                
                               
            if(sim_time - starting_time>85):
                l_pose[j].pose.position.z=0
                l_pub[j].publish(l_pose[j])


        # if i >= 50:
        #     #ii = i
        #     for j in xrange(6):
        #         # l_pose[j].pose.position.x, l_pose[j].pose.position.y, l_pose[j].pose.position.z = X[0+j*12,ii], X[1+j*12,ii], X[2+j*12,ii]+8
        #         l_pose[j].pose.position.x, l_pose[j].pose.position.y = j, Y[ii-49,j]

        #         print (l_pose[j].pose.position.x, l_pose[j].pose.position.y, l_pose[j].pose.position.z)

        #         l_pub[j].publish(l_pose[j])

        # elif i >= 60:
        #     #ii = i
        #     for j in xrange(6):
        #         l_pose[j].pose.position.x = X[ii-70,j]

        #         print (l_pose[j].pose.position.x, l_pose[j].pose.position.y, l_pose[j].pose.position.z)

        #         l_pub[j].publish(l_pose[j])

        # else: 
        #     for j in xrange(6):
        #         # l_pose[j].pose.position.x, l_pose[j].pose.position.y, l_pose[j].pose.position.z = X[0+j*12,0], X[1+j*12,0], X[2+j*12,0]+(i+float(k))/k*8
        #         l_pose[j].pose.position.x, l_pose[j].pose.position.y, l_pose[j].pose.position.z = j, 0, Z[i,j]

        #         print (l_pose[j].pose.position.x, l_pose[j].pose.position.y, l_pose[j].pose.position.z)

        #         l_pub[j].publish(l_pose[j])
             
        # Pose_quad1.pose.position.x, Pose_quad1.pose.position.y, Pose_quad1.pose.position.z = X[0,i], X[1,i], X[2,i]+8
        # quad1_trajectory_ref_pub.publish(Pose_quad1)
        rate.sleep()
    # mpl.rcParams['legend.fontsize'] = 10
    
    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    # for j in xrange(6):
    #     ax.plot(l_pose[j].pose.position.x, l_pose[j].pose.position.y, l_pose[j].pose.position.z, label='quad'+str(j+1))

    # ax.legend()
    
    # plt.show()


if __name__ == "__main__":
    #formation_change_data = sio.loadmat('/home/asladmin/quad_ru/src/hector_Astar/scripts/Formation_change_data.mat')
    formation_change_data = sio.loadmat("/home/mkutan/Desktop/gazebo_quadcopter/src/hector_Astar/scripts/my001.mat") 
    # x_r,k,n,X = formation_change_data['x_r'],formation_change_data['k'],formation_change_data['n'][0][0],formation_change_data['X']
    X,Y,Z = formation_change_data["X"],formation_change_data["Y"],formation_change_data["Z"]
    


    dt = 0.05
    try:
        quadcopter_trajectory_ref_pub(X,Y,Z)
    except rospy.ROSInterruptException:
        pass


