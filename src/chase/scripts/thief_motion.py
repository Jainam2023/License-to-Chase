#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rospy
import time


def move(pub, vel):
    tdiff=20.0/vel
    start=time.time()
    end=time.time()
    msg=Twist()

    while (end-start)<tdiff:
        end=time.time()
        msg.linear.x=vel
        msg.angular.z=0
        pub.publish(msg)
        rate.sleep()

    while not rospy.is_shutdown():
        msg.linear.x=0
        msg.angular.z=0
        pub.publish(msg)
        rate.sleep()

if __name__=='__main__':
    
    rospy.init_node("straight")
    rate=rospy.Rate(10)
    pub=rospy.Publisher("/tbot/base_controller/cmd_vel",Twist,queue_size=10)
    vel=0.6
    move(pub, vel)