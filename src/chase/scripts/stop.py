#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rospy
from nav_msgs.msg import Odometry
import math

xi=0
yi=0
xg=7
yg=7
dist=10

def pose_callbackg(pose:Odometry):
    global xg,yg
    x=pose.pose.pose.position.x
    y=pose.pose.pose.position.y
    xg=x
    yg=y

def pose_callbacki(pose:Odometry):
    global xi,yi
    x=pose.pose.pose.position.x
    y=pose.pose.pose.position.y
    xi=x
    yi=y

def move(pub):
    global xi,yi,xg,yg,dist
    msg=Twist()
    msg1=Twist()
    while(True):
        dist=math.dist([xg,yg],[xi,yi])
        if(dist<1.5) :
            while(True):
                dist=math.dist([xg,yg],[xi,yi])
                msg.linear.x=0
                msg.angular.z=0
                pub.publish(msg)
                if(dist<1):
                    while(True):
                        msg1.linear.x=0
                        msg1.angular.z=0
                        msg.linear.x=0
                        msg.angular.z=0
                        pub.publish(msg)
                        pub.publish(msg1)


if __name__=='__main__':
    rospy.init_node("straight")
    pub=rospy.Publisher("/tbot/base_controller/cmd_vel",Twist,queue_size=10)
    pub1=rospy.Publisher("/pbot/base_controller/cmd_vel",Twist,queue_size=10)
    subg=rospy.Subscriber("/tbot/base_controller/odom",Odometry,callback=pose_callbackg)
    subi=rospy.Subscriber("/pbot/base_controller/odom",Odometry,callback=pose_callbacki)
    move(pub)
