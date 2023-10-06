#!/usr/bin/env python3
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import math
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from my_robot_controller.msg import ps
# from my_robot_controller.msg import pos

class goal:
    def __init__(self,x_r,y_r,x,y,yaw):
        self.x_r=x_r
        self.y_r=y_r
        self.x=x
        self.y=y
        self.yaw=yaw
        self.sub1=rospy.Subscriber("/pbot/base_controller/odom",Odometry,callback=self.pose_callback)
        self.sub2=rospy.Subscriber("/path_data",ps,callback=self.msg_callback)
        # self.sub=rospy.Subscriber("/bot_pose",pos,callback=self.pose)
    def pose_callback(self,pose:Odometry):
        orientation_q=pose.pose.pose.orientation
        orientation_list=[orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
        (roll,pitch,self.yaw)=euler_from_quaternion(orientation_list)
        self.x=pose.pose.pose.position.x
        self.y=pose.pose.pose.position.y

    def msg_callback(self,msg):
        self.x_r=msg.x
        self.y_r=msg.y 

    # def pose(self,pose:pos):
    #     self.x=pose.xp
    #     self.y=pose.yp
    #     self.yaw=pose.yawp
    
def move(pub,var):
    
    while not rospy.is_shutdown():
        x=var.x
        y=var.y
        yaw=var.yaw
        x_0=var.x_r
        y_0=var.y_r
        dx = x-x_0
        dy = y-y_0
        dist = math.sqrt(dx*dx + dy*dy)
        a = math.atan2(dy, dx)

        msg = Twist()
        msg.linear.x =  dist
        if msg.linear.x > 3 :
            msg.linear.x = 3
        

        err_angle = a - yaw -math.pi/2
        # if err_angle > math.pi:
        #     err_angle -= 2*math.pi
        # elif err_angle < -math.pi:
        #     err_angle += 2*math.pi
        if err_angle>0:
            err_angle=math.pi-err_angle
        if err_angle<0:
            err_angle=-math.pi-err_angle
        ang=8*err_angle
        msg.angular.z = ang

        print(err_angle)
        pub.publish(msg)



        rate.sleep()

    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)

        
var=goal(0,0,0,0,0)

if __name__== "__main__":
    rospy.init_node("follower")
    rate=rospy.Rate(10)
    pub=rospy.Publisher("/pbot/base_controller/cmd_vel",Twist,queue_size=10)
    move(pub,var)