#!/usr/bin/env python3
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import time
import numpy as np

w=35
img=[]
bridge=CvBridge()
class Images:
    global bridge
    def __init__(self,im):
         self.im=im
    
    def image_callback(self,ros_image):
         self.im=bridge.imgmsg_to_cv2(ros_image,"bgr8")


x=Images(img)
def main():
        global x
        time.sleep(0.05)
        rospy.init_node("Image",anonymous=True)
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            image_sub=rospy.Subscriber("/cam/camera3/ovh/image_raw",Image,x.image_callback)
            print("Hi")
            rate.sleep()
            break
        cv.destroyAllWindows()
    
if __name__== "__main__":
    main()
    image=x.im
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    blur = cv.GaussianBlur(gray, (5, 5),
                        cv.BORDER_DEFAULT)
    ret, thresh = cv.threshold(blur, 120, 250,
                            cv.THRESH_BINARY_INV)


    contours, hierarchies = cv.findContours(
        thresh, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

    blank = np.zeros(thresh.shape[:2],
                    dtype='uint8')

    cv.drawContours(blank, contours, -1,
                    (255, 0, 0), 1)

    
    for i in contours:
        M = cv.moments(i)
        if M['m00'] != 0 :

            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv.drawContours(image, [i], -1, (0, 255, 0), 2)
            cv.rectangle(image, (cx-w, cy+w), (cx+w, cy-w), (255,255,255), -1)
            print(f"x:{cx} y:{cy}")
                
    _ , image = cv.threshold(image, 125, 255, cv.THRESH_BINARY_INV)

    cv.imwrite("/home/jainam/catkin_ws/src/test3/scripts/images/img_f1.jpg",image)
    time.sleep(1)
    
    
