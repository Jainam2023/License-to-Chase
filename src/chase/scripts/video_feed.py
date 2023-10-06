import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError 
import sys

bridge=CvBridge()

def image_callback(ros_image):
    global bridge
    cv_image=bridge.imgmsg_to_cv2(ros_image,"bgr8")
    cv2.imshow("video_feed",cv_image)
    cv2.waitKey(3)

def main(args):
    rospy.init_node("video_feed",anonymous=True)
    image_sub=rospy.Subscriber("/cam/camera3/ovh/image_raw",Image,image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)
