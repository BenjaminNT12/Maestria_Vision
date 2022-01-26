#! /usr/bin/python


# import roslib
import rospy
import cv2 as cv
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        cv.imshow("bebop_image", frame)
    except CvBridgeError, e:
        print e


    key = cv.waitKey(1)
    if key == 27:
        rospy.signal_shutdown("desactivar")

def main():

    image = rospy.Subscriber("/bebop/image_raw", Image, callback)
    rospy.init_node("image_get", anonymous = True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("apagando")
    cd.destroyAllWindows()

if __name__ == '__main__':
    main()
