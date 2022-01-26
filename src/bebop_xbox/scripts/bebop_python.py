#! /usr/bin/python

import roslib
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        cv.imshow("bebop_image", frame)
    except CvBridgeError, e:
        print e

    tecla = cv.waitKey(1000/30)
    if tecla == 27:
        rospy.signal_shutdown("adios")

def main():
    image = rospy.Subscriber("/bebop/image_raw", Image, callback)
    rospy.init_node("bebop_python", anonymous = True)
    try:
        rospy.spin()
    except KeyInterrupt:
        print ("terminamos")
        cv.destroyAllWindows()

if __name__ == '__main__':
    main()
