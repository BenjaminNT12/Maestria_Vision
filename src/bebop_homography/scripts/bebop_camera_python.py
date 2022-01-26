#! /usr/bin/python

import roslib
import rospy
import sys
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError


class Image_converter():
    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print

        cv.imshow("bebop_image", frame)

        self.key = cv.waitKey(1)
        if self.key == 27:
            rospy.signal_shutdown("User hit esc key to quit")

def main(args):
    Image_converter()
    rospy.init_node("image_converter", anonymous = True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print(" shutting down vision node ")
    cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
