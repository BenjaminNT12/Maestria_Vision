#!/usr/bin/env python
import rospy
import message_filters
from message_filters import TimeSynchronizer, Subscriber
from bebop_msgs.msg import Ardrone3CameraStateOrientation, CommonCommonStateBatteryStateChanged

def callback(test1, test2):
    print("test1")
    # print(test2)

camera_control = message_filters.Subscriber('/bebop/states/ardrone3/CameraState/Orientation', Ardrone3CameraStateOrientation)
battery = message_filters.Subscriber('/bebop/states/common/CommonState/BatteryStateChanged', CommonCommonStateBatteryStateChanged)

rospy.init_node('listener', anonymous = True)

tss = message_filters.TimeSynchronizer([camera_control, battery], 10)
tss.registerCallback(callback)


rospy.spin()
