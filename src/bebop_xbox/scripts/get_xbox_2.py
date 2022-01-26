#!/usr/bin/env python
import struct
import sys
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

empty_msg = Empty()
twist_msg_cam = Twist()
twist_msg_vel = Twist()
twist_msg_cam.angular.y = 0.0
twist_msg_cam.angular.z = 0.0

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

rospy.init_node('bebop')

pub  = rospy.Publisher('bebop/land', Empty, queue_size = 1)
pub2  = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
pub3  = rospy.Publisher('bebop/camera_control', Twist, queue_size = 1)
pub4  = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)

button_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
axis_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

buttons = ('A_BUTTON', 'B_BUTTON', 'X_BUTTON', 'Y_BUTTON', '/usLB_BUTTON', 'RB_BUTTON', 'OPTIONS_BUTTON', 'START_BUTTON', 'XBOX_BUTTON', 'JL_BUTTON', 'JR_BUTTON')
axis = ('AXIS_0' , 'AXIS_1', 'AXIS_2', 'AXIS_3', 'AXIS_4', 'AXIS_5', 'AXIS_6', 'AXIS_7')

def mapeo(valor, in_min, in_max, out_min, out_max):
    return (valor - in_min) * (out_max - out_min)/ (in_max - in_min) + out_min

infile_path = "/dev/input/js0"
EVENT_SIZE = struct.calcsize("Lhbb")
file = open(infile_path, "rb")
event = file.read(EVENT_SIZE)
print EVENT_SIZE
while True:
    # pint struct.unpack("Lhbb", event)
    (time, value, type, number) = struct.unpack("Lhbb", event)
    event = file.read(EVENT_SIZE)
    if type == (JS_EVENT_BUTTON ):
        button_data[number] = value
        if button_data[0] == 1:
            pub2.publish(empty_msg)
        if button_data[1] == 1:
            pub.publish(empty_msg)
    if type == (JS_EVENT_AXIS ):
        # axis_data[number] = mapeo(value,1,-1)
        if number == 0:
            twist_msg_cam.angular.z = mapeo(value, -32767.0, 32767.0, -35.0, 35.0)
        if number == 1:
            twist_msg_cam.angular.y = mapeo(value, -32767.0, 32767.0, 15.0, -83.0)
        print twist_msg_cam.angular.y , twist_msg_cam.angular.z

    pub3.publish(twist_msg_cam)
    # print axis_data, button_data
