#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, tty

msg = """
Maestria en automatizacion y control

 
Aterrizaje de un mini-helicoptero de 4 rotores
sobre una plataforma movil utilzando vision
artificial.

Take off    1
Land        2
"""

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .5
turn = 1

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)
    pub2 = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
    pub3 = rospy.Publisher('bebop/land', Empty, queue_size = 1)
    pub4 = rospy.Publisher('bebop/camera_control', Twist, queue_size = 1)
    empty_msg = Empty()
    rospy.init_node('teleop_twist_keyboard')

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    VirtualCameraAngX = 0
    VirtualCameraAngY = 0
    VirtualCameraAngZ = 0
    VirtualCameraLinX = 0
    VirtualCameraLinY = 0
    VirtualCameraLinZ = 0

    try:
        print msg
        print vels(speed,turn)
        while(1):
            key = getKey()
            print 'Key:'
            print key
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                
                print vels(speed,turn)
                if (status == 14):
                    print msg
                    status = (status + 1) % 15
            elif key ==  '1':
                print 'pressed key 1'
                pub2.publish(empty_msg)
            elif key ==  '2':
                print 'pressed key 2'
                pub3.publish(empty_msg)
            elif key ==  'Q':
                print 'Up Camera'                     
                if VirtualCameraAngY < 15:##
                    VirtualCameraAngY += 1 
                print VirtualCameraAngY
            elif key ==  'A':
                print 'Down Camera'                     
                if VirtualCameraAngY > -80:##
                    VirtualCameraAngY -= 1
                print VirtualCameraAngY
            elif key ==  'X':
                print 'Left Camera'
                if VirtualCameraAngZ < 35:##
                    VirtualCameraAngZ += 1
                print VirtualCameraAngZ
            elif key ==  'Z':
                print 'Right Camera'
                if VirtualCameraAngZ > -35:##
                    VirtualCameraAngZ -= 1    
                print VirtualCameraAngZ                    
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            TwistVirtualCamera = Twist()        ## declaramos un tipo de datos Twist()
            TwistVirtualCamera.linear.x = 0;    TwistVirtualCamera.linear.y = 0;     TwistVirtualCamera.linear.z = 0     
            TwistVirtualCamera.angular.x = 0;    TwistVirtualCamera.angular.y = VirtualCameraAngY;    TwistVirtualCamera.angular.z = VirtualCameraAngZ
            pub4.publish(TwistVirtualCamera)


            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)

    except:
        print e
    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

