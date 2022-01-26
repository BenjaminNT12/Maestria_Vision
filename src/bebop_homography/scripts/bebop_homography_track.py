#! /usr/bin/python
import roslib
import numpy as np
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import sys
import cv2 as cv
import math
import sys, select, termios, tty
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

from mpl_toolkits.mplot3d import Axes3D

FLANN_INDEX_KDTREE = 1
FLANN_INDEX_LSH    = 6
flann_params= dict(algorithm = FLANN_INDEX_LSH,
                   table_number = 6, # 12
                   key_size = 12,     # 20
                   multi_probe_level = 1) #2


error_track_y = 0.0
error_track_z = 0.0
background_image = np.zeros((480, 856, 3), dtype = np.uint8)
xi, xf, yi, yf, = 0 , 0 , 0 ,0
close_paint = False
frame = np.array([])
image_track = np.array([])
image_to_track = np.array([])
grayimage_track = np.array([])
desc_image, kp_image = 0, 0

twist_virtual_camera = Twist()
twist_cmd_vel = Twist()
pub = rospy.Publisher("bebop/camera_control", Twist, queue_size = 1)
pub_track = rospy.Publisher("bebop/cmd_vel", Twist,  queue_size = 1)

orb = cv.ORB_create(nfeatures = 500)
flann = cv.FlannBasedMatcher(flann_params, {})

xs_y = []
ys_y = []

xs_z = []
ys_z = []


def map(valor, in_min, in_max, out_min, out_max):
    return (valor - in_min) * (out_max - out_min)/ (in_max - in_min) + out_min


def paint(event, x, y, flags, params):
    global xi, xf, yi, yf, drawing, close_paint, background_image

    if not hasattr(paint, "drawing"):
        paint.drawing = False

    if event == cv.EVENT_LBUTTONDOWN:
        paint.drawing = True
        xi, yi  = x, y
    elif event == cv.EVENT_MOUSEMOVE and paint.drawing == True:
        background_image[:] = 0
        cv.rectangle(background_image, (xi, yi), (x, y), (0, 0, 255), 3 )
        xf, yf = x, y
    elif event == cv.EVENT_LBUTTONUP:
        paint.drawing = False
        close_paint = True


def select_image_to_track():
    global close_paint, background_image, frame, image_track, image_to_track

    if not hasattr(select_image_to_track, "flag_paint"):
        select_image_to_track.flag_paint = False

    cv.namedWindow("homography")
    cv.setMouseCallback("homography", paint)

    if close_paint == False and select_image_to_track.flag_paint == False:
        image_track = cv.addWeighted(frame, 1, background_image, 1, 0)
        cv.imshow("homography", image_track)
        select_image_to_track.flag_paint = False
    elif close_paint == True and xi != xf and yi != yf and select_image_to_track.flag_paint == False:
        image_to_track = frame[yi:yf, xi:xf]
        # cv.imshow("image_track", image_track)
        select_image_to_track.flag_paint = True
    return select_image_to_track.flag_paint


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def interpolacion(val, in1, in2, y1, y2):
    return(((val - in1) / (in2 - in1)) * (y2 - y1) + y1)

def homography(active):
    global frame, desc_image, grayimage_track, kp_image, orb, flann, image_to_track, flag_orb, error_track_y, error_track_z

    c_disp_z = np.int32(856/2)
    c_disp_y = np.int32(480/2)

    center_disp = (c_disp_z, c_disp_y)

    if not hasattr(homography, "delta_center_y"):
        homography.delta_center_y = 0

    if not hasattr(homography, "delta_center_z"):
        homography.delta_center_z = 0

    if not hasattr(homography, "flag_orb"):
        homography.flag_orb = False

    if not hasattr(homography, "counter"):
        homography.counter = 0

    if active == True and homography.flag_orb == False:
        grayimage_track = cv.cvtColor(image_to_track, cv.COLOR_BGR2GRAY)
        # cv.imshow("grayimage_track", grayimage_track)
        kp_image, desc_image = orb.detectAndCompute(grayimage_track, None)
        key_image = cv.drawKeypoints(grayimage_track, kp_image, grayimage_track)
        homography.flag_orb = True

    if  homography.flag_orb == True:

        grayframe = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        kp_grayimage, desc_grayimage = orb.detectAndCompute(grayframe, None)

        keygrayframe = cv.drawKeypoints(grayframe, kp_grayimage, grayframe)

        matches = flann.knnMatch(desc_image, desc_grayimage, k = 2 )

        good_point = []

        try:
            for m, n in matches:
                if m.distance < 0.7 * n.distance:
                    good_point.append(m)

            # keyimg3 = cv.drawMatches(grayimage_track, kp_image, grayframe, kp_grayimage, good_point, grayframe)
            # cv.imshow("keyimg3", keyimg3)
        except:
            print("no matches")

        if len(good_point) > 10:
            query_pts = np.float32([kp_image[m.queryIdx].pt for m in good_point]).reshape(-1, 1, 2)
            train_pts = np.float32([kp_grayimage[m.trainIdx].pt for m in good_point]).reshape(-1, 1, 2)

            matrix, mask = cv.findHomography(query_pts, train_pts, cv.RANSAC, 5.0)
            matches_mask = mask.ravel().tolist()

            h, w = grayimage_track.shape
            pts = np.float32([[0,0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
            dst = cv.perspectiveTransform(pts, matrix)

            p4z = np.int32(dst[3, 0, 0]) ; p4y = np.int32(dst[3, 0, 1])
            p3z = np.int32(dst[2, 0, 0]) ; p3y = np.int32(dst[2, 0, 1])
            p2z = np.int32(dst[1, 0, 0]) ; p2y = np.int32(dst[1, 0, 1])
            p1z = np.int32(dst[0, 0, 0]) ; p1y = np.int32(dst[0, 0, 1])

            p4 = (p4z, p4y)
            p3 = (p3z, p3y)
            p2 = (p2z, p2y)
            p1 = (p1z, p1y)

            c_track_z = p1z + (p3z - p1z)/2
            c_track_y = p1y + (p3y - p1y)/2

            center_track = (c_track_z , c_track_y)

            homography_show = cv.polylines(frame, [np.int32(dst)], True, (255, 0, 0), 3 )

            homography_show = cv.circle(homography_show, center_track, 5, (255, 255, 255), -1)
            homography_show = cv.circle(homography_show, center_disp, 5, (0, 0, 255), -1)
            homography_show = cv.line(homography_show, center_disp, center_track, (255, 0, 0), 2)

            error_track_z = c_track_z - c_disp_z
            error_track_y = c_track_y - c_disp_y
            error_track_z = interpolacion(error_track_z,-240.0,240.0,-1.0,1.0)
            error_track_y = interpolacion(error_track_y,-428.0,428.0,-1.0,1.0)

            if homography.delta_center_y > -80 and homography.delta_center_y < 15:
                homography.delta_center_y = homography.delta_center_y + (-0.9*error_track_y)
                twist_virtual_camera.angular.y = homography.delta_center_y
            if homography.delta_center_z > -35 and homography.delta_center_z < 35:
                homography.delta_center_z = homography.delta_center_z + (0.9*error_track_z)
                twist_virtual_camera.angular.z = homography.delta_center_z

            if homography.delta_center_z <= -35: homography.delta_center_z = -33;
            if homography.delta_center_z >= 35: homography.delta_center_z = 34;

            if homography.delta_center_y <= -80: homography.delta_center_y = -79;
            if homography.delta_center_y >= 15: homography.delta_center_y = 14;

            pub.publish(twist_virtual_camera)

            print "e_z = ", error_track_z,"\t", twist_virtual_camera.angular.z, "\t" ,homography.delta_center_z,"\t" ,"e_y = ", error_track_y, "\t", twist_virtual_camera.angular.y,"\t", homography.delta_center_y
            # print("error en z: ", error_camera_axis_x, "error en y: ", error_camera_axis_y, ))
            cv.imshow("homography", homography_show)
        else:
            cv.imshow("homography", frame)
        homography.counter = homography.counter + 1
        xs_y.append(homography.counter)
        ys_y.append(error_track_y)
        xs_z.append(homography.counter)
        ys_z.append(error_track_z)

def callback(data):
    global frame
    bridge = CvBridge()

    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        ret = select_image_to_track()
        homography(ret)
    except CvBridgeError, e:
        print e

    key = cv.waitKey(1) & 0xFF

    if key == 27:
        rospy.signal_shutdown("Saliendo del nodo")


def converter_ros_to_python():

    rospy.init_node("get_image", anonymous = True )
    rospy.Subscriber("/bebop/image_raw", Image, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("apagando get_image")

if __name__ == '__main__':

    settings = termios.tcgetattr(sys.stdin)
    converter_ros_to_python()
    plt.plot(xs_y, ys_y, label = "eje y")
    plt.plot(xs_z, ys_z, label = "eje z")

    # naming the x axis
    plt.xlabel('x - axis')
    # naming the y axis
    plt.ylabel('y - axis')

    # giving a title to my graph
    plt.title('eje z')

    # function to show the plot
    plt.legend()
    plt.show()

    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    #
    # ax.plot_trisurf(ys_z, ys_y, xs_y, linewidth=0.2, antialiased=True)
    #
    # plt.show()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    cv.DestroyAllWindows()
