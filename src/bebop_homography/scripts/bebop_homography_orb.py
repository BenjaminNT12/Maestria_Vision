#! /usr/bin/python
import roslib
import numpy as np
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import sys
import cv2 as cv
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

frame = 0
crop_img = 0
background_image = 0
xi, xf = 0, 0
yi, yf = 0, 0
drawing = False
close_paint = False
sift_c = False
homography_c = False
contador = 0.0




FLANN_INDEX_KDTREE = 1
FLANN_INDEX_LSH    = 6
flann_params= dict(algorithm = FLANN_INDEX_LSH,
                   table_number = 6, # 12
                   key_size = 12,     # 20
                   multi_probe_level = 1) #2

class Image_converter():
    global close_paint, background_image, crop_img, frame
    # caffe.set_mode_gpu()
    # caffe.set_device(args.gpu_id)
    close_paint, drawing = False, False

    background_image = np.zeros((480, 856, 3), dtype = np.uint8)

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback)

    def paint(self, event, x, y, flags, params):
        global xi, xf, yi, yf, drawing, close_paint, contador

        if event == cv.EVENT_LBUTTONDOWN:
            drawing = True
            xi, yi = x, y
        elif event == cv.EVENT_MOUSEMOVE and drawing == True:
            if drawing == True:
                background_image[:] = 0
                cv.rectangle(background_image, (xi, yi), (x, y) ,(0, 0, 255), 3)
                xf, yf = x, y
        elif event == cv.EVENT_LBUTTONUP:
            drawing = False
            close_paint = True

    def callback(self, data):
        global sift_c, homography_c, crop_img, homography_start, contador

        def map(valor, in_min, in_max, out_min, out_max):
            return (valor - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


        twist_virtual_camera = Twist()

        pub = rospy.Publisher("bebop/camera_control", Twist, queue_size = 1)

        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # cv.imshow("frame",frame)
            # print("parte limpia")
        except CvBridgeError, e:
            print e

        cv.namedWindow("homography")
        cv.setMouseCallback("homography", self.paint)
        frame_crop = cv.addWeighted(frame, 1, background_image, 1, 0)
        if close_paint == False:
            cv.imshow("homography", frame_crop)
        else:
            cv.imshow("homography", frame)

        if close_paint == True and xi!= xf and yi != yf and sift_c == False:
            crop_img = frame[yi:yf, xi:xf]
            print(yi, yf, xi, xf)
            # print("parte paint")
            try:
                sift_c = True
                print("imagen correcta")
            except:
                print("error en la imagen selecionada")

        if sift_c == True and homography_c == False:
            self.crop_img_gray = cv.cvtColor(crop_img, cv.COLOR_BGR2GRAY)
            # cv.imshow("crop_img_gray", self.crop_img_gray )
            homography_c = True

            self.orb = cv.ORB_create(nfeatures = 1000)
            self.kp_image, self.desc_image = self.orb.detectAndCompute(self.crop_img_gray, None)

            # keyimg = cv.drawKeypoints(self.crop_img_gray, self.kp_image, self.crop_img_gray)
            # print("parte sift")
            # print("keyimg", keyimg)

            # index_params = dict(algorithm = 0, trees = 5)
            # search_params = dict()
            self.flann =  cv.FlannBasedMatcher(flann_params, {})
            sift_c = False
            homography_c = True

        if homography_c == True:
            contador = contador + 0.1

            grayframe  = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            # print("parte homography")
            kp_grayframe, desc_grayframe = self.orb.detectAndCompute(grayframe, None)
            # keygrayframe = cv.drawKeypoints(grayframe, kp_grayframe, grayframe)

            matches = self.flann.knnMatch(self.desc_image, desc_grayframe, k = 2)
            good_point = []
            try:
                for m, n in matches:
                    if m.distance < 0.7*n.distance:
                        good_point.append(m)
            except:
                print("no matches")
            keyimg3 = cv.drawMatches(self.crop_img_gray , self.kp_image, grayframe, kp_grayframe, good_point, grayframe)

            cv.imshow("keyimg3", keyimg3)
            # kinect_intrinsic_param = np.array([[514.04093664, 0., 320], [0., 514.87476583, 240], [0., 0., 1.]])
            # kinect_distortion_param = np.array([2.68661165e-01, -1.31720458e+00, -3.22098653e-03, -1.11578383e-03, 2.44470018e+00])
            if len(good_point ) > 10:
                query_pts = np.float32([self.kp_image[m.queryIdx].pt for m in good_point]).reshape(-1, 1, 2 )
                train_pts = np.float32([kp_grayframe[m.trainIdx].pt for m in good_point]).reshape(-1, 1, 2)

                matrix, mask = cv.findHomography(query_pts, train_pts, cv.RANSAC, 5.0)
                matches_mask = mask.ravel().tolist()

                h, w = self.crop_img_gray.shape

                pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
                # pts_3d = np.float32([ [-w/2,-h/2,0],[-w/2,(h-1)/2,0],[(w-1)/2,(h-1)/2,0],[(w-1)/2,-h/2,0] ])
                # center_pts = np.float32([w/2, h/2]).reshape(-1, 1, 2)
                # cntr = cv.perspectiveTransform(center_pts, matrix)
                dst = cv.perspectiveTransform(pts, matrix)

                p4x = np.int32(dst[3, 0, 0]) ; p4y = np.int32(dst[3, 0, 1])
                p3x = np.int32(dst[2, 0, 0]) ; p3y = np.int32(dst[2, 0, 1])
                p2x = np.int32(dst[1, 0, 0]) ; p2y = np.int32(dst[1, 0, 1])
                p1x = np.int32(dst[0, 0, 0]) ; p1y = np.int32(dst[0, 0, 1])

                c_disp_x = np.int32(856/2)
                c_disp_y = np.int32(480/2)

                center_disp = (c_disp_x, c_disp_y)

                p4 = (p4x, p4y)
                p3 = (p3x, p3y)
                p2 = (p2x, p2y)
                p1 = (p1x, p1y)

                c_track_x = p1x + (p3x - p1x)/2
                c_track_y = p1y + (p3y - p1y)/2

                center_track = (c_track_x , c_track_y)



                # retval, rotation, translation = cv.solvePnP(pts_3d, dst, kinect_intrinsic_param, kinect_distortion_param)
                # convert to centimeters
                # translation = (40./53.) * translation *.1
                # rotation = rotation * 180./np.pi
                # print(rotation, translation)
                # font = cv.FONT_HERSHEY_SIMPLEX

                # cv.putText(frame,'position(cm)',(10,30), font, 0.7,(0,255,0),1,cv.LINE_AA)
                # cv.putText(frame,'x:'+str(round(translation[0],2)),(250,30), font, 0.7,(0,0,255),2,cv.LINE_AA)
                # cv.putText(frame,'y:'+str(round(translation[1],2)),(350,30), font, 0.7,(0,0,255),2,cv.LINE_AA)
                # cv.putText(frame,'z:'+str(round(translation[2],2)),(450,30), font, 0.7,(0,0,255),2,cv.LINE_AA)
                #
                # cv.putText(frame,'orientation(degree)',(10,60), font, 0.7,(0,255,0),1,cv.LINE_AA)
                # cv.putText(frame,'x:'+str(round(rotation[0],2)),(250,60), font, 0.7,(0,0,255),2,cv.LINE_AA)
                # cv.putText(frame,'y:'+str(round(rotation[1],2)),(350,60), font, 0.7,(0,0,255),2,cv.LINE_AA)
                # cv.putText(frame,'z:'+str(round(rotation[2],2)),(450,60), font, 0.7,(0,0,255),2,cv.LINE_AA)
                # print(contador)


                # print(np.int32(dst))
                # print(type(dst))

                homography = cv.polylines(frame, [np.int32(dst)], True, (255, 0, 0), 3)
                homography = cv.circle(homography, center_track, 5, (255, 255, 255), -1)
                homography = cv.circle(homography, center_disp, 5, (0, 0, 255), -1)
                homography = cv.line(homography, center_disp, center_track, (255, 0, 0), 2)
                error_camera_axis_x = c_disp_x - c_track_x
                error_camera_axis_y = c_disp_y - c_track_y
                kp = 2
                u_control_z = 4 * error_camera_axis_x
                u_control_y = 4 * error_camera_axis_y
                # print(camera_axis_x, camera_axis_y)
                # center_circle = cv.norm()
                # homography = cv.Circle(homography, )

                twist_virtual_camera.linear.x = 0
                twist_virtual_camera.linear.y = 0
                twist_virtual_camera.linear.z = 0

                twist_virtual_camera.angular.x = 0
                twist_virtual_camera.angular.y = 0# np.int32(map(u_control_y, -240, 240, -80, 15))
                twist_virtual_camera.angular.z = np.int32(map(u_control_z, -428, 428, 35, -35))

                pub.publish(twist_virtual_camera)
                print("error en z: ", error_camera_axis_x, "error en y: ", error_camera_axis_y, "control en z: ", np.int32(map(u_control_z, -428, 428, 35, -35)), "control en y: ",np.int32(map(u_control_y,-240, 240, -80, 15)))

                cv.imshow("homography", homography)
            else:
                cv.imshow("homography", frame)

        self.key = cv.waitKey(1) & 0xFF
        if self.key == 27:
            rospy.signal_shutdown("user hit esc key to quit")

def main(arg):

    Image_converter()
    rospy.init_node("Image_converter", anonymous = True)

    try:
        rospy.spin()
    except keyboardInterrupt:
        print("shutting down vision node")

    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
