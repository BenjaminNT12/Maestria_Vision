#! /usr/bin/python
import roslib
import numpy as np
import rospy
import sys
import cv2 as cv
from std_msgs.msg import String
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

class Image_converter():
    global close_paint, background_image, crop_img, frame, contador

    close_paint, drawing = False, False

    background_image = np.zeros((480, 856, 3), dtype = np.uint8)

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback)

    def paint(self, event, x, y, flags, params):
        global xi, xf, yi, yf, drawing, close_paint

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


        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            contador = contador + 0.1
            print(contador)
            print("parte limpia")
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
            print("parte paint")
            try:
                sift_c = True
                print("imagen correcta")
            except:
                print("error en la imagen selecionada")

        if sift_c == True and homography_c == False:
            self.crop_img_gray = cv.cvtColor(crop_img, cv.COLOR_BGR2GRAY)

            homography_c = True

            self.sift = cv.xfeatures2d.SIFT_create()
            self.kp_image, self.desc_image = self.sift.detectAndCompute(self.crop_img_gray, None)

            keyimg = cv.drawKeypoints(self.crop_img_gray, self.kp_image, self.crop_img_gray)
            print("parte sift")


            index_params = dict(algorithm = 0, trees = 5)
            search_params = dict()
            self.flann =  cv.FlannBasedMatcher(index_params, search_params)
            sift_c = False
            homography_c = True

        if homography_c == True:
            grayframe = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            print("parte homography")
            kp_grayframe, desc_grayframe = self.sift.detectAndCompute(grayframe, None)
            # keygrayframe = cv.drawKeypoints(grayframe, kp_grayframe, grayframe)

            matches = self.flann.knnMatch(self.desc_image, desc_grayframe, k = 2)
            good_point = []

            for m, n in matches:
                if m.distance < 0.5*n.distance:
                    good_point.append(m)

            keyimg3 = cv.drawMatches(crop_img, self.kp_image, grayframe, kp_grayframe, good_point, grayframe)

            cv.imshow("keyimg3", keyimg3)

            if len(good_point ) > 10:
                query_pts = np.float32([self.kp_image[m.queryIdx].pt for m in good_point]).reshape(-1, 1, 2 )
                train_pts = np.float32([kp_grayframe[m.trainIdx].pt for m in good_point]).reshape(-1, 1, 2)

                matrix, mask = cv.findHomography(query_pts, train_pts, cv.RANSAC, 5.0)
                matches_mask = mask.ravel().tolist()

                h, w = self.crop_img_gray.shape

                pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
                dst = cv.perspectiveTransform(pts, matrix)

                homography = cv.polylines(frame, [np.int32(dst)], True, (255, 0, 0), 3)

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
