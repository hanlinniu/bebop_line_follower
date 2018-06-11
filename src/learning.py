#!/usr/bin/python

'''

__Author_ = Lowyi
__Email__ = MR.LowBattery@gmail.com
__Team__  = MRL_UAV

'''

import roslib

roslib.load_manifest('bebop_line_follower')
import sys
import rospy
import numpy as np
import cv2
import time
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from math import *

image_exist = 0


class image_receiver:
    ''' Class used in the modes in which camera is required '''

    def __init__(self, camera=1):
        self.best_x = 0
        self.camera = camera
        self.bridge = CvBridge()
        self.subscribed = 0
        # self.lower = np.array([0, 141, 214], dtype="uint8")
        # self.upper = np.array([15, 255, 255], dtype="uint8")
        self.lower = np.array([0, 224, 0], dtype=np.uint8)
        self.upper = np.array([103, 255, 255], dtype=np.uint8)
        # self.lower = np.array([99, 48, 72], dtype=np.uint8)
        # self.upper = np.array([255, 255, 255], dtype=np.uint8)
        self.contours = []
        self.kernelOpen = np.ones((5, 5))
        self.kernelClose = np.ones((20, 20))
        self.image_pos_pub = rospy.Publisher("data", Quaternion, queue_size=10)  # change topic nmae

    def follow_line(self, camera_image):
        # sends for the controller the distance from the line and the angle between the drone and the line
        global image_exist

        height, width = camera_image.shape[:2]
        a = height / 2
        b = width / 2

        # crop_img = camera_image[a-200:a+200, b-300:b+300]
        crop_img = camera_image[0:300, 0:width]
        img2 = cv2.GaussianBlur(crop_img, (15,15),2)

        hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        one_color_image = cv2.bitwise_and(crop_img, crop_img, mask=mask)

        best_x, best_y = width / 2, height + 1
        worst_x, worst_y = width / 2, -1
        detecting = 0
        for i in contours:
            detecting = 1
            (x, y), radius = cv2.minEnclosingCircle(i)
            # M = cv2.moments(i)
            # if M["m00"] != 0:
            #     cx = int(M["m10"] / M["m00"])
            #     cy = int(M["m01"] / M["m00"])
            # else:
            #     cx, cy = 0, 0
            if y < best_y:
                best_y = y
                best_x = x
            if y > worst_y:
                worst_y = y
                worst_x = x
        if best_y != worst_y:
            ang = float(-((best_x - worst_x) / (best_y - worst_y)))

        else:
            ang = 0
        self.worst_x = (worst_x - width / 2)
        cv2.circle(one_color_image, (int(best_x), int(best_y)), 5, (0, 255, 0), -1)
        cv2.circle(one_color_image, (int(worst_x), int(worst_y)), 5, (0, 255, 0), -1)
        self.image_pos_pub.publish(self.worst_x, detecting, atan(ang), self.camera)
        one_color_image = cv2.resize(one_color_image, (0, 0), fx=0.7, fy=0.7)
        crop_img = cv2.resize(crop_img, (0, 0), fx=0.7, fy=0.7)
        cv2.imshow("Image window", np.hstack([one_color_image, crop_img]))
        image_exist = 1
        cv2.waitKey(3)

    def callback(self, data):

        start = time.time()

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e);

        if self.camera ==1:
            self.follow_line(cv_image)

    def __del__(self):
        pass


ic = None
ic = image_receiver(1)


def callback(data):
    ic.callback(data)


def main(args):
    rospy.init_node('image_receiver', anonymous=True)
    rospy.Subscriber("/bebop/image_raw", Image, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
