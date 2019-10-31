#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from human_detect.msg import uniMSG


class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("/thresholded_Image",Image,queue_size = 10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/radio_cam/rgb/image_raw",Image,self.callback)
        self.first_frame = None
        self.threshold = 50
        self.separated = rospy.get_param("~separated")



    def checkChange(self, frame):
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.first_frame is None:
            self.first_frame = frame
        self.frameDelta = cv2.absdiff(self.first_frame, frame)
        self.thres = cv2.threshold(self.frameDelta, self.threshold, 255, cv2.THRESH_BINARY)[1]
        count = np.count_nonzero(self.thres) 
        if count > 0:
            return True
        else:
            return False
    def convert_img(self, original_img, threshold_img):
        grey_img = cv2.cvtColor(original_img, cv2.COLOR_BGR2GRAY)
        pseudo_color_img = cv2.cvtColor(grey_img, cv2.COLOR_GRAY2BGR)
        bool_thres = [threshold_img > self.threshold]
        new_img = np.where(threshold_img > self.threshold, original_img, pseudo_color_img)

        return new_img


    def callback(self, data):
        if self.separated:
            try:
              frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
              print(e)

            flag =  self.checkChange(frame)
            if flag:
                new_img = self.convert_img(frame, self.thres)
                cv2.waitKey(3)
                try:
                  self.image_pub.publish(self.bridge.cv2_to_imgmsg(new_img, "bgr8"))
                except CvBridgeError as e:
                  print(e)
        else:
            try:
              frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
              print(e)

            flag =  self.checkChange(frame)
            if flag:
                new_img = self.convert_img(frame, self.thres)
                cv2.waitKey(3)

                msg = uniMSG()
                try:
                    msg.pixel_values = self.bridge.cv2_to_imgmsg(new_img, "bgr8").data
                except CvBridgeError as e:
                  print(e)

                # publish the created message
                pub = rospy.Publisher('/uni_message', uniMSG, queue_size = 10)
                pub.publish(msg)


def main(args):
    rospy.init_node('cameraListener', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)