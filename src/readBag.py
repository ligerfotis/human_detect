#!/usr/bin/env python
import rosbag
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from human_detect.msg import uniMSG

class bag_manager:

    def __init__(self):
        self.bridge = CvBridge()
        self.first_frame = None
        self.threshold = 50

    def read_bag(self, bag_file, msg_count = None):
        # Open rosbag.
        bag = rosbag.Bag(bag_file, "r")
        messages = bag.read_messages(topics=["/radio_cam/rgb/image_raw"])
        camera = bag.get_message_count(topic_filters=["/radio_cam/rgb/image_raw"])
        rate = rospy.Rate(50)


        # Publish velodyne packets from bag to topic.
        pub = rospy.Publisher('/newBag', Image, queue_size=10)

        if msg_count is None:
            msg_count = camera
        #else:
        #    msg_count = min(msg_count, n_lidar)
        # print('msg_count: ', msg_count)

        published_count = 0
        for i in range(msg_count):
            topic, msg, t = messages.next()
            try:
                  frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                print(e)
            #cv2.imshow("topic",frame)
            #cv2.waitKey(3)
            new_img = self.convert_img(frame)
            try:
                pub.publish(self.bridge.cv2_to_imgmsg(new_img, "bgr8"))
            except CvBridgeError as e:
                print(e)
            published_count += 1
            rate.sleep()

        print('published_count: {0}'.format(published_count))

        bag.close() 

    def convert_img(self, frame):
        if self.first_frame is None:
            self.first_frame = frame
        frameDelta = cv2.absdiff(self.first_frame, frame)
        thres = cv2.threshold(frameDelta, self.threshold, 255, cv2.THRESH_BINARY)[1]
        
        grey_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pseudo_color_img = cv2.cvtColor(grey_img, cv2.COLOR_GRAY2BGR)
        new_img = np.where(thres > self.threshold, frame, pseudo_color_img)

        return new_img

def main(args):
    rospy.init_node('readBag', anonymous=True)
    #ic = image_converter()
    bm = bag_manager()
    bag_file = rospy.get_param('~bag_file')
    #bag_file = "1.bag"
    try:
        bm.read_bag(bag_file)
    except KeyboardInterrupt:
        print("Shutting down")
    #try:
    #    rospy.spin()
    #except KeyboardInterrupt:
    #    print("Shutting down")
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)