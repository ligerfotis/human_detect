#!/usr/bin/env python
# Ros libraries
import rospy
import sys
import numpy as np
from sensor_msgs.msg import LaserScan
#from human_detect.msg import laserScanner
from human_detect.msg import uniMSG

#prev_ranges = np.full(721,30)


class laserScan_converter:

    def __init__(self):
        self.laser_pub = rospy.Publisher('/new_scan', LaserScan, queue_size = 10)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.first_ranges = None
        self.threshold = 0.01
        self.RANGE_MAX = None
        self.threshold_min = 0.1
        self.separated = rospy.get_param("~separated")


    def checkChange(self, laserMSG):
        ranges = laserMSG.ranges
        if self.first_ranges is None:
            self.first_ranges = ranges
        if self.RANGE_MAX is None:
            self.RANGE_MAX = laserMSG.range_max
        
        # absolute difference between two frames
        abs_dif_array = np.absolute( np.subtract( self.first_ranges, ranges ) )
        if (all(x > self.RANGE_MAX for x in ranges)):
            print("Here")
        #print(np.sort(abs_dif_array))
        #print(np.argmax(abs_dif_array))
        
        # error threshold array
        for value in abs_dif_array:
            if value > self.threshold_min:
                return True
        return False

    # ToDo fix
    def formatScannerOutput(self, ranges):
        abs_dif_array = np.absolute( np.subtract( self.first_ranges, ranges ) )
        new_ranges = np.asarray(ranges)
        for i in range(len(abs_dif_array)):
            if abs_dif_array[i] <= self.threshold_min:
                new_ranges[i] = self.RANGE_MAX + 1
        return new_ranges

    def callback(self, msgScanner):
        if self.separated:
            # check movement is True
            flag = self.checkChange(msgScanner)
            #print("Flag: " + str(flag))
            if flag:
                new_ranges = self.formatScannerOutput(msgScanner.ranges)

                msg = msgScanner
                msg.ranges = new_ranges

                # publish the created message
                self.laser_pub.publish(msg)
            # do nothing
            else:
                a=0
                #print("Print NAN")
        else:
            # check movement is True
            flag = self.checkChange(msgScanner)
            if flag:
                new_ranges = self.formatScannerOutput(msgScanner.ranges)
                #print(new_ranges)
                msg = uniMSG()
                msg.distances = new_ranges
                step = abs((msgScanner.angle_min - msgScanner.angle_max))/len(new_ranges)
                msg.angles = np.arange(msgScanner.angle_min, msgScanner.angle_max, step)

                # publish the created message
                pub = rospy.Publisher('/uni_message', uniMSG, queue_size = 10)
                pub.publish(msg)
            # do nothing
            #else:
                #a=0
                #print("Print NAN")


        prev_ranges = msgScanner.ranges


def listener(args):
    rospy.init_node('ScanListener', anonymous=True)
    ic = laserScan_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    listener(sys.argv)