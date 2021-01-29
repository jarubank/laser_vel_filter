#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
class NumberCounter:
    def __init__(self):
        self.counter = 0
        self.aa = False
        self.xtable =[]
    
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        
    def callback_number(self, msg):
        self.counter += msg.data
        new_msg = Int64()
        new_msg.data = self.counter
        self.pub.publish(new_msg)

    def scan_cb(self, msg):
        xtable = msg.ranges[413]
        print(xtable)

        plt.plot(msg.ranges)
        # plt.ylabel('some numbers')
        # plt.xlim(250, 575)
        # plt.ylim(0.28, 0.4)
        plt.show()


        
    
if __name__ == '__main__':
    rospy.init_node('number_counter')
    NumberCounter()
    rospy.spin()
