#!/usr/bin/env python3
import tttp
import rospy
import matplotlib.pyplot as plt

if __name__ == '__main__':
    rospy.init_node('prediction_system_fixed')
    plt.ion()
    rospy.loginfo("init node prediction_system_fixed.py")
    obj = tttp.Listener()
    rospy.spin()
