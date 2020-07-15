#!/usr/bin/env python3
import tttp_all
import rospy
import matplotlib.pyplot as plt

if __name__ == '__main__':
    rospy.init_node('prediction_system_not_fixed')
    plt.ion()
    rospy.loginfo("init node prediction_system_not_fixed.py")
    obj = tttp_all.Listener()
    rospy.spin()
