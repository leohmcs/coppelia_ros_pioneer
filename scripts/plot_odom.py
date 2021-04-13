#!/usr/bin/env python

from matplotlib import pyplot as plt
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


class PlotOdom():
    def __init__(self):
        self.odom_x = []
        self.odom_y = []

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)

        self.odom_filtered_x = []
        self.odom_filtered_y = []

        self.odom_filtered_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_filtered_cb)

        self.odom_sim_x = []
        self.odom_sim_y = []

        self.odom_sim_sub = rospy.Subscriber("/gt_relative_pose", Pose, self.odom_sim_cb)


    # wheel encoder odometry (no filter applied)
    def odom_cb(self, msg):
        self.odom_x.append(msg.pose.pose.position.x)
        self.odom_y.append(msg.pose.pose.position.y)
    
    # odom after fusing wheel encoder and imu data
    def odom_filtered_cb(self, msg):
        self.odom_filtered_x.append(msg.pose.pose.position.x)
        self.odom_filtered_y.append(msg.pose.pose.position.y)

    # real robot position received from simulator
    def odom_sim_cb(self, msg):
        self.odom_sim_x.append(msg.position.x)
        self.odom_sim_y.append(msg.position.y)

    def plot(self):
        plt.plot(self.odom_x, self.odom_y, "b--", label="Encoder Odom")
        plt.plot(self.odom_filtered_x, self.odom_filtered_y, "r-", label="Filtered Odom")
        plt.plot(self.odom_sim_x, self.odom_sim_y, "g-", label="Real Position")
        plt.legend()
        plt.show()

rospy.init_node('odometry_plot')
rospy.loginfo('Plot odom node initialization. The graph will be shown once this node is killed')

plotOdom = PlotOdom()

rospy.spin()

# data will be plotted when once is killed
plotOdom.plot()


