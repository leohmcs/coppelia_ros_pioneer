#!/usr/bin/env python

import sys
import argparse

from matplotlib import pyplot as plt
import numpy as np

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


class PlotOdom():
    def __init__(self):
        self.odom_x = np.array([])
        self.odom_y = np.array([])

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb)

        self.odom_filtered_x = np.array([])
        self.odom_filtered_y = np.array([])

        self.odom_filtered_sub = rospy.Subscriber("odometry/filtered", Odometry, self.odom_filtered_cb)

        self.odom_sim_x = np.array([])
        self.odom_sim_y = np.array([])

        self.odom_sim_sub = rospy.Subscriber("gt_pose", Pose, self.odom_sim_cb)

        # beacuse simulation script runs in higher rate, we need to know which points actually correspond 
        # to the points on odom and odom_filtered list
        self.odom_sim_indices = []


    # wheel encoder odometry (no filter applied)
    def odom_cb(self, msg):
        self.odom_x = np.append(self.odom_x, msg.pose.pose.position.x)
        self.odom_y = np.append(self.odom_y, msg.pose.pose.position.y)

    
    # odom after fusing wheel encoder and imu data
    def odom_filtered_cb(self, msg):
        self.odom_filtered_x = np.append(self.odom_filtered_x, msg.pose.pose.position.x)
        self.odom_filtered_y = np.append(self.odom_filtered_y, msg.pose.pose.position.y)
        self.odom_sim_indices.append(len(self.odom_sim_x) - 1)        


    # real robot position received from simulator
    def odom_sim_cb(self, msg):
        self.odom_sim_x = np.append(self.odom_sim_x, msg.position.x)
        self.odom_sim_y = np.append(self.odom_sim_y, msg.position.y)


    # plots robot's pose over time
    def plot_pose(self, enc_pose, filter_pose, sim_pose, axs, i):
        odom_sim_x = np.take(self.odom_sim_x, self.odom_sim_indices)
        odom_sim_y = np.take(self.odom_sim_y, self.odom_sim_indices)

        if enc_pose:
            axs[i].plot(self.odom_x, self.odom_y, "b--", label="Giro de Roda")
        
        if filter_pose:
            axs[i].plot(self.odom_filtered_x, self.odom_filtered_y, "r-", label="Giro de Roda + IMU")
        
        if sim_pose:
            axs[i].plot(odom_sim_x, odom_sim_y, "g-", label="Real")
        
        axs[i].legend()
        axs[i].set_xlabel("X (m)")
        axs[i].set_ylabel("Y (m)")
        axs[i].set_title("Pose")


    # plot euclidean distance between points from different pose sources
    def plot_euclidean(self, accumulated, enc_sim, enc_filter, filter_sim, axs, i):
        odom_sim_x = np.array([])
        odom_sim_y = np.array([])

        if enc_sim or filter_sim:
            odom_sim_x = np.take(self.odom_sim_x, self.odom_sim_indices)
            odom_sim_y = np.take(self.odom_sim_y, self.odom_sim_indices)

        if enc_sim:
            dist_enc_sim = self.calc_dist(accumulated, self.odom_x, self.odom_y, odom_sim_x, odom_sim_y)
            axs[i].plot(dist_enc_sim, "b--", label="Giro de Roda vs Real")
            axs[i].set_xticks([])
            axs[i].set_ylabel("Distancia Euclideana (m)")

        if enc_filter:
            dist_enc_filter = self.calc_dist(accumulated, self.odom_x, self.odom_y, self.odom_filtered_x, self.odom_filtered_y)
            axs[i].plot(dist_enc_filter, "r-", label="Giro de Roda vs Giro de Roda + IMU")
            axs[i].set_xticks([])
            axs[i].set_ylabel("Distancia Euclideana (m)")

        if filter_sim:
            dist_filter_sim = self.calc_dist(accumulated, self.odom_filtered_x, self.odom_filtered_y, odom_sim_x, odom_sim_y)
            axs[i].plot(dist_filter_sim, "g-", label="Giro de Roda + IMU vs Real")
            axs[i].set_xticks([])
            axs[i].set_ylabel("Distancia Euclideana (m)")

        axs[i].legend()
        
        if accumulated:
            axs[i].set_title("Distancia Acumulada")
        else:
            axs[i].set_title("Distancia no Instante")


    def calc_dist(self, accumulated, x1, y1, x2, y2):
        dists = np.array([0.0])
        for i in range(min(len(x1), len(x2))):
            a = np.array([x1[i], y1[i]])
            b = np.array([x2[i], y2[i]])

            dist = np.linalg.norm(a - b)
            if accumulated:
                dists = np.append(dists, dist + dists[len(dists) - 1])
            else:
                dists = np.append(dists, dist)

        return dists


# parse command line arguments
parser = argparse.ArgumentParser()

parser.add_argument("--no-encpose", dest="encpose", action="store_false", help="Do not plot wheel encoder estimated pose")
parser.add_argument("--no-filterpose", dest="filterpose", action="store_false", help="Do not plot pose")
parser.add_argument("--no-realpose", dest="realpose", action="store_false", help="Do not plot simulator pose (robot's real position)")
parser.add_argument("--no-pose", dest="pose", action="store_false", help="Plot pose")

parser.add_argument("--no-encreal", dest="encreal", action="store_false", help="Do not plot Encoder vs Real error")
parser.add_argument("--no-encfilter", dest="encfilter", action="store_false", help="Do not plot Encoder vs Filter error")
parser.add_argument("--no-filterreal", dest="filterreal", action="store_false", help="Do not plot Filter vs Real error")
parser.add_argument("--no-accdist", dest="accdist", action="store_false", help="Do not accumulated error (Euclidean distance) between pose from different pose sources")
parser.add_argument("--no-dist", dest="dist", action="store_false", help="Do not plot error (Euclidean distance) between pose from different sources")

parser.set_defaults(encpose=True, filterpose=True, realpose=True, pose=True, encreal=True, encfilter=True, accdist=True, filterreal=True, dist=True)
args = parser.parse_args()

# ROS
rospy.init_node('odometry_plot')
rospy.loginfo('Plot odom node initialization. The graph will be shown once this node is shutdown')

plotOdom = PlotOdom()

rospy.spin()

# plot data
n = 3

if not args.pose:
    n -= 1
    
if not args.accdist:
    n -=1
    
if not args.dist:
    n -= 1

fig, axs = plt.subplots(1, n)
index = 0

if args.pose:
    plotOdom.plot_pose(args.encpose, args.filterpose, args.realpose, axs, index)
    index += 1

if args.accdist:
    plotOdom.plot_euclidean(True, args.encreal, args.encfilter, args.filterreal, axs, index)
    index += 1

if args.dist:
    plotOdom.plot_euclidean(False, args.encreal, args.encfilter, args.filterreal, axs, index)
    index += 1

plt.show()
