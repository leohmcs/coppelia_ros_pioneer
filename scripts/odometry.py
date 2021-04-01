#!/usr/bin/env python

import math
from math import sin, cos, pi

# import os
# os.environ["ROS_NAMESPACES"] = "/coppelia_ros_pioneer"

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState


class Robot_Odom:
    def __init__(self):
        self.w_radius = 0.0975
        self.w_base = 0.4150

        # assumindo que o robo inicia parado na origem
        self.r_wheel_vel = (0.0,)
        self.l_wheel_vel = (0.0,)

        self.vel_linear = 0.0
        self.vel_angular = 0.0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # variacoes na posicao e orientacao desde a ultima publicacao
        self.delta_s = 0.0
        self.delta_th = 0.0

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.right_joint_sub = rospy.Subscriber("/pioneer/right_wheel_joint_state", JointState, self.right_joint_callback)
        self.left_joint_sub = rospy.Subscriber("/pioneer/left_wheel_joint_state", JointState, self.left_joint_callback)

        self.right_joint_state = JointState()
        self.left_joint_state = JointState()

        self.last_time = rospy.Time.now()
        self.current_time = self.last_time

        
    def right_joint_callback(self, msg):
        self.right_joint_state = msg
        self.r_wheel_vel = msg.velocity

    
    def left_joint_callback(self, msg):
        self.left_joint_state = msg
        self.l_wheel_vel = msg.velocity 

    # calcula a posicao atual do robo
    def pose(self):
        # calcula o intervalo de tempo
        self.current_time = self.right_joint_state.header.stamp
        delta_t = (self.current_time - self.last_time).to_sec()
        # rospy.loginfo("%s", delta_t)

        # rospy.loginfo("%s", self.right_joint_state.velocity)

        # calcula velocidade linear e angular a partir das velocidades das duas rodas
        self.vel_linear = (self.r_wheel_vel[0] + self.l_wheel_vel[0]) * self.w_radius / 2
        self.vel_angular = (self.r_wheel_vel[0] - self.l_wheel_vel[0]) * self.w_radius / self.w_base

        # calcula a distancia percorrida e o angulo girado
        self.delta_s = self.vel_linear * delta_t
        self.delta_th = self.vel_angular * delta_t 

        self.last_time = self.current_time
        
        # variacao das posicoes (x, y) do robo
        self.x += self.delta_s * cos(self.th + self.delta_th / 2)
        self.y += self.delta_s * sin(self.th + self.delta_th / 2)

        self.th += self.delta_th 

        self.publish()


    def publish(self):
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        self.odom_broadcaster.sendTransform (
            (self.x, self.y, 0.),
            odom_quat,
            rospy.Time.now(), #self.current_time,
            "base_link",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        odom.child_frame_id = "wheel_odom"
        odom.twist.twist = Twist(Vector3(self.vel_linear, 0, 0), Vector3(0, 0, self.vel_angular))

        self.odom_pub.publish(odom)


rospy.init_node('odometry_publisher')
rospy.loginfo("odometry_publisher node initialization") 

odom = Robot_Odom()

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    odom.pose()
    rate.sleep()
