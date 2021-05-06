#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class Move():
    def __init__(self):
        # raio da roda e distancia entre as rodas do robo
        self.w_radius = 0.0975
        self.w_base = 0.4150

        # publishers que passam as velocidades de cada roda para o robo
        self.right_motor_pub = rospy.Publisher('pioneer/right_wheel_velocity', Float32, queue_size=1)
        self.left_motor_pub = rospy.Publisher('pioneer/left_wheel_velocity', Float32, queue_size=1)

        # subscriber para receber o input do teclado
        self.vel_sub = rospy.Subscriber("cmd_vel", Twist, self.vel_callback)

        self.vel = Twist()

        self.right_velocity = 0
        self.left_velocity = 0


    # mover o robo quando o input do teclado for recebido do teleop
    def vel_callback(self, data):
        self.vel = data

        vel_linear = self.vel.linear.x
        vel_angular = self.vel.angular.z

        self.right_velocity = self.right_vel(vel_linear, vel_angular)
        self.left_velocity = self.left_vel(vel_linear, vel_angular)


    # calcular velocidade da roda direita
    def right_vel(self, v, w):
        return (2*v + w*self.w_base) / (2*self.w_radius)


    # calcular velocidade da roda esquerda
    def left_vel(self, v, w):
        return (2*v - w*self.w_base) / (2*self.w_radius)


    def publish(self):
        self.left_motor_pub.publish(self.left_velocity)
        self.right_motor_pub.publish(self.right_velocity)


rospy.init_node('move', anonymous=False)
rospy.loginfo('move node initialization')

move = Move()

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    move.publish()
    rate.sleep()
