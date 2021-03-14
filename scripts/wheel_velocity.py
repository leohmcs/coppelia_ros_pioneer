#!/usr/bin/env python
# import os
# os.environ["ROS_NAMESPACE"] = "/coppelia_ros_pioneer"

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class Move():
    def __init__(self):
        # raio da roda e distancia entre as rodas do robo
        self.w_radius = 0.0975
        self.w_base = 0.381

        # publishers que passam as velocidades de cada roda para o robo
        self.right_motor_pub = rospy.Publisher('/rightMotorSpeed_p3dx', Float32, queue_size=1)
        self.left_motor_pub = rospy.Publisher('/leftMotorSpeed_p3dx', Float32, queue_size=1)

        # subscriber para receber o input do teclado
        self.vel_sub = rospy.Subscriber("cmd_vel", Twist, self.vel_callback)

        self.vel = Twist()


    # mover o robo quando o input do teclado for recebido do teleop
    def vel_callback(self, data):
        self.vel = data

        vel_linear = self.vel.linear.x
        vel_angular = self.vel.angular.z

        right_velocity = self.right_vel(vel_linear, vel_angular)
        left_velocity = self.left_vel(vel_linear, vel_angular)
        # rospy.loginfo("%s %s", right_velocity, left_velocity)

        self.left_motor_pub.publish(left_velocity)
        self.right_motor_pub.publish(right_velocity)


    # calcular velocidade da roda direita
    def right_vel(self, v, w):
        return (2*v + w*self.w_base) / (2*self.w_radius)

    # calcular velocidade da roda esquerda
    def left_vel(self, v, w):
        return (2*v - w*self.w_base) / (2*self.w_radius)



rospy.init_node('move', anonymous=False)
rospy.loginfo('move node initialization')

move = Move()

rospy.spin()
