#!/usr/bin/env python

import math, sys, time

import rospy, tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Pose, Quaternion, Vector3, PointStamped

class TurnAndGo():
    def __init__(self):
        # points to follow
        self.abs_points = []
        self.rel_points = []
        
        # index of next point
        self.n = 0
        
        self.linear_vel = 0.5 # m/s
        self.angular_vel = 0.1 # rad/s

        # robot's initial pose
        self.initial_pose_ok = False
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_th = 0.0
        
        # robot's current pose
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # angle to turn to reach next point
        self.target_angle = 0.0

        self.limit_time = rospy.Time.now().to_sec()

        self.odom_sub = rospy.Subscriber('odometry/filtered', Odometry, self.odom_cb)
        self.initial_pose_sub = rospy.Subscriber('pioneer/initial_pose', Pose, self.initial_pose)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.points_pub = rospy.Publisher('points', PointStamped, queue_size=10)


    def initial_pose(self, msg):
        (_, _, self.initial_th) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        print("Rotated %s degrees", self.initial_th * (180 / math.pi))
        self.initial_x = msg.position.x
        self.initial_y = msg.position.y
        self.initial_pose_ok = True

    def odom_cb(self, msg):
        (_, _, self.th) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def abs_to_rel_points(self, abs_points):
        if abs_points is None:
            abs_points = self.abs_points
        
        rel_points = []
        # translation = [self.initial_x, self.initial_y]
        # rotation = [[math.cos(self.initial_th), -math.sin(self.initial_th)], [math.cos(self.initial_th), math.sin(self.initial_th)]]

        for i in range(0, len(self.abs_points), 2):
            abs_p = [self.abs_points[i].x, self.abs_points[i].y]
            rel_p = [abs_p[0] * math.cos(self.initial_th) - abs_p[1] * math.sin(self.initial_th),
                     abs_p[0] * math.sin(self.initial_th) + abs_p[1] * math.cos(self.initial_th)]
            rel_p = [rel_p[0] - self.initial_x, rel_p[1] - self.initial_y] 

            rel_points.append(Point(rel_p[0], rel_p[1], 0.0))

        return rel_points


    def distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    # calculates the angle the robot at p1 might rotate to align to p2
    def angle(self, p1, p2):
        hip = self.distance(p1, p2)
        angle = math.asin((p2.y - p1.y) / hip)

        if p2.x < p1.x:
            if angle < 0:
                angle = -math.pi - angle
            else:
                angle = math.pi - angle

        return angle 

    def navigate(self):
        current_position = Point(self.x, self.y, 0.0)
        try:
            next_point = self.rel_points[self.n]
        except(IndexError):
            self.stop()

        # rospy.loginfo("Going to (%s, %s)", next_point.x + self.initial_x, next_point.y + self.initial_y)
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = "odom"
        point_msg.point.x = next_point.x + self.initial_x
        point_msg.point.y = next_point.y + self.initial_y
        point_msg.point.z = 0.0

        self.points_pub.publish(point_msg)

        position_error = self.distance(current_position, next_point)
        rospy.loginfo("Position error: %s", position_error)
        
        if position_error > 0.05:
            self.target_angle = self.angle(current_position, next_point)
            # rospy.loginfo("Target angle: %s, Current angle: %s", self.target_angle, self.th)

            angle_error = self.target_angle - self.th
            # rospy.loginfo("Angle error: %s", abs(angle_error))

            if abs(angle_error) > 0.05 or rospy.Time.now().to_sec() > self.limit_time:
                self.turn(math.copysign(1, angle_error) * self.angular_vel)
            else:
                self.go()

        else:
            rospy.loginfo("Arrived at point %s", self.n + 1)
            self.n += 1


    def turn(self, velocity):
        # rospy.loginfo("Turning")
        vel_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, velocity))
        self.vel_pub.publish(vel_msg)

        self.limit_time = rospy.Time.now().to_sec() + 0.1 / abs(self.target_angle - self.th)
        
    
    def go(self):
        # rospy.loginfo("Going")
        vel_msg = Twist(Vector3(self.linear_vel, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        self.vel_pub.publish(vel_msg)


    def stop(self):
        self.vel_pub.publish(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))) 


rospy.init_node('turn_and_go')
rospy.loginfo('turn_and_go node initialization')

turn_and_go = TurnAndGo()
while not turn_and_go.initial_pose_ok:
    rospy.loginfo("Waiting for initial pose.")
    time.sleep(1)

for i in rospy.get_param("~points").split():
    turn_and_go.abs_points.append(float(i))

if not turn_and_go.abs_points:
    rospy.logfatal("No points provided. Shutting down")
    time.sleep(1)
    rospy.signal_shutdown("No points were provided. The node need a list of points to follow.")
elif len(turn_and_go.abs_points) % 2 != 0:
    rospy.logfatal("Number of coordinates must be even (two for each point): %s given. Shutting down.", len(turn_and_go.abs_points))
    time.sleep(1)
    rospy.signal_shutdown("Some point has a missing coordinate, i.e. missing x or y.")
else:
    # make [x1, y1, x2, y2, ...] into [Point(x1, y1, 0.0), Point(x2, y2, 0.0), ...] 
    for i in range(0, len(turn_and_go.abs_points), 2):
        turn_and_go.abs_points[i] = Point(turn_and_go.abs_points[i], turn_and_go.abs_points[i + 1], 0.0)

    turn_and_go.rel_points = turn_and_go.abs_to_rel_points(turn_and_go.abs_points)

rospy.loginfo("Points: %s", turn_and_go.abs_points)
rospy.loginfo("Relative Points: %s", turn_and_go.rel_points)

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    if turn_and_go.n >= len(turn_and_go.rel_points):
        turn_and_go.stop()
        rospy.loginfo("Finished")
        break
    
    turn_and_go.navigate()
    rate.sleep()

