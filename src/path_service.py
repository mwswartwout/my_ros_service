#!/usr/bin/env python
# path_service:
# example showing how to receive a nav_msgs/Path request
# run with complementary path_client
# this could be useful for 

import math

import rospy
from example_ros_service.srv import PathSrv, PathSrvResponse
from geometry_msgs.msg import Pose, Twist


# Function returns the sign of a number
def sgn(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0


# Function to consider periodicity and find min delta angle
def min_spin(spin_angle):
    if spin_angle > math.pi:
        spin_angle -= 2 * math.pi
    elif spin_angle < -1 * math.pi:
        spin_angle += 2 * math.pi
    return spin_angle


# Converts from Quaternion to yaw
def convert_planar_quaternion_to_phi(quaternion):
    return 2 * math.atan2(quaternion.z, quaternion.w)


class PathService:
    move_speed = 1
    spin_speed = 1
    sample_dt = .01
    twist_cmd = Twist()
    current_pose = Pose()
    twist_commander = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=1)

    def __init__(self):
        self.twist_cmd.linear.x = 0
        self.twist_cmd.linear.y = 0
        self.twist_cmd.linear.z = 0
        self.twist_cmd.angular.x = 0
        self.twist_cmd.angular.y = 0
        self.twist_cmd.angular.z = 0

        self.current_pose.position.x = 0
        self.current_pose.position.y = 0
        self.current_pose.position.z = 0
        self.current_pose.orientation.x = 0
        self.current_pose.orientation.y = 0
        self.current_pose.orientation.z = 0
        self.current_pose.orientation.w = 0
   
        self.loop_timer = rospy.Rate(1 / self.sample_dt)

    def do_spin(self, spin_angle):
        timer = 0
        final_time = math.fabs(spin_angle) / self.spin_speed
        self.twist_cmd.angular.z = sgn(spin_angle) * self.spin_speed

        while timer < final_time:
            self.twist_commander.publish(self.twist_cmd)
            timer += self.sample_dt
            self.loop_timer.sleep()

        self.do_halt()

    def do_move(self, distance):
        timer = 0
        final_time = math.fabs(distance) / self.move_speed
        self.twist_cmd.angular.z = 0
        self.twist_cmd.linear.x = sgn(distance) * self.move_speed

        while timer < final_time:
            self.twist_commander.publish(self.twist_cmd)
            timer += self.sample_dt
            self.loop_timer.sleep()

        self.do_halt()

    def do_halt(self):
        self.twist_cmd.angular.z = 0
        self.twist_cmd.linear.x = 0

        for i in range(0, 10):
            self.twist_commander.publish(self.twist_cmd)
            self.loop_timer.sleep()

    def callback(self, request):
        rospy.loginfo('callback activated')

        npts = len(request.nav_path.poses)
        rospy.loginfo('Received path request with %d poses at time %f', npts, request.nav_path.header.stamp.secs)

        for i in range(len(request.nav_path.poses)):
            pose_desired = request.nav_path.poses[i].pose

            yaw_desired = convert_planar_quaternion_to_phi(
                pose_desired.orientation)
            rospy.loginfo('pose %d: desired yaw = %f', i, yaw_desired)
            yaw_current = convert_planar_quaternion_to_phi(
                self.current_pose.orientation)
            
            spin_angle = yaw_desired - yaw_current
            spin_angle = min_spin(spin_angle)
            self.do_spin(spin_angle)
            self.current_pose.orientation = pose_desired.orientation
            
            travel_distance = pose_desired.position.x - self.current_pose.position.x
            self.do_move(travel_distance)

        return PathSrvResponse(request.nav_path)


def main():
    rospy.init_node('path_service')

    server = PathService()
    service = rospy.Service("path_service", PathSrv, server.callback)
    rospy.loginfo("Ready accept paths.")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
