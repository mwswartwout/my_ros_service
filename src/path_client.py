#!/usr/bin/env python

# path_client:
# illustrates how to send a request to the path_service service

import math

import rospy
from example_ros_service.srv import PathSrv
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Header


def convert_planar_phi_to_quaternion(phi):
    quaternion = Quaternion()
    quaternion.x = 0
    quaternion.y = 0
    quaternion.z = math.sin(phi / 2)
    quaternion.w = math.cos(phi / 2)
    return quaternion


# Takes a Pose and adds a header to make it a PoseStamped
def stamp_pose(pose):
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose

    header = Header()
    header.stamp = rospy.Time.now()
    pose_stamped.header = header 
   
    return pose_stamped


def stamp_path(path):
    header = Header()
    header.stamp = rospy.Time.now()
    path.header = header

    return path


def generate_pose(pos_x, quaternion):
    pose = Pose()
    pose.position.x = pos_x
    pose.orientation = quaternion

    return pose


def main():
    rospy.init_node('path_client')

    rospy.wait_for_service('path_service')
    try:
        client = rospy.ServiceProxy('path_service', PathSrv)
        rospy.loginfo("connected client to service")

        path = Path()

        # create some path points...
        # this should be done by some intelligent algorithm
        # but we'll hard-code it here

        # First move straight ahead while facing "east"
        quaternion = convert_planar_phi_to_quaternion(0)
        pose = generate_pose(3.85, quaternion)
        path.poses.append(stamp_pose(pose))

        # Next face "north" and move straight ahead
        quaternion = convert_planar_phi_to_quaternion(1.57)
        pose = generate_pose(3.2, quaternion)
        path.poses.append(stamp_pose(pose))

        # Now face "east" and move straight ahead
        quaternion = convert_planar_phi_to_quaternion(0)
        pose = generate_pose(3.1, quaternion)
        path.poses.append(stamp_pose(pose))

        # Now face "north" and move straight ahead
        quaternion = convert_planar_phi_to_quaternion(1.57)
        pose = generate_pose(1.8, quaternion)
        path.poses.append(stamp_pose(pose))

        # Now face "west" and move straight ahead
        quaternion = convert_planar_phi_to_quaternion(3.1)
        pose = generate_pose(5.7, quaternion)
        path.poses.append(stamp_pose(pose))

        # Now face "north" and move straight ahead
        quaternion = convert_planar_phi_to_quaternion(1.57)
        pose = generate_pose(2, quaternion)
        path.poses.append(stamp_pose(pose))

        # Now face "west" and move straight ahead
        quaternion = convert_planar_phi_to_quaternion(3.1)
        pose = generate_pose(1, quaternion)
        path.poses.append(stamp_pose(pose))

        # Now face "north" and move straight ahead
        quaternion = convert_planar_phi_to_quaternion(1.57)
        pose = generate_pose(7, quaternion)
        path.poses.append(stamp_pose(pose))

        response = client(stamp_path(path))

    except rospy.ServiceException, e:
        rospy.logerr("Service exception, %s", e)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
