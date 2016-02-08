#!/usr/bin/env python

#path_client:
# illustrates how to send a request to the path_service service

import rospy
from example_ros_service.srv import PathSrv
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from std_msgs.msg import Header
import math

def convertPlanarPhiToQuaternion(phi) :
    quaternion = Quaternion()
    quaternion.x = 0
    quaternion.y = 0
    quaternion.z = math.sin(phi / 2)
    quaternion.w = math.cos(phi / 2)
    return quaternion

# Takes a Pose and adds a header to make it a PoseStamped
def stampPose(pose) :
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose

    header = Header()
    header.stamp = rospy.Time.now()
    pose_stamped.header = header 
   
    return pose_stamped

def stampPath(path) :
    header = Header()
    header.stamp = rospy.Time.now()
    path.header = header

    return path

def main() :
    rospy.init_node('path_client')

    rospy.wait_for_service('path_service')
    try :
        client = rospy.ServiceProxy('path_service', PathSrv)

    except rospy.ServiceException, e :
        rospy.logerr("Service exception, %s", e)

    rospy.loginfo("connected client to service")

    path = Path()
    quaternion = Quaternion()

    # create some path points...
    # this should be done by some intelligent algorithm
    # but we'll hard-code it here
    pose = Pose();
    pose.position.x = 1.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0; # let's hope so!
    pose.orientation.x = 0.0; # always, for motion in horizontal plane
    pose.orientation.y = 0.0; # ditto
    pose.orientation.z = 0.0; # implies oriented at yaw=0, i.e. along x axis
    pose.orientation.w = 1.0; # sum of squares of all components of unit quaternion is 1
    path.poses.append(stampPose(pose))

    quaternion = convertPlanarPhiToQuaternion(1.57)
    pose.orientation = quaternion
    pose.position.y = 1.0
    path.poses.append(stampPose(pose))
    
    quaternion = convertPlanarPhiToQuaternion(3.14)
    pose.orientation = quaternion
    path.poses.append(stampPose(pose))

    response = client(stampPath(path))

if __name__ == '__main__' :
    try :
        main()
    except rospy.ROSInterruptException :
        pass
