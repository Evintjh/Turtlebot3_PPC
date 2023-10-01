#!/usr/bin/env python

import rospy
from scipy.interpolate import splrep, splev
from geometry_msgs.msg import PoseArray, Pose

# Define a global variable to store waypoints
waypoints = None

def interpolate_waypoints(waypoints):
    rospy.loginfo("generating path...")
    x = [waypoint.position.x for waypoint in waypoints.poses]
    y = [waypoint.position.y for waypoint in waypoints.poses]
    
    # Interpolate the path using scipy
    tck, u = splrep(x, y, k=1)
    interpolated_x, interpolated_y = splev(u, tck)

    # Publish the interpolated path
    
    for i in range(len(interpolated_x)):
        pose = Pose()
        pose.position.x = interpolated_x[i]
        pose.position.y = interpolated_y[i]
        interpolated_path.poses.append(pose)
    

    while not rospy.is_shutdown():
        interpolated_path_pub.publish(interpolated_path)
        rate = rospy.Rate(10)  # Adjust the rate as needed
        rate.sleep()
    
    rospy.loginfo("path generated")



def main():
    rospy.loginfo("initialising...")
    rospy.init_node('interpolation_node')

    # Create a Path message
    interpolated_path = Path()
    interpolated_path.header.frame_id = 'map'  # Replace with the appropriate frame ID   

    # Create Publisher for interpolated path
    interpolated_path_pub = rospy.Publisher('/interpolated_path', PoseArray, queue_size=1)

    # Subscribe to the original waypoints topic
    waypoints_sub = rospy.Subscriber('/waypoints', PoseArray, interpolate_waypoints)
    




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass