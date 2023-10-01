#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
import math
import rospkg 

# Path for saving and retrieving the pose.csv file 
output_file_path = rospkg.RosPack().get_path('follow_waypoints') + "/saved_path/pose3.csv"

class WaypointCollector:
    def __init__(self):
        rospy.init_node('waypoint_collector')

        # Initialize variables
        self.waypoints = []
        self.last_position = None
        self.position_threshold = 0.1  # Adjust the threshold as needed

        # Subscribe to the amcl_pose topic
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)

        # Create a publisher for robot movement (optional)
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def amcl_pose_callback(self, amcl_pose):
        rospy.loginfo("Collecting waypoints with teleop...")
        current_position = amcl_pose.pose.pose.position

        if self.last_position is None:
            # Initialize last_position on the first callback
            self.last_position = current_position
            return

        # Calculate the Euclidean distance between current and last positions
        distance = ((current_position.x - self.last_position.x) ** 2 +
                    (current_position.y - self.last_position.y) ** 2 +
                    (current_position.z - self.last_position.z) ** 2) ** 0.5

        if distance > self.position_threshold:
            # Capture the robot's position as a waypoint
            waypoint = PoseStamped()
            waypoint.header.stamp = rospy.Time.now()
            waypoint.header.frame_id = "map"  # Adjust the frame_id as needed
            waypoint.pose = amcl_pose.pose.pose
            self.waypoints.append(waypoint)
            with open(output_file_path, 'w') as file:
                for current_pose in self.waypoints:
                    pose = current_pose.pose
                    file.write(
                        str(pose.position.x) + ',' + str(pose.position.y) + ',' + str(pose.position.z) + ',' +
                        str(pose.orientation.x) + ',' + str(pose.orientation.y) + ',' +
                        str(pose.orientation.z) + ',' + str(pose.orientation.w) + '\n')
            rospy.loginfo('Poses written to ' + output_file_path)

            # Update the last_position
            self.last_position = current_position

    def run(self):
        rate = rospy.Rate(10)  # Adjust the rate as needed

        while not rospy.is_shutdown():
            # Add any additional processing or control logic here
            rate.sleep()

if __name__ == '__main__':
    try:
        collector = WaypointCollector()
        collector.run()
    except rospy.ROSInterruptException:
        pass
