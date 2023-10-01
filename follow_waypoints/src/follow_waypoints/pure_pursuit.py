#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
# from ackermann_msgs.msg import AckermannDriveStamped
from math import atan2

class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit')
        self.tfBuffer_ = tf2_ros.Buffer()
        self.tfListener_ = tf2_ros.TransformListener(self.tfBuffer_)
        self.map_frame_ = rospy.get_param("map_frame", "map")
        self.base_frame_ = rospy.get_param("base_frame", "base_link")
        self.ld_gain_ = rospy.get_param("ld_gain", 1.0)
        self.min_ld_ = rospy.get_param("min_ld", 0.6)                                      # Tune
        # self.car_wheel_base_ = rospy.get_param("car_wheel_base", 0.22)                     # Tune -> this car wheel base is the y-axis wheelbase
        self.controller_freq_ = rospy.get_param("controller_freq", 10)                     
        self.ld_ = self.min_ld_
        self.ros_rate_ = rospy.Rate(self.controller_freq_)
        self.K_linear_p = 0.6
        self.K_linear_i = 0.02
        # self.K_angular_p = 0.75
        # self.K_angular_i = 0.02

        # self.control_pub_ = rospy.Publisher('/pure_pursuit/control', AckermannDriveStamped, queue_size=1)
        self.control_pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_clk)
        rospy.Subscriber('/target_path', Path, self.path_clk)
        self.l_point_pub_ = rospy.Publisher('/pure_pursuit/lookahead_point', PointStamped, queue_size=1)

        self.path_ = []
        self.got_path_ = False
        self.point_idx_ = 0
        self.last_p_idx_ = 0
        self.last_dist_ = 0
        self.loop_ = False
        self.car_speed_ = 0
        self.base_location_ = None
        self.target_point_ = None
        self.Odometry = Odometry()
        self.control_msg = Twist()
        self.time = rospy.get_rostime()
        self.windup_limit_ = 1.0
        self.point_idx_ = 0

    def odom_clk(self, msg):
        self.car_speed_ = msg.twist.twist.linear.x
        self.ld_ = max(self.ld_gain_ * self.car_speed_, self.min_ld_)

    def path_clk(self, msg):
        rospy.loginfo("New path is received.")
        self.path_ = msg.poses
        self.got_path_ = True
        start_end_dist = self.distance(self.path_[0].pose.position, self.path_[-1].pose.position)
        rospy.loginfo("Start to End Distance: %f", start_end_dist)
        rospy.loginfo("Min lookup distance: %f", self.min_ld_)
        if start_end_dist <= self.min_ld_:
            self.loop_ = True
            rospy.loginfo("Is Loop: True")

    def control_loop(self):
        while not rospy.is_shutdown():
            if self.got_path_:
                try:
                    self.base_location_ = self.tfBuffer_.lookup_transform(                          # bot's POV of map wrt to its base
                        self.map_frame_, self.base_frame_, rospy.Time(0), rospy.Duration(0.1))
                    
                    for self.point_idx_ in range(self.point_idx_, len(self.path_)):
                        print(self.path_[self.point_idx_])
                        distance_ = self.distance(self.path_[self.point_idx_].pose.position,
                                                  self.base_location_.transform.translation)
                        # print(distance_)
                        rospy.loginfo("Point ID: %d, Distance %f", self.point_idx_, distance_)
                        if distance_ >= self.ld_:
                            self.path_[self.point_idx_].header.stamp = rospy.Time.now()
                            # print(self.path_[self.point_idx_])
                            transform_exists = self.tfBuffer_.can_transform(self.base_frame_, self.path_[self.point_idx_].header.frame_id, rospy.Time(0))
                            print(transform_exists)
                            if transform_exists:
                                self.target_point_ = self.tfBuffer_.transform(self.path_[self.point_idx_], self.base_frame_,
                                                                           rospy.Duration(0.1))      # bot's POV of path wrt to its base -> needs it for its own decision
                            break

                    ld_2 = self.ld_ * self.ld_
                    # x_t = self.target_point_.pose.position.x
                    y_t = self.target_point_.pose.position.y                # lateral  distance b/w bot & target -> because target/path point has been transformed to be wrt to base frame, thus base frame coordinates would be = (0,0), & the y coordinate of target pt = lateral  distance
                    print(y_t)
                    delta = atan2(2 * y_t, ld_2)
                    # delta = atan2(y_t, x_t)
                    print(f"{delta} radians")
 
                    dt = rospy.get_rostime() - self.time
                    dt_float = dt.to_sec()
                    self.time = rospy.get_rostime()
                    # control_msg.drive.steering_angle = delta
                    # control_msg.drive.speed = 2                                                                       # not sure if can use this integral mtd here


                    if distance_ > abs(self.windup_limit_):                                                       
                        distance_ = abs(self.windup_limit_)                                                         


                    # self.control_msg.angular.z = self.K_linear_p * (delta - self.Odometry.twist.twist.angular.z) + 0.5 * self.K_angular_i * dt_float * (delta - self.Odometry.twist.twist.angular.z)
                    self.control_msg.linear.x = self.K_linear_p * distance_ + 0.5 * self.K_linear_i * dt_float * distance_
                    self.control_msg.angular.z = delta
                    # self.control_msg.linear.x = 0.2                         # pub to cmd_vel      
                    # self.control_msg.header.stamp = rospy.Time.now()
                    self.control_pub_.publish(self.control_msg)

                    self.last_p_idx_ = self.point_idx_
                    self.last_dist_ = distance_
                    if self.point_idx_ == len(self.path_) and self.loop_:
                        self.point_idx_ = 0
                    elif self.point_idx_ == len(self.path_)-1:
                        if distance_ >= 0.1:
                            self.control_msg.linear.x = self.K_linear_p * distance_ + 0.5 * self.K_linear_i * dt_float * distance_
                            self.control_msg.angular.z = delta 
                        else:
                            rospy.loginfo("Reached final point")                # add in 'if distance_ != 0: continue to publish linear.x and angular.z
                            self.control_msg.angular.z = 0                      # if odom / baselink orientation != path id orientation, turn bot back by -delta
                            self.control_msg.linear.x = 0                       
                            # control_msg.header.stamp = rospy.Time.now()
                            self.control_pub_.publish(self.control_msg)
                            self.got_path_ = False
                            # self.point_idx_ = 0

                    lookahead_p = PointStamped()
                    lookahead_p.point = self.path_[self.point_idx_].pose.position
                    lookahead_p.header = self.path_[self.point_idx_].header
                    self.l_point_pub_.publish(lookahead_p)

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                    rospy.logwarn(str(ex))

            self.ros_rate_.sleep()

    def distance(self, pos1, pos2):                     # euclidean distance function
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        return (dx**2 + dy**2)**0.5

if __name__ == '__main__':
    try:
        pp_node = PurePursuit()
        pp_node.control_loop()
    except rospy.ROSInterruptException:
        pass
