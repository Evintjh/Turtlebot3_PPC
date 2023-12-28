#!/usr/bin/env python
import rospy
from path_planner import PathPlanner


def on_shutdown():
    path_planned.cancel_goal()

if __name__ == '__main__':
    try:
        rospy.init_node('path_planner', anonymous=True)
        path_planned = PathPlanner()
        rospy.on_shutdown(on_shutdown)
        rospy.loginfo("Wating for map...")
        path_planned.wait_for_map()
        while not rospy.is_shutdown():
            pos_x = float(input('Input X coordinate:'))
            pos_y = float(input('Input Y coordinate:'))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
