#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from path_planner import PathPlanner

path_planned = None


def goto_goal(pos_x: float, pos_y: float):
    goal = Pose()
    goal.position.x = pos_x
    goal.position.y = pos_y
    goal.orientation.w = 1.0

    path_planned.send_goal(goal)

    # Is result successful
    return path_planned.wait_for_result(120)
    


def on_shutdown():
    path_planned.cancel_goal()


if __name__ == '__main__':
    try:
        rospy.init_node('goto_mover', anonymous=True)

        path_planned = PathPlanner()
        rospy.on_shutdown(on_shutdown)
        rospy.loginfo("Wating for map...")
        path_planned.wait_for_map()

        while not rospy.is_shutdown():
            pos_x = float(input('Input X coordinate:'))
            pos_y = float(input('Input Y coordinate:'))

            if goto_goal(pos_x, pos_y):                 # mover.py returns true if reached -> change to PPC return instead
                print("Goal reached successfully")
            else:
                print("Goal couldn't be reached")
    except rospy.ROSInterruptException:
        pass
    except ValueError:
        print("Error: not a number")
