# Turtlebot3_PPC

- Turtlebot3 in Gazebo environment 
- Purepursuit for steering control along with PID for velocity control
- Tune self.min_ld_ to adjust trajectory smoothness


To generate self-customised path manually:
- Run teleop_waypoints.py to move Turtlebot. Run follow_waypoints.py to start recording points.
- Once done, run waypoints_path.py to generate out a path to track with the points collected.


To begin Pure Pursuit:
- roslaunch follow_waypoints pure_pursuit.launch
