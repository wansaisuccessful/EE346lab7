# EE346lab7
Lab 7 Autonomous Navigation with SMACH
1.Introduction
Refer to the figure, the overall task starts from location P1, and the robot should successively move to P2, P3, P4 and back to P1 in that order before moving toward the ArUco marker to recognize the ID of the marker placed near the red triangle in the floor plan. The ArUco marker ID is a number n = 2, 3 or 4 and, upon recognizing the ID, the robot should beep with the buzzer on TurtleBot3 n times, and then move to Pn before coming to a stop. 
               
Figure 1:Top view of the competition sketch
2.Approach 
  2.1:Mapping
     We first built maps for the racing environment using GMapping(roslaunch turtlebot3_slam turtlebot3_slam.launch). After controlling the robot running around the competition field (roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch), we can get maps as showing below.
                    

   2.2 Navigation
       Navigation launch file is an embeded file to execute the auto-navigation function with given type of robot and map file. Using RVIZ, we need to first calibrate the position of robot corresponding to the map.. The navigation node has been provided by Turtlebot3 package. we just launch the this node with the pre-built map. 
(roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml)
Our navigation program is written base on the code in navigation/Tutorials/SendingSimpleGoals - ROS Wiki. We use three methods to implement the navigation set_initial_pose, set_goal, move_goal.

2.3 Smach
The smach viewer is a GUI that shows the state of hierarchical SMACH state machines. It can visualize the possible transitions between states, as well as the currently active state and the values of user data that is passed around between states. In this lab we use SMACH to move to the goal depend on the ID of the aruco.


2.4  Aruco 
     As we test the turtlebot camera is of 6 frame per second, which the high refreshing rate enable fast recognition of AR tags. We use the launch files to create three nodes: aruco_2, aruco_3, aruco_4, by subcribing to the aruco_2/pose ,aruco_3/pose,aruco_4/pose to determine the ID of the aruco, if the aruco_x/pose.header is less than 10000, the ID of the aruco is x.
Code:
 while(self.seq2!=100001 or self.seq3!=100001 or self.seq4!=100001):
            if self.seq2<self.tmp:
                self.counter = 2
                break
            elif self.seq3<self.tmp:
                self.counter = 3
                break
            elif self.seq4<self.tmp:
                self.counter = 4
                break
 
    
   
2.5 Parameter modification
   In order to make the task better, we modification the parameter in /turtlebot3 _navigation/param. First we change the shape of the turtlebot from rectangle to circle, This makes it easier to pass tight corners.

We also enlarge the xy_goal_tolerance and the yaw_goal_tolerance, which makes the turtlebot reach the destination faster.


3.Experiment Content
1.(Remote PC) roscore
2.(Remote PC) ssh pi@raspberrypi and input password to start SSH session with Turtlebot
3.(on connected SSH session) roslaunch turtlebot3_bringup turtlebot3_robot.launch
4.(on connected SSH session) roslaunch raspicam_node camerav2_320x240.launch
5.(Remote PC) rosrun image_transport republish compressed in:=raspicam_node/image raw out:=raspicam_node/image
6.(Remote PC)roslaunch aruco2_marker_finder.launch markerId:=2 markerSize:=0.05
7.(Remote PC)roslaunch aruco3_marker_finder.launch markerId:=3 markerSize:=0.05
8.(Remote PC)roslaunch aruco4_marker_finder.launch markerId:=4 markerSize:=0.05
9.(Remote PC)rostopic echo amcl/pose
           #Get the location information of the Turtlebot, and set it in set_initial_pose method.
10.(Remote PC)python lab7.py
4.Conclusion
