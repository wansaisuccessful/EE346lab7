import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt

goalPoints = [ 
    # from point1 to point2, to point3, to point4 and then back to point1
    # position[XYZ] and pose[quaternion]
    # In our map of lab, X-direction is from bottom to top and Y-direction is from right to left
    [(1.353, 0.793, 0.000), (0.000, 0.000, 0.9517, 0.3069)],
    [(0.329, 4.858, 0.000), (0.000, 0.000, -0.670, 0.743)],
    [(-3.641, 3.844, 0.000), (0.000, 0.000, 0.786, 0.618)],
    [(-2.809, -0.310, 0.000), (0.000, 0.000, 0.733, 0.680)],
    [(-2.339, 0.113, 0.000), (0.000, 0.000, 0.250, 0.968)],
]

def aruco2_cb(msg):
    global seq2
    seq2 = msg.header.seq

def aruco3_cb(msg):
    global seq3
    seq3 = msg.header.seq

def aruco4_cb(msg):
    global seq4
    seq4 = msg.header.seq

class AutoNav:
    def __init__(self):
        self.moveBaseAction = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.init_pose_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, latch=True, queue_size=1)


        wait_status = self.moveBaseAction.wait_for_server(rospy.Duration(10))
        rospy.loginfo("Waiting for move_base action server...")
        if not wait_status:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        
        rospy.loginfo("Connected to move base server!")
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.moveBaseAction.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def set_initial_pose1(self):
        '''To set the 2D pose estimate of initial moment'''
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = 'map'
        init_pose.pose.pose.position.x = -2.776 #-2.809
        init_pose.pose.pose.position.y = -0.214 #-0.310
        init_pose.pose.pose.position.z = 0.0
        init_pose.pose.pose.orientation.x = 0.0
        init_pose.pose.pose.orientation.y = 0.0
        init_pose.pose.pose.orientation.z =  0.057#0.0292
        init_pose.pose.pose.orientation.w =  0.9984#0.9995
        init_pose.pose.covariance[0] = 0.25
        init_pose.pose.covariance[7] = 0.25
        init_pose.pose.covariance[35] =  0.06853892326654787
        self.init_pose_pub.publish(init_pose)

    def set_goal(self, index):
        '''
        To produce and return the goal pose variable which contains position and orientation
        '''
        goal_pose = MoveBaseGoal()
        pose = goalPoints[index]
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]
        return goal_pose

    def move_goal(self, index, wait_time=60):
        '''To send the move command and wait for the result'''
        goal = self.set_goal(index)
        self.moveBaseAction.send_goal(goal)
        # wait_time = 60   #unit:seconds
        rospy.loginfo("Begin to navigate autonomous to the point"+str(index+1))
        finish_status = self.moveBaseAction.wait_for_result(rospy.Duration(wait_time))
        if not finish_status:
            self.moveBaseAction.cancel_goal()
            rospy.loginfo(str(wait_time)+" seconds time out without reaching the goal")
        else:
            if self.moveBaseAction.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("Arrive the point "+str(index+1))

if __name__ == '__main__':

    rospy.init_node("navController_class")
    cmd = AutoNav()
    cmd.set_initial_pose1()
    cmd.move_goal(0)
    cmd.move_goal(1)
    cmd.move_goal(2)
    cmd.move_goal(3)
    cmd.move_goal(4)
    rospy.Subscriber("/aruco_2/pose", PoseStamped, aruco2_cb, queue_size=1)
    rospy.Subscriber("/aruco_3/pose", PoseStamped, aruco3_cb, queue_size=1)
    rospy.Subscriber("/aruco_4/pose", PoseStamped, aruco4_cb, queue_size=1)
    if seq2<10000:
        cmd.move_goal(0)
    elif seq3<10000:
        cmd.move_goal(1)
    elif seq4<10000:
        cmd.move_goal(2)
    rospy.spin()
