#!/usr/bin/env python

import rospy
import smach
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
from sound_play.msg import SoundRequest
from turtlebot3_msgs.msg import *




goalPoints = [ 
    # from point1 to point2, to point3, to point4 and then back to point1
    # position[XYZ] and pose[quaternion]
    # In our map of lab, X-direction is from bottom to top and Y-direction is from right to left
    [(1.353, 0.793, 0.000), (0.000, 0.000, 0.9517, 0.3069)],
    [(0.333, 4.833, 0.000), (0.000, 0.000, -0.670, 0.743)],
    [(-3.533, 3.830, 0.000), (0.000, 0.000, 0.786, 0.618)],
    [(-2.809, -0.310, 0.000), (0.000, 0.000, 0.733, 0.680)],
    [(-2.339, 0.113, 0.000), (0.000, 0.000, 0.250, 0.968)],
    [(-2.076, 0.726,0.000), (0.000, 0.000,0.3161,0.9487)]
]

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
        init_pose.pose.pose.position.x = -2.713 #-2.809
        init_pose.pose.pose.position.y = -0.273#-0.310
        init_pose.pose.pose.position.z = 0.0
        init_pose.pose.pose.orientation.x = 0.0
        init_pose.pose.pose.orientation.y = 0.0
        init_pose.pose.pose.orientation.z = -0.0250#0.0292
        init_pose.pose.pose.orientation.w =  0.9997#0.9995
        init_pose.pose.covariance[0] = 0.25
        init_pose.pose.covariance[7] = 0.25
        init_pose.pose.covariance[35] =  0.06853892326654787
        self.init_pose_pub.publish(init_pose)


    def set_initial_pose2(self):
        '''To set the 2D pose estimate of initial moment'''
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = 'map'
        init_pose.pose.pose.position.x = -2.339
        init_pose.pose.pose.position.y = 0.113
        init_pose.pose.pose.position.z = 0.0
        init_pose.pose.pose.orientation.x = 0.0
        init_pose.pose.pose.orientation.y = 0.0
        init_pose.pose.pose.orientation.z = 0.2503
        init_pose.pose.pose.orientation.w = 0.9682
        init_pose.pose.covariance[0] = 0.25
        init_pose.pose.covariance[7] = 0.25
        init_pose.pose.covariance[35] =  0.06853892326654787
        self.init_pose_pub.publish(init_pose)


    def set_initial_pose3(self):
        '''To set the 2D pose estimate of initial moment'''
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = 'map'
        init_pose.pose.pose.position.x = 1.353
        init_pose.pose.pose.position.y = 0.792
        init_pose.pose.pose.position.z = 0.0
        init_pose.pose.pose.orientation.x = 0.0
        init_pose.pose.pose.orientation.y = 0.0
        init_pose.pose.pose.orientation.z = 0.9517
        init_pose.pose.pose.orientation.w = 0.3069
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
        rospy.loginfo("Begin to navigate autonomous to the point"+str(index+2))
        finish_status = self.moveBaseAction.wait_for_result(rospy.Duration(wait_time))
        if not finish_status:
            self.moveBaseAction.cancel_goal()
            rospy.loginfo(str(wait_time)+" seconds time out without reaching the goal")
        else:
            if self.moveBaseAction.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("Arrive the point "+str(index+2))

# define navigation
class Navi(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, ud):
        global cmd
        cmd.set_initial_pose1()
        cmd.move_goal(5)
        cmd.move_goal(0)
        cmd.move_goal(1)
        cmd.move_goal(2)
        cmd.move_goal(3)
        cmd.move_goal(4)
        return 'outcome1'



class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['outcome2','outcome3','outcome4','outcome8'])
        self.counter = 0
        self.tmp = 100000
        self.seq2 = 100001
        self.seq3 = 100001
        self.seq4 = 100001
        self.msg = Sound()
        self.pub = rospy.Publisher("/sound",Sound,queue_size=10)
        rospy.Subscriber("/aruco_2/pose", PoseStamped, self.aruco2_cb, queue_size=10)
        rospy.Subscriber("/aruco_3/pose", PoseStamped, self.aruco3_cb, queue_size=10)
        rospy.Subscriber("/aruco_4/pose", PoseStamped, self.aruco4_cb, queue_size=10)

    def aruco2_cb(self,msg):
        self.seq2 = msg.header.seq
        #rospy.loginfo(str(self.seq2))

    def aruco3_cb(self,msg):
        self.seq3 = msg.header.seq
        #rospy.loginfo(str(self.seq3))

    def aruco4_cb(self,msg):
        self.seq4 = msg.header.seq
        #rospy.loginfo(str(self.seq4))
       
        
    def execute(self, userdata):
        rospy.loginfo(str(self.seq2))
        rospy.loginfo(str(self.seq3))
        rospy.loginfo(str(self.seq4))
        rospy.sleep(2)
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
        

        if self.counter == 2:
            self.msg.value = 1
            self.pub.publish(self.msg)
            return 'outcome2'
        elif  self.counter == 3:
            self.msg.value = 2
            self.pub.publish(self.msg)
            return 'outcome3'
        elif self.counter == 4:
            self.msg.value = 3
            self.pub.publish(self.msg)
            return 'outcome4'
        else:
            return 'outcome8'




# define ID = 2
class Aruco2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome5'])

    def execute(self, userdata):
        global cmd
        #playsound (Home/Downloads/2.mp3)
        #cmd.set_initial_pose2()
        cmd.move_goal(5)
        cmd.move_goal(0)
        rospy.loginfo('Arrive p2')
        return 'outcome5'


# define ID = 3
class Aruco3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome6'])

    def execute(self, userdata):
        #cmd.set_initial_pose2()
        cmd.move_goal(5)
        cmd.move_goal(1)
        rospy.loginfo('Arrive p3')
        return 'outcome6'


# define ID = 4
class Aruco4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome7'])

    def execute(self, userdata):
        global cmd
        #cmd.set_initial_pose3()
        rospy.loginfo('moving to p4')
        cmd.move_goal(5)
        cmd.move_goal(2)
        rospy.loginfo('Arrive P4')
        return 'outcome7'

# main
def main():
    rospy.init_node('smach_example_state_machine')
    global cmd 
    cmd = AutoNav()
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome10'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Navi', Navi(), 
                               transitions={'outcome1':'FOO'})
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome2':'Aruco2', 
                                            'outcome3':'Aruco3',
                                            'outcome4':'Aruco4',
                                            'outcome8':'outcome10'})
        smach.StateMachine.add('Aruco2', Aruco2(), 
                               transitions={'outcome5':'outcome10'})
        smach.StateMachine.add('Aruco3', Aruco3(), 
                               transitions={'outcome6':'outcome10'})
        smach.StateMachine.add('Aruco4', Aruco4(), 
                               transitions={'outcome7':'outcome10'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
