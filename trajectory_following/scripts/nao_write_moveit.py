#!/usr/bin/env python

"""
Listens for a trajectory on 'write_traj' topic and communicates with a MoveIt!
move_group to get a joint trajectory plan which is sent to the robot by a
joint_trajectory action client.

Requires running move_group (see nao_moveit) and joint_trajectory action server
(see nao_robot).

"""
import pdb
import rospy
import tf
import sys

from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from moveit_msgs.msg import PositionIKRequest, RobotState, DisplayRobotState, MotionPlanRequest, JointConstraint, Constraints, PositionConstraint

from nao_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction

from geometry_msgs.msg import Transform, PoseStamped, Pose
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

import actionlib
    
TRAJ_TOPIC = "/write_traj"
ENDEFFECTOR = "/r_gripper"
WRITING_FRAME = False;

compute_path = rospy.ServiceProxy('plan_kinematic_path',GetMotionPlan);

pub = rospy.Publisher('display_robot_state', DisplayRobotState)
rs = DisplayRobotState()

import moveit_commander
moveit_commander.roscpp_initialize(sys.argv)

rospy.init_node("love_letters_receiver")


tl = tf.TransformListener()

trajectory_client = actionlib.SimpleActionClient('joint_trajectory',JointTrajectoryAction)
trajectory_client.wait_for_server()

rospy.wait_for_service('compute_ik')
compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)


robot = moveit_commander.RobotCommander();
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("left_arm")

from copy import deepcopy
    
def on_traj(traj):

    start_pose = group.get_current_pose()

    missed = 0
    for p in traj.points:

        target_pose = deepcopy(start_pose);
        target_pose.pose.position.z+=0.1#p.transforms[0].translation.x
        #target_pose.pose.position.y+=p.transforms[0].translation.y

        service_request = PositionIKRequest()
        service_request.group_name = group.get_name()
        #service_request.ik_link_name = ENDEFFECTOR
        service_request.pose_stamped = target_pose
        #service_request.timeout.secs= 0.005
        service_request.avoid_collisions = True
        service_request.robot_state = robot.get_current_state()

        pdb.set_trace()
        group.set_joint_value_target(target_pose,True)
        plan = group.plan()
        
        goal = JointTrajectoryGoal();
        goal.trajectory = deepcopy(plan.joint_trajectory)
        goal.trajectory.points = deepcopy([plan.joint_trajectory.points[9]]) #just take the last point for now
        trajectory_client.send_goal(goal);

pub_traj = rospy.Subscriber(TRAJ_TOPIC, MultiDOFJointTrajectory, on_traj)
rospy.spin()
