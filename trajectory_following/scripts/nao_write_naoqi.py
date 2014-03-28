"""
Listens for a trajectory on 'write_traj' topic and sends it to the nao via 
pyrobots interface to naoqi SDK.

Requires pyrobots and a running robot/simulation with ALNetwork proxies.

"""

import pdb;
import robots;
from robots.action import naoqi_request;
from math import sqrt
from geometry_msgs.msg import Transform, PoseStamped
from nav_msgs.msg import Path
import rospy
import tf
from copy import deepcopy

TRAJ_TOPIC = "/write_traj"

# masks for which axes naoqi is to control with its planning
AXIS_MASK_X = 1
AXIS_MASK_Y = 2
AXIS_MASK_Z = 4
AXIS_MASK_WX = 8
AXIS_MASK_WY = 16
AXIS_MASK_WZ = 32

nao = robots.Nao(ros=True, host='127.0.0.1'); #connect to webots simulator locally
nao.setpose("Crouch");

#nao.execute([naoqi_request("motion","wbEnableEffectorControl",['LArm',False])])

tl = tf.TransformListener()
#target=[0,0,0,'base_link']; pose = nao.poses[target];dtarget = nao.poses.ros.inframe(pose,"base_footprint");
#nao.execute([naoqi_request("motion","wbSetEffectorControl",["LArm",[dtarget['x'],dtarget['y'],dtarget['z']]])])
#nao.execute([naoqi_request("motion","wbEnableEffectorControl",['LArm',False])])
rospy.sleep(2)


effector   = "LArm" #LArm or RArm
space      = 2#0torso#2motion.FRAME_ROBOT
axisMask   = AXIS_MASK_X+AXIS_MASK_Y+AXIS_MASK_Z+AXIS_MASK_WX#+AXIS_MASK_WY+AXIS_MASK_WZ#control all the effector's axes 7 almath.AXIS_MASK_VEL    # just control position
isAbsolute = True

def on_traj(traj):

    first = True

    target = PoseStamped()

    target_frame = traj.header.frame_id
    target.header.frame_id = target_frame
    
    #pdb.set_trace();
    #tl.waitForTransform(ENDEFFECTOR,  target_frame, rospy.Time(), rospy.Duration(1))
    #t = tl.getLatestCommonTime(ENDEFFECTOR, target_frame)
    #toto, quaternion = tl.lookupTransform(target_frame, ENDEFFECTOR, t)
    path = []; times = []; dt=0.1;
    if(first == True):
        for trajp in traj.poses:
	    
            trajp.pose.position.z = 0.05
            #trajp.transforms[0].rotation.w = sqrt(0.5)
            #trajp.transforms[0].rotation.x = -sqrt(0.5)
            
            target.pose.position = deepcopy(trajp.pose.position)
            target.pose.orientation = deepcopy(trajp.pose.orientation)
            target_robot = tl.transformPose("base_footprint",target)
            #target = [p.transforms[0].translation.x, p.transforms[0].translation.y, p.transforms[0].translation.z, target_frame];
            #pose = nao.poses[target];
            #(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([target_robot.pose.orientation.x, target_robot.pose.orientation.y, target_robot.pose.orientation.z, target_robot.pose.orientation.w])
            
            if(effector == "LArm"):
	      roll = -1.7; #rotate wrist to the left (about the x axis, w.r.t. robot frame)
	    else:
	      roll = 1.7; #rotate wrist to the right (about the x axis, w.r.t. robot frame)
            
            point = [target_robot.pose.position.x,target_robot.pose.position.y,target_robot.pose.position.z,roll,0,0]#roll,pitch,yaw];
            #dtarget = nao.poses.ros.inframe(pose,"base_footprint");
            #point = [0.1+0.5*trajp.transforms[0].translation.y,0.1+trajp.transforms[0].translation.x*0.5, 0.3,0,0,0];
            path.append(point);
            #pdb.set_trace()
            #nao.execute([naoqi_request("motion","positionInterpolation",[effector,space,point,axisMask,[.1],isAbsolute])]);
	    #pdb.set_trace();
	    times.append(trajp.header.stamp.to_sec());
	    '''
            if(len(times)==0):
                times.append(dt);
            else:            
                times.append(dt+times[len(times)-1]);
	    '''
        #  
        #pdb.set_trace();
        nao.execute([naoqi_request("motion","positionInterpolation",[effector,space,path,axisMask,times,isAbsolute])]); 

    first = False;

pub_traj = rospy.Subscriber(TRAJ_TOPIC, Path, on_traj)
rospy.spin()
