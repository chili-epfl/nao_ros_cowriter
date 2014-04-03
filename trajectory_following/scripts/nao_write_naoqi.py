"""
Listens for a trajectory on 'write_traj' topic and sends it to the nao via 
pyrobots interface to naoqi SDK.

Requires pyrobots and a running robot/simulation with ALNetwork proxies.

"""

import pdb;
import robots;
from naoqi import ALModule, ALBroker
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


effector   = "RArm" #LArm or RArm
#NAO_IP = '192.168.1.2';
NAO_IP = '127.0.0.1';#connect to webots simulator locally


nao = robots.Nao(ros=True, host=NAO_IP); 
#nao.execute([naoqi_request("motion","setFallManagerEnabled",[True])]);
nao.execute([naoqi_request("motion","wbEnableEffectorControl",[effector,False])]) #if robot has fallen it will have a hard time getting up if the effector is still trying to be kept in a particular position
nao.setpose("StandInit");
nao.open_gripper(); #pyrobots only does right gripper. To-do: use motion.openHand("LHand");
#pdb.set_trace();
#nao.close_gripper();

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
    print("got traj at "+str(rospy.Time.now())) 
    if(hasFallen == False): #no harm in executing trajectory

    #wait until time instructed to start executing
    rospy.sleep(traj.header.stamp-rospy.Time.now()-rospy.Duration(0.3));
    print("executing traj at "+str(rospy.Time.now()))   
    
    target = PoseStamped()

    target_frame = traj.header.frame_id
    target.header.frame_id = target_frame
    
    #pdb.set_trace();
    #tl.waitForTransform(ENDEFFECTOR,  target_frame, rospy.Time(), rospy.Duration(1))
    #t = tl.getLatestCommonTime(ENDEFFECTOR, target_frame)
    #toto, quaternion = tl.lookupTransform(target_frame, ENDEFFECTOR, t)
    path = []; times = []; dt=0.1;
    
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
	    times.append(trajp.header.stamp.to_sec());
	#pdb.set_trace();
        startTime = rospy.Time.now();
        nao.execute([naoqi_request("motion","positionInterpolation",[effector,space,path,axisMask,times,isAbsolute])]); 
        print("Time taken for whole trajectory: "+str((rospy.Time.now()-startTime).to_sec()));
    else:
      print("Got traj but not allowed to execute it because I've fallen");

pdb.set_trace();
class FallResponder(ALModule):
  """ Module to react to robotHasFallen events """
  
  def __init__(self, name, robot):
      ALModule.__init__(self, name)
      self.robot = robot;
      self.robot.execute([naoqi_request("memory","subscribeToEvent",["robotHasFallen",name,self.has_fallen.__name__])])
      print("Subscribed");
  def has_fallen(self, *_args):
      global hasFallen
      hasFallen = True;
      #self.robot.execute([naoqi_request("motion","killTasksUsingResources",["RShoulderPitch"])]);
      self.robot.execute([naoqi_request("motion","killAll",[])]);
      print("Stopped task");
      

# We need this broker to be able to construct
# NAOqi modules and subscribe to other modules
# The broker must stay alive until the program exists
pport = 9559;
myBroker = ALBroker("myBroker", #I'm not sure that pyrobots doesn't already have one of these open called NAOqi?
    "0.0.0.0",   # listen to anyone
    0,           # find a free port and use it
    NAO_IP,         # parent broker IP
    pport)       # parent broker port
hasFallen = False;
fallResponder = FallResponder("fallResponder",nao);
pub_traj = rospy.Subscriber(TRAJ_TOPIC, Path, on_traj)
rospy.spin()
myBroker.shutdown()
#nao.execute([naoqi_request("motion","rest",[])]);
