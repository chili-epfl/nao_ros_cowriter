"""
Listens for a trajectory on 'write_traj' topic and sends it to the nao via 
pyrobots interface to naoqi SDK.

Requires pyrobots and a running robot/simulation with ALNetwork proxies.

"""

from naoqi import ALModule, ALBroker, ALProxy
from math import sqrt
from geometry_msgs.msg import Transform, PoseStamped
from nav_msgs.msg import Path
import rospy
import tf
from copy import deepcopy
from naoqi import ALProxy

TRAJ_TOPIC = "/write_traj"

# masks for which axes naoqi is to control with its planning
AXIS_MASK_X = 1
AXIS_MASK_Y = 2
AXIS_MASK_Z = 4
AXIS_MASK_WX = 8
AXIS_MASK_WY = 16
AXIS_MASK_WZ = 32

effector   = "RArm" #LArm or RArm
#NAO_IP = '192.168.1.3';
NAO_IP = '127.0.0.1';#connect to webots simulator locally

def on_traj(traj):
    print("got traj at "+str(rospy.Time.now())) 
    if(hasFallen == False): #no harm in executing trajectory
        
        target = PoseStamped()

        target_frame = traj.header.frame_id
        target.header.frame_id = target_frame
        trajStartPosition = traj.poses[0].pose.position;
        trajStartPosition_robot = tl.transformPose("base_footprint",target)
        
        path = []; times = [];
        
        for trajp in traj.poses:
        
            trajp.pose.position.z = 0.05
            
            target.pose.position = deepcopy(trajp.pose.position)
            target.pose.orientation = deepcopy(trajp.pose.orientation)
            target_robot = tl.transformPose("base_footprint",target)

            #(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([target_robot.pose.orientation.x, target_robot.pose.orientation.y, target_robot.pose.orientation.z, target_robot.pose.orientation.w])
            
            if(effector == "LArm"):
                roll = -1.7; #rotate wrist to the left (about the x axis, w.r.t. robot frame)
            else:
                roll = 1.7; #rotate wrist to the right (about the x axis, w.r.t. robot frame)
            
            point = [target_robot.pose.position.x,target_robot.pose.position.y,target_robot.pose.position.z,roll,0,0]#roll,pitch,yaw];
            path.append(point);
            times.append(trajp.header.stamp.to_sec());
        
        #wait until time instructed to start executing
        rospy.sleep(traj.header.stamp-rospy.Time.now());
        print("executing traj at "+str(rospy.Time.now())) ;
        startTime = rospy.Time.now();
        motionProxy.positionInterpolation(effector,space,path,axisMask,times,isAbsolute);
        print("Time taken for whole trajectory: "+str((rospy.Time.now()-startTime).to_sec()));
        
    else:
        print("Got traj but not allowed to execute it because I've fallen");

class FallResponder(ALModule):
  """ Module to react to robotHasFallen events """
  
  def __init__(self, name, motionProxy, memoryProxy):
      ALModule.__init__(self, name)
      self.motionProxy = motionProxy;
      memoryProxy.subscribeToEvent("robotHasFallen",name,self.has_fallen.__name__);
      print("Subscribed");
  def has_fallen(self, *_args):
      global hasFallen
      hasFallen = True;
      self.motionProxy.killAll();
      print("Stopped task");
      
if __name__ == "__main__":
    rospy.init_node("nao_writer");

    # We need this broker to be able to construct
    # NAOqi modules and subscribe to other modules
    # The broker must stay alive until the program exists
    port = 9559;
    myBroker = ALBroker("myBroker", #I'm not sure that pyrobots doesn't already have one of these open called NAOqi?
        "0.0.0.0",   # listen to anyone
        0,           # find a free port and use it
        NAO_IP,      # parent broker IP
        port)        # parent broker port
    hasFallen = False;
    motionProxy = ALProxy("ALMotion", NAO_IP, port);
    memoryProxy = ALProxy("ALMemory", NAO_IP, port);
    postureProxy = ALProxy("ALRobotPosture", NAO_IP, port)
    fallResponder = FallResponder("fallResponder",motionProxy,memoryProxy);
    pub_traj = rospy.Subscriber(TRAJ_TOPIC, Path, on_traj)
    
    motionProxy.wbEnableEffectorControl(effector,False); #if robot has fallen it will have a hard time getting up if the effector is still trying to be kept in a particular position
    postureProxy.goToPosture("StandInit", 0.5)
    
    if(effector == "LArm"):
        motionProxy.openHand("LHand");
    else:
        motionProxy.openHand("RHand");

    tl = tf.TransformListener()

    motionProxy.wbEnableEffectorControl(effector,False);
    rospy.sleep(2)

    space      = 2
    axisMask   = AXIS_MASK_X+AXIS_MASK_Y+AXIS_MASK_Z+AXIS_MASK_WX#+AXIS_MASK_WY+AXIS_MASK_WZ#control all the effector's axes 7 almath.AXIS_MASK_VEL    # just control position
    isAbsolute = True
    
    
    rospy.spin()
    myBroker.shutdown()
    #nao.execute([naoqi_request("motion","rest",[])]);
