#!/usr/bin/env python
'''
Publish marker and frame representing a tablet where a chilitag is detected.
'''

import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

TAG_FRAME = "tag_1" #name of frame to detect tablet with (chilitag frame, with y horizontal and x vertical)
FRAME_ID = "writing_surface" #name of frame to publish as tablet (with x horizontal and y vertical)

pub_markers = rospy.Publisher('visualization_marker', Marker)

def tablet():

    w = 0.217
    h = 0.136
    tablet = Marker()
    tablet.header.frame_id = FRAME_ID
    tablet.header.stamp = rospy.get_rostime()
    tablet.pose.orientation.w = 1.0
    tablet.pose.position.z = -.0005
    tablet.pose.position.x = w/2
    tablet.pose.position.y = h/2
    tablet.id = 99
    tablet.type = Marker.CUBE
    tablet.scale.x = w
    tablet.scale.y = h
    tablet.scale.z = 0.0005
    tablet.color.b = 1.0
    tablet.color.g = 1.0
    tablet.color.r = 1.0
    tablet.color.a = 1.0
    tablet.lifetime.secs = 1; #timeout for display
    return tablet


if __name__=="__main__":
    rospy.init_node("tablet_placer")    
        
    tf_broadcaster = tf.TransformBroadcaster()
    tf_listener = tf.TransformListener(True, rospy.Duration(10))
    
    rospy.sleep(.5)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
    #tf_broadcaster.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),"v4l_frame","gaze") #manually "attach" the webcam to the robot's frame
        try:
            tf_listener.waitForTransform("map", TAG_FRAME, rospy.Time.now(), rospy.Duration(5.0))
            t = tf_listener.getLatestCommonTime("map", TAG_FRAME)
            (trans,rot) = tf_listener.lookupTransform("map", TAG_FRAME, t)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
            
        #rotate coordinate system of tag to match the desired one for the tablet
        surfacePose = PoseStamped();
        surfacePose.header.frame_id = TAG_FRAME;
        orientation = tf.transformations.quaternion_from_euler(3.14159,0,3.14159/2); #convert 'x up, y to the right' of chilitag from to 'y up, x to the right' for the tablet

        surfacePose.pose.orientation.x=orientation[0]
        surfacePose.pose.orientation.y=orientation[1]
        surfacePose.pose.orientation.z=orientation[2]
        surfacePose.pose.orientation.w=orientation[3]
        surfacePose = tf_listener.transformPose("map",surfacePose)
        o=surfacePose.pose.orientation
        #publish tablet's frame w.r.t. map
        tf_broadcaster.sendTransform(trans,(o.x,o.y,o.z,o.w),rospy.Time.now(),FRAME_ID,"map")
        pub_markers.publish(tablet()) #show tablet

    rate.sleep()
