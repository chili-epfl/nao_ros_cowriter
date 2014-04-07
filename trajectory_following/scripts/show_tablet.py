#!/usr/bin/env python
'''
Publish marker and frame representing a tablet where a chilitag is detected.
'''

import rospy
import tf
from visualization_msgs.msg import Marker

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
    buf = tf2_ros.Buffer()
    tf2_listener = tf2_ros.TransformListener(buf)
    
        
    tf_broadcaster = tf.TransformBroadcaster()
    tf_listener = tf.TransformListener(True, rospy.Duration(10))
    
    #rospy.sleep(10.0)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    #tf_broadcaster.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),"v4l_frame","gaze")
        try:
            tf_listener.waitForTransform("map", TAG_FRAME, rospy.Time.now(), rospy.Duration(5.0))
            t = tf_listener.getLatestCommonTime("map", TAG_FRAME)
            (trans,rot) = tf_listener.lookupTransform("map", TAG_FRAME, t)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("no")
            continue
        orientation = tf.transformations.quaternion_from_euler(3.14,0,3.14/2);
        tf_broadcaster.sendTransform((0,0,0),orientation,rospy.Time.now(),FRAME_ID,TAG_FRAME)
        pub_markers.publish(tablet()) 

    rate.sleep()
