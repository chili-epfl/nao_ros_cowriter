#!/usr/bin/env python

"""
Listens on 'write_traj' topic for a trajectory message, and publishes the corresponding 
trajectory with markers on the 'visualization_markers' topic, as an animation.

Trajectory coordinates are published in the 'paper_sheet' frame. You may want to first
broadcast it.
"""

import logging
logger = logging.getLogger("write." + __name__)
logger.setLevel(logging.DEBUG)

handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(levelname)s] %(name)s -> %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

import rospy
from visualization_msgs.msg import Marker
from trajectory_msgs.msg import MultiDOFJointTrajectory

FRAME = "paper_sheet"

pub_markers = rospy.Publisher('visualization_marker', Marker)

rospy.init_node("love_letters_display")


def visualize_traj(points):

    traj = Marker()
    traj.header.frame_id = FRAME
    traj.header.stamp = rospy.get_rostime()
    traj.ns = "love_letter"
    traj.action = Marker.ADD
    traj.pose.orientation.w = 1.0
    traj.id = 0
    traj.type = Marker.LINE_STRIP
    traj.scale.x = 0.001 # line width
    traj.color.r = 0.1
    traj.color.r = 0.1
    traj.color.b = 0.5
    traj.color.a = 1.0
    traj.lifetime.secs = 1; #timeout for display
    
    traj.points = list(points)
    
    # use interactive marker from place_paper instead
    #pub_markers.publish(a4_sheet()) 
    pub_markers.publish(traj)

def on_traj(requested_traj):
    written_points = []
    print("got traj")   
    
    #wait for robot to get to starting point
    rospy.sleep(requested_traj.points[0].time_from_start); 

    #add points to the display one at a time, like an animation
    for i in range(len(requested_traj.points)-1): 
	p = requested_traj.points[i].transforms[0].translation;
        written_points.append(p)
        visualize_traj(written_points)
        duration = requested_traj.points[i+1].time_from_start - requested_traj.points[i].time_from_start;
        rospy.sleep(duration); #wait until it's time to show the next point
        
    #show final point (no sleep afterwards, but it does have a "lifetime" set in visualize_traj)    
    p = requested_traj.points[len(requested_traj.points)-1].transforms[0].translation;
    written_points.append(p)
    visualize_traj(written_points)

#when we get a trajectory, start publishing the animation
pub_traj = rospy.Subscriber('write_traj', MultiDOFJointTrajectory, on_traj)
rospy.spin()
