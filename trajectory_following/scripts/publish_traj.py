#!/usr/bin/env python

"""
Takes an SVG file containing a path as input, and publishes the corresponding 
trajectory on the 'write_traj' topic using nav_msgs/Path message. The timestamps
of the PoseStamped vector in the Path are the time from start of the point in 
the trajectory.

If started with a "--show" option, it also publishes the trajectory with markers
on the visualization_markers topic.

Requires:
    - svg2traj (which itself relies on softMotion-libs) to compute the trajectory
    OR
    - svg_subsampler to compute the trajectory (from cowriter-trajectory-generator)

Trajectory coordinates are published in the 'paper_sheet' frame. You may want to first
broadcast it.
"""

#SVG_SAMPLER = "svg2traj"
SVG_SAMPLER = "svg_subsampler"
YFLIP = "yflip" #yflip or no-yflip
# svg_subsampler parameters
SAMPLE_DENSITY = 8 # points per cm
SAMPLE_TYPE = "homogeneous"
#SAMPLE_TYPE = "curvature"
dt = 0.1; #seconds between points in traj
t0 = 0.5; #extra time for the robot to get to the first point in traj
import logging
logger = logging.getLogger("write." + __name__)
logger.setLevel(logging.DEBUG)

handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(levelname)s] %(name)s -> %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

import sys
import math
import subprocess

import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, Transform, Twist, Vector3
from nav_msgs.msg import Path

FRAME = "paper_sheet"

pub_traj = rospy.Publisher('write_traj', Path)
pub_markers = rospy.Publisher('visualization_marker', Marker)

rospy.init_node("love_letters_sender")

def a4_sheet():

    w = 0.21
    h = 0.297

    sheet = Marker()
    sheet.header.frame_id = FRAME
    sheet.header.stamp = rospy.get_rostime()
    sheet.ns = "robot_loves_letters"
    sheet.action = Marker.ADD
    sheet.pose.orientation.w = 1.0
    sheet.pose.position.z = -.0005
    sheet.pose.position.x = w/2
    sheet.pose.position.y = h/2
    sheet.id = 99
    sheet.type = Marker.CUBE
    sheet.scale.x = w
    sheet.scale.y = h
    sheet.scale.z = 0.0005
    sheet.color.b = 1.0
    sheet.color.g = 1.0
    sheet.color.r = 1.0
    sheet.color.a = 1.0

    return sheet

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
    
    traj.points = list(points)
    
    # use interactive marker from place_paper instead
    #pub_markers.publish(a4_sheet()) 
    pub_markers.publish(traj)

def get_traj(svgfile):
    logger.info("Running " + SVG_SAMPLER + "...")
    if(SVG_SAMPLER=="svg2traj"):
        p = subprocess.Popen([SVG_SAMPLER, svgfile], stdout=subprocess.PIPE, stderr = subprocess.PIPE)
    elif(SVG_SAMPLER=="svg_subsampler"):    
        p = subprocess.Popen([SVG_SAMPLER, svgfile, str(SAMPLE_DENSITY), SAMPLE_TYPE, YFLIP], stdout=subprocess.PIPE, stderr =    subprocess.PIPE)
    
    traj, errors = p.communicate()
    logger.info(errors)

    traj = traj.strip().split("\n")

    # first line of the output of svg2traj is the x,y origin of the path in meters,
    # relative to the SVG document origin
    x_orig, y_orig = [float(x) for x in traj[0].split()]
    for l in traj[1:]:
        x, y, z = l.split()
        if(SVG_SAMPLER=="svg2traj"):
            x=x_orig + float(x);
            y=y_orig + float(y);
        elif(SVG_SAMPLER=="svg_subsampler"):
            x=float(x)*0.4;
            y=float(y)*0.4 + .05;

        yield Vector3(x, y, 0) # stange values on Z! better set it to 0


def make_traj_msg(points):
    traj = Path()
    traj.header.frame_id = FRAME
    traj.header.stamp = rospy.Time()
    
    for i, p in enumerate(points):
        point = PoseStamped();
        point.pose.position = p;
        point.header.frame_id = FRAME;
        point.header.stamp = rospy.Time(t0+(i+1)*dt); #assume constant time between points for now
        traj.poses.append(point)
    return traj

if __name__ == "__main__":


    import argparse

    parser = argparse.ArgumentParser(description='Publish an SVG trajectory as a ROS Path')
    parser.add_argument('file', action="store",
                    help='an SVG file containing a single path')
    parser.add_argument('--show', action='store_true', help='publish the trajectory as markers on /visualization_markers')

    args = parser.parse_args()
    raw_traj = list(get_traj(args.file))
    rate = rospy.Rate(.1) # continually publish since paper may be an interactive marker
    while not rospy.is_shutdown():
        if args.show:
            #logger.info("Publishing the trajectory visualization...")
            for i in range(2):
                visualize_traj(raw_traj)
	traj = make_traj_msg(raw_traj)
	pub_traj.publish(traj)
        rate.sleep()
