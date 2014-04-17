#!/usr/bin/env python

"""
Reads data from 'file' argument, publishes shapes on the 
/shapes_to_draw topic and adapt them based on feedback received on the 
/shape_feedback topic'
"""


import numpy
import pdb
import matplotlib.pyplot as plt
import time
from shape_modeler import ShapeModeler

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

#shape learning parameters
paramToVary = 4;            #Natural number between 1 and numPrincipleComponents, representing which principle component to vary from the template
numPrincipleComponents = 5; #Number of principle components to keep during PCA of dataset
maxNumAttempts = 500;       #Allowed attempts to draw a new shape which is significantly different to the previous one but still within the range
diffThresh = 0.25;          #How different two shapes' parameters need to be to be published for comparison
tol = 1e-2;                 #Tolerance on convergence test
stdDevMultiples = [-2, 4];  #Starting bounds for paramToVary, as multiples of the parameter's observed standard deviation in the dataset
simulatedFeedback = False;  #Simulate user feedback as whichever shape is closest to goal parameter value
    
#trajectory publishing parameters
FRAME = 'writing_surface';
dt = 0.1;                   #seconds between points in traj
t0 = 0.5;                   #time for the robot to get to the first point in traj (seconds)
delayBeforeExecuting = 0.5; #how far in future to request the traj be executed (to account for transmission delays and preparedness)
sizeScale = 0.1;            #desired max dimension of shape (metres)

pub_traj = rospy.Publisher('write_traj', Path);

rospy.init_node("shape_learner");

### ------------------------------------------------------ MESSAGE MAKER
def make_traj_msg(shape):
    traj = Path();
    traj.header.frame_id = FRAME;
    traj.header.stamp = rospy.Time.now()+rospy.Duration(delayBeforeExecuting);
    
    numPointsInShape = len(shape)/2;
    x_shape = shape[0:numPointsInShape];
    y_shape = shape[numPointsInShape:];
    for i in range(numPointsInShape):
        point = PoseStamped();
        point.pose.position.x = x_shape[i,0]*sizeScale;
        point.pose.position.y = -y_shape[i,0]*sizeScale;
        point.header.frame_id = FRAME;
        point.header.stamp = rospy.Time(t0+i*dt); #assume constant time between points for now
        traj.poses.append(point);
    return traj
    
###

bestParamValue = [];
converged = False;
numIters = 0;
### ----------------------------------------------------------- CALLBACK
def on_feedback(feedback):
    global bestParamValue #@todo: avoid global variables
    global numIters
    global converged
    global newParamValue
    #----------------------------------------------- respond to feedback
    if(numIters == 0): #interpret feedback as whether or not the first shape is good enough
        if(feedback.data == 'yes'):
            converged = True;
    else:
        bestShape = feedback.data;
        #restrict limits
        if( bestShape == 'new' ):   #new shape is better
            worstParamValue = bestParamValue;
            bestParamValue = newParamValue;  
        else:                   #new shape is worse
            worstParamValue = newParamValue;
            bestParamValue = bestParamValue;
            
        if( worstParamValue == min(bestParamValue,newParamValue) ): #shape with lower value is worse
            bounds[0] = worstParamValue; #increase min bound to worst so we don't try any lower
        else: #shape with higher value is worse
            bounds[1] = worstParamValue; #decrease max bound to worst so we don't try any higher
        
        #continue if there are more shapes to try which are different enough
        if((abs(bounds[1]-bestParamValue)-diffThresh < tol) and (abs(bestParamValue-bounds[0])-diffThresh) < tol):
            converged = True;
    
    #------------------------------------------------ continue iterating
    numIters+=1;
           
    #try again if shape is not good enough
    if(not converged):
        
        #make new shape to compare with
        [newShape, newParamValue] = shapeModeler.makeRandomShapeFromTriangular(paramToVary, bounds, bestParamValue);
        #ensure it is significantly different
        numAttempts = 1;
        while(abs(newParamValue - bestParamValue) < diffThresh and numAttempts < maxNumAttempts):
            [newShape, newParamValue] = shapeModeler.makeRandomShapeFromTriangular(paramToVary, bounds, bestParamValue);
            numAttempts+=1;      
        
        if(numAttempts>=maxNumAttempts): #couldn't find a 'different' shape in range
            print('Oh no!'); #this should be prevented by the convergence test below
         
        #publish shape and get feedback
        
        print(newParamValue);
        print(bounds);        
        
        if(simulatedFeedback):
            if(args.show):
                plt.figure(1);
                ShapeModeler.normaliseAndShowShape(newShape);
                time.sleep(1.3);
            
            #temporary code in place of feedback from user: go towards goal parameter value
            goalParamValue =  -1.5*parameterVariances[paramToVary-1];
            errors = [abs(bestParamValue- goalParamValue), abs(newParamValue- goalParamValue)];
            names = ('old', 'new');
            bestShape_idx = errors.index(min(errors));
            feedback.data = names[bestShape_idx];
            on_feedback(feedback);
        else:
            traj = make_traj_msg(newShape);
            pub_traj.publish(traj);
            
    else:          
        print('Converged');  
        if(args.show):      
            plt.show(block=True);
     
### --------------------------------------------------------------- MAIN
        
if __name__ == "__main__":

    import argparse
    parser = argparse.ArgumentParser(description='Publish shapes on the \
    /shapes_to_draw topic and adapt them based on feedback received on the /shape_feedback topic');
    parser.add_argument('file', action="store",
                    help='a .txt file containing a dataset of the shape');
    parser.add_argument('--show', action='store_true', help='display plots of the shapes');

    args = parser.parse_args();
    
    shapeModeler = ShapeModeler();
    shapeModeler.makeDataMatrix(args.file);
    shapeModeler.performPCA(numPrincipleComponents);

    parameterVariances = shapeModeler.getParameterVariances();
    bounds = numpy.array(stdDevMultiples)*parameterVariances[paramToVary-1];
    
    if(args.show):
        plt.ion(); #to plot one shape at a time

    #make initial shape
    [shape, bestParamValue] = shapeModeler.makeRandomShapeFromUniform(paramToVary, bounds);
    shape = ShapeModeler.normaliseShape(shape);
    print(bestParamValue);
    
    #publish shape and get feedback
    if(simulatedFeedback):
        #pretend that first one wasn't good enough
        feedback = String();
        feedback.data = "no";
        on_feedback(feedback); #call callback manually
        
    else:
        traj = make_traj_msg(shape);
        pub_traj.publish(traj);
        feedback_subscriber = rospy.Subscriber('shape_feedback', String, on_feedback);
        rospy.spin();
        