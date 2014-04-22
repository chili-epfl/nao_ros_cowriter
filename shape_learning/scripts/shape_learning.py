#!/usr/bin/env python

"""
Publishes shapes specified in 'word' argument on the /shapes_to_draw 
topic and adapts them based on feedback received on the /shape_feedback 
topic. Feedback messages should be formatted as shapeName_bestShapeIndex
eg. 'a_2'.
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
maxNumAttempts = 1000;      #Allowed attempts to draw a new shape which is significantly different to the previous one but still within the range (just a precaution; sampling should be theoretically possible)
diffThresh = 0.2;           #How different two shapes' parameters need to be to be published for comparison
tol = 1e-2;                 #Tolerance on convergence test
stdDevMultiples = [-6, 6];  #Starting bounds for paramToVary, as multiples of the parameter's observed standard deviation in the dataset
simulatedFeedback = False;  #Simulate user feedback as whichever shape is closest to goal parameter value
doGroupwiseComparison = True; #instead of pairwise comparison with most recent two shapes
#@todo: make groupwise comparison/pairwise comparison different implementations of shapeLearner class
#trajectory publishing parameters
FRAME = 'writing_surface';  #Frame ID to publish points in
FEEDBACK_TOPIC = 'shape_feedback'; #Name of topic to receive feedback on
SHAPE_TOPIC = 'write_traj'; #Name of topic to publish shapes to
t0 = 0.5;                   #Time allowed for the first point in traj (seconds)
dt = 0.1;                   #Seconds between points in traj
delayBeforeExecuting = 0.5; #How far in future to request the traj be executed (to account for transmission delays and preparedness)
sizeScale = 0.06            #Desired max dimension of shape (metres)

pub_traj = rospy.Publisher(SHAPE_TOPIC, Path);

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

import bisect
class ShapeLearner:
    def __init__(self, shapeModeler, feedbackManagerEnabled, shape_learning):
        self.shapeModeler = shapeModeler;
        self.feedbackManagerEnabled = feedbackManagerEnabled; #if false, this shape subscribes to the feedback topic itself
        self.converged = False;
        self.numIters = 0;
        self.newParamValue = []; #this is just for the pair-wise one
        self.shape_learning = shape_learning;
        
### ----------------------------------------------------- START LEARNING        
    def startLearning(self, startingBounds):
        self.bounds = startingBounds;
        
        #make initial shape
        [shape, self.bestParamValue] = self.shapeModeler.makeRandomShapeFromUniform(paramToVary, self.bounds);
        shape = ShapeModeler.normaliseShape(shape);
        print('Bounds: '+str(self.bounds));
        print('Test param: '+str(self.bestParamValue));
        if(doGroupwiseComparison):
            self.params_sorted = [self.bounds[0], self.bounds[1]];
            bisect.insort(self.params_sorted, self.bestParamValue);
            self.shapeToParamMapping = [self.bestParamValue];
        
        #publish shape and get feedback
        if(simulatedFeedback):
            #pretend that first one wasn't good enough
            feedback = String();
            feedback.data = "no";
            if not self.feedbackManagerEnabled:
                self.on_feedback(feedback); #call callback manually
            
        else:
            traj = make_traj_msg(shape);
            pub_traj.publish(traj);
            if not self.feedbackManagerEnabled:
                #manage callback ourselves
                feedback_subscriber = rospy.Subscriber(FEEDBACK_TOPIC, String, self.on_feedback);
                rospy.spin();
        
### ----------------------------------------------- MAKE DIFFERENT SHAPE        
    def makeShapeDifferentTo(self, paramValue):
        #make new shape to compare with
        [newShape, newParamValue] = self.shapeModeler.makeRandomShapeFromTriangular(paramToVary, self.bounds, self.bestParamValue);
        #ensure it is significantly different
        numAttempts = 1;
        while(abs(newParamValue - paramValue) < diffThresh and numAttempts < maxNumAttempts):
            [newShape, newParamValue] = self.shapeModeler.makeRandomShapeFromTriangular(paramToVary, self.bounds, self.bestParamValue);
            numAttempts+=1;      
        
        if(numAttempts>=maxNumAttempts): #couldn't find a 'different' shape in range
            print('Oh no!'); #this should be prevented by the convergence test below
        
        #store it as an attempt
        if(doGroupwiseComparison):
            bisect.insort(self.params_sorted, newParamValue);
            self.shapeToParamMapping.append(newParamValue);
            
        return newShape, newParamValue   
        
### ------------------------------------------------------ PUBLISH SHAPE        
    def publishShapeAndWaitForFeedback(self, shape, newParamValue):
        if(simulatedFeedback):
            if(args.show):
                plt.figure(1);
                ShapeModeler.normaliseAndShowShape(shape);
                time.sleep(1.3);
            
            #temporary code in place of feedback from user: go towards goal parameter value
            goalParamValue =  numpy.float64(-0.22);#-1.5*parameterVariances[paramToVary-1];
            if(doGroupwiseComparison):
                errors = numpy.ndarray.tolist(abs(shapeToParamMapping-goalParamValue));
                bestShape_idx = errors.index(min(errors));
                feedback.data = self.shape_learning + '_' + str(bestShape_idx);
            else:   
                errors = [abs(self.bestParamValue- goalParamValue), abs(newParamValue- goalParamValue)];
                names = ('old', 'new');
                bestShape_idx = errors.index(min(errors));
                feedback.data = names[bestShape_idx];
            if not self.feedbackManagerEnabled:
                self.on_feedback(feedback);
        else:
            traj = make_traj_msg(shape);
            pub_traj.publish(traj);
            
### ------------------------------------------------ RESPOND TO FEEDBACK                
    def respondToFeedback(self, bestShape):
        if(doGroupwiseComparison):
            bestShape = int(bestShape)
            self.bestParamValue = self.shapeToParamMapping[bestShape];
            bestParamValue_index = bisect.bisect(self.params_sorted,self.bestParamValue) - 1; #indexing seems to start at 1 with bisect
            self.bounds = [self.params_sorted[bestParamValue_index-1],self.params_sorted[bestParamValue_index+1] ];
            #restrict bounds if they were caused by other shapes, because it must be sufficiently different to said shape(s)
            if((bestParamValue_index-1) > 0): #not the default min
                self.bounds[0] += diffThresh;
            if((bestParamValue_index+1) < (len(self.params_sorted)-1)): #not the default max
                self.bounds[1] -= diffThresh;

        else: #do pairwise comparison with most recent shape and previous
            #restrict limits
            if( bestShape == 'new' ):   #new shape is better
                worstParamValue = self.bestParamValue;
                bestParamValue = self.newParamValue;  
            else:                   #new shape is worse
                worstParamValue = self.newParamValue;
                bestParamValue = self.bestParamValue;
                
            if( worstParamValue == min(self.bestParamValue,self.newParamValue) ): #shape with lower value is worse
                self.bounds[0] = worstParamValue; #increase min bound to worst so we don't try any lower
            else: #shape with higher value is worse
                self.bounds[1] = worstParamValue; #decrease max bound to worst so we don't try any higher
                
        #return self.bounds
        
### ----------------------------------------------------------- CALLBACK       
    def on_feedback(self, feedbackMessage):        
        feedback = feedbackMessage.data.split('_');
        shape_messageFor = feedback[0];
        if not shape_messageFor == self.shape_learning:
            print('Ignoring message because it was for ' + shape_messageFor + 
            ' not for this shape (' + self.shape_learning + ').');
        else:
            feedbackData = feedback[1];
            #------------------------------------------- respond to feedback
            if(self.numIters == 0): #interpret feedback as whether or not the first shape is good enough
                if(feedbackData == '0'):
                    self.converged = True;
            else:
                bestShape = feedbackData;
                
                self.respondToFeedback(bestShape); #update bounds and bestParamValue
                
                #continue if there are more shapes to try which are different enough
                if((abs(self.bounds[1]-self.bestParamValue)-diffThresh < tol) and (abs(self.bestParamValue-self.bounds[0])-diffThresh) < tol):
                    self.converged = True;
            
            #-------------------------------------------- continue iterating
            self.numIters+=1;
                   
            #try again if shape is not good enough
            if(not self.converged):
                [newShape, newParamValue] = self.makeShapeDifferentTo(self.bestParamValue);
                self.newParamValue = newParamValue;
                print('Bounds: '+str(self.bounds));
                print('Test param: '+str(newParamValue));        
                
                #publish shape and get feedback
                self.publishShapeAndWaitForFeedback(newShape,newParamValue);
                    
            else:          
                print('Converged');  
                if(args.show):      
                    plt.show(block=True);
         
###--------------------------------------------- WORD LEARNING FUNCTIONS
# @todo make methods of a class
def getDatasetFiles(charsToGet):
    datasetFiles = [];
    for char in charsToGet:
        if char == 'd':
            datasetFiles.append('../res/d_cursive_dataset.txt');
        elif char == 's':
            datasetFiles.append('../res/s_cursive_dataset.txt');
        else:
            raise RuntimeError("Dataset is not known for shape "+ char);
    
    return datasetFiles;
    
def initialiseShapeLearners(charsToLearn):
    datasetFiles = getDatasetFiles(charsToLearn);  
    shapeLearners = [];
    
    for i in range(len(charsToLearn)):
        #analyse database of shapes
        shapeModeler = ShapeModeler();
        shapeModeler.makeDataMatrix(datasetFiles[i]);
        shapeModeler.performPCA(numPrincipleComponents);

        #learn parameter of shape
        parameterVariances = shapeModeler.getParameterVariances();
        bounds = numpy.array(stdDevMultiples)*parameterVariances[paramToVary-1];  
        
        shapeLearner = ShapeLearner(shapeModeler,True,charsToLearn[i]);
        shapeLearner.startLearning(bounds);
        shapeLearners.append(shapeLearner);
        
    return shapeLearners;
    
def feedbackManager(feedbackMessage):
    feedback = feedbackMessage.data.split('_');
    shape_messageFor = feedback[0];
    try:
        shapeIndex_messageFor = args.word.index(shape_messageFor);
        shapeLearners[shapeIndex_messageFor].on_feedback(feedbackMessage);
    except ValueError:
        print('Skipping message because it is not for a known shape');
        

### --------------------------------------------------------------- MAIN
        
if __name__ == "__main__":

    #parse arguments
    import argparse
    parser = argparse.ArgumentParser(description='Publish shapes on the \
    /shapes_to_draw topic and adapt them based on feedback received on the /shape_feedback topic');
    parser.add_argument('word', action="store",
                    help='a string containing the letters to be learnt');
    parser.add_argument('--show', action='store_true', help='display plots of the shapes');

    args = parser.parse_args();
    
    if(args.show):
        plt.ion(); #to plot one shape at a time
    
    #start learning
    shapeLearners = initialiseShapeLearners(args.word); 

    #subscribe to feedback topic with a feedback manager which will pass messages to appropriate shapeLearners
    feedback_subscriber = rospy.Subscriber(FEEDBACK_TOPIC, String, feedbackManager);
    rospy.spin();
