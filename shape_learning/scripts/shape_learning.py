#!/usr/bin/env python

"""
Publishes shapes specified in 'word' argument on the /shapes_to_draw 
topic and adapts them based on feedback received on the /shape_feedback 
topic. Feedback messages should be formatted as shapeName_bestShapeIndex
eg. 'a_2'.
"""


import numpy
import math
import pdb
import matplotlib.pyplot as plt
import time
from shape_modeler import ShapeModeler
from shape_learner import ShapeLearner

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from std_msgs.msg import String, Empty

#shape learning parameters
numPrincipleComponents = 5; #Number of principle components to keep during PCA of dataset
simulatedFeedback = False;  #Simulate user feedback as whichever shape is closest to goal parameter value
boundExpandingAmount = 0.2; #How much to expand the previously-learnt parameter bounds by when the letter comes up again

#trajectory publishing parameters
FRAME = 'writing_surface';  #Frame ID to publish points in
FEEDBACK_TOPIC = 'shape_feedback'; #Name of topic to receive feedback on
SHAPE_TOPIC = 'write_traj'; #Name of topic to publish shapes to
t0 = 1.5;                   #Time allowed for the first point in traj (seconds)
dt = 0.35                   #Seconds between points in traj
delayBeforeExecuting = 1.5; #How far in future to request the traj be executed (to account for transmission delays and preparedness)
sizeScale = 0.04;           #Desired max dimension of shape (metres)
numDesiredShapePoints = 15.0; #Number of points to downsample the length of shapes to (not guaranteed)
TOUCH_TOPIC = 'touch_info';
CLEAR_SCREEN_TOPIC = 'clear_screen';
WORDS_TOPIC = 'words_to_write';
pub_traj = rospy.Publisher(SHAPE_TOPIC, Path);
pub_clear = rospy.Publisher(CLEAR_SCREEN_TOPIC, Empty);

rospy.init_node("shape_learner");

### ------------------------------------------------------ MESSAGE MAKER
shapeWidth = 0.05;
shapeHeight = 0.044;
shapeSize = numpy.array([shapeWidth,shapeHeight]);
def getPositionToDrawAt(shapeType):
    global currentWord
    offset = currentWord.index(shapeType);
    col = 1 + offset;
    
    shapeType_code = currentWord.index(shapeType);
    shapeID = numpy.equal(shapesDrawn[:,:,0],shapeType_code).sum();
    row = shapeID;
    shapesDrawn[row,col,0] = shapeType_code;
    shapesDrawn[row,col,1] = shapeID;
    
    numRows = shapesDrawn.shape[0];
    position = [(col+0.5)*shapeWidth,((numRows-1)-row+0.5)*shapeHeight];
    return position;
    
from numpy import mean;
def downsample_1d(myarr,factor,estimator=mean):
    """
    FROM http://code.google.com/p/agpy/source/browse/trunk/AG_image_tools/downsample.py?r=452
    Downsample a 1D array by averaging over *factor* pixels.
    Crops right side if the shape is not a multiple of factor.

    This code is pure numpy and should be fast.

    keywords:
        estimator - default to mean.  You can downsample by summing or
            something else if you want a different estimator
            (e.g., downsampling error: you want to sum & divide by sqrt(n))
    """
    #xs = myarr.shape
    xs = len(myarr);
    factor = int(factor);
    crarr = myarr[:xs-(xs % factor)]
    dsarr = estimator( numpy.concatenate([[crarr[i::factor]
        for i in range(factor)] ]),axis=0)
    return dsarr
    
    
def make_traj_msg(shape, shapeCentre):      
    
    traj = Path();
    traj.header.frame_id = FRAME;
    traj.header.stamp = rospy.Time.now()+rospy.Duration(delayBeforeExecuting);
    shape = ShapeModeler.normaliseShapeHeight(shape);
    numPointsInShape = len(shape)/2;   
    
    x_shape = shape[0:numPointsInShape];
    y_shape = shape[numPointsInShape:];
    
    #downsample shape
    downsampleFactor = math.ceil(numPointsInShape/numDesiredShapePoints);#determine appropriate factor for downsampling
    x_shape = downsample_1d(x_shape,downsampleFactor);
    y_shape = downsample_1d(y_shape,downsampleFactor);
    numPointsInShape = len(x_shape);
    
    for i in range(numPointsInShape):
        point = PoseStamped();
        point.pose.position.x = x_shape[i,0]*sizeScale;
        point.pose.position.y = -y_shape[i,0]*sizeScale;
        
        point.pose.position.x+= + shapeCentre[0];
        point.pose.position.y+= + shapeCentre[1];
        
        point.header.frame_id = FRAME;
        point.header.stamp = rospy.Time(t0+i*dt); #@todo allow for variable time between points for now
        traj.poses.append(point);

    return traj
    
###
            
         
###--------------------------------------------- WORD LEARNING FUNCTIONS
# @todo make methods of a class

def generateSettings(shapeType):
    paramToVary = 2;            #Natural number between 1 and numPrincipleComponents, representing which principle component to vary from the template
    initialBounds_stdDevMultiples = [-6, 6];  #Starting bounds for paramToVary, as multiples of the parameter's observed standard deviation in the dataset
    doGroupwiseComparison = True; #instead of pairwise comparison with most recent two shapes
    
    if shapeType == 'c':
        datasetFile = '../res/c_dataset.txt';
    elif shapeType == 'd':
        datasetFile = '../res/d_cursive_dataset.txt';
    elif shapeType == 'e':
        datasetFile = '../res/e_dataset.txt';
    elif shapeType == 'm':
        datasetFile = '../res/m_dataset.txt';
    elif shapeType == 'n':
        datasetFile = '../res/n_dataset.txt';
    elif shapeType == 'o':
        datasetFile = '../res/o_dataset.txt';
    elif shapeType == 's':
        datasetFile = '../res/s_print_dataset.txt';
    elif shapeType == 'u':
        datasetFile = '../res/u_dataset.txt';
    elif shapeType == 'w':
        datasetFile = '../res/w_dataset.txt';
    else:
        raise RuntimeError("Dataset is not known for shape "+ shapeType);
    settings = ShapeLearner.SettingsStruct(shape_learning = shapeType,
    paramToVary = 2, doGroupwiseComparison = True, initialBounds = [], minParamDiff = 0.2);
    return settings, datasetFile, initialBounds_stdDevMultiples
    
def initialiseShapeLearners(wordToLearn):
    global shapesLearnt, shapeLearners, settings_shapeLearners
    for i in range(len(wordToLearn)):
        shapeType = wordToLearn[i];
        
        #check if shape has been learnt before
        try:
            shapeType_index = shapesLearnt.index(shapeType);
            newShape = False;
        except ValueError: 
            newShape = True;
        
        if(newShape):
            [settings, datasetFile, initialBounds_stdDevMultiples] = generateSettings(shapeType); 

            #analyse database of shapes
            shapeModeler = ShapeModeler();
            shapeModeler.makeDataMatrix(datasetFile);
            shapeModeler.performPCA(numPrincipleComponents);
            
            #learn parameter of shape
            parameterVariances = shapeModeler.getParameterVariances();
            settings.initialBounds = numpy.array(initialBounds_stdDevMultiples)*parameterVariances[settings.paramToVary-1];  
            
            shapeLearner = ShapeLearner(shapeModeler,settings);
            shapesLearnt.append(shapeType);
            shapeLearners.append(shapeLearner);
            settings_shapeLearners.append(settings);
        else:
            #use the bounds determined last time
            previousBounds = shapeLearners[shapeType_index].getParameterBounds();
            newInitialBounds = previousBounds;
            newInitialBounds[0] -= boundExpandingAmount;
            newInitialBounds[1] += boundExpandingAmount;
            settings_shapeLearners[shapeType_index].initialBounds = newInitialBounds;
    return shapeLearners, settings_shapeLearners;

def startShapeLearners(wordToLearn):
    global shapesLearnt, shapeLearners, settings_shapeLearners

    #start learning        
    for i in range(len(wordToLearn)):
        shapeType = wordToLearn[i];
        shape_index = shapesLearnt.index(shapeType);
        [shape, paramValue] = shapeLearners[shape_index].startLearning(settings_shapeLearners[shape_index].initialBounds);
        publishShapeAndWaitForFeedback(shape,shapeType);
        if(simulatedFeedback): #pretend first one isn't good enough
            publishSimulatedFeedback(0,shapeType,settings_shapeLearners[shape_index].doGroupwiseComparison); 

        else:
            rospy.sleep(10.0);
        
### ----------------------------------------- PUBLISH SIMULATED FEEDBACK    
def publishSimulatedFeedback(bestShape_index, shapeType, doGroupwiseComparison):           
        feedback = String();
        if(doGroupwiseComparison):
            feedback.data = shapeType + '_' + str(bestShape_index);
        else:
            names = ('old', 'new');
            feedback.data = names[bestShape_index];            
        feedbackManager(feedback);
        
### ------------------------------------------------------ PUBLISH SHAPE        
def publishShapeAndWaitForFeedback(shape, shapeType):
    if(simulatedFeedback):
        if(args.show):
            plt.figure(1);
            ShapeModeler.normaliseAndShowShape(shape);
            time.sleep(1.3); 
    else:
        shapeCentre = getPositionToDrawAt(shapeType);
        traj = make_traj_msg(shape, shapeCentre);
        pub_traj.publish(traj);   
            
def feedbackManager(feedbackMessage):
    #todo: make class attribute
    feedback = feedbackMessage.data.split('_');
    shape_messageFor = feedback[0];
    feedbackData = feedback[1];
    bestShape_index = int(feedbackData);
    try:
        shapeIndex_messageFor = shapesLearnt.index(shape_messageFor);
        [converged, newShape, newParamValue] = shapeLearners[shapeIndex_messageFor].generateNewShapeGivenFeedback(bestShape_index);
        
        if(not converged):
            publishShapeAndWaitForFeedback(newShape,shape_messageFor);
            if(simulatedFeedback):
                bestShape_index = shapeLearners[shapeIndex_messageFor].generateSimulatedFeedback(newShape, newParamValue);
                publishSimulatedFeedback(bestShape_index, shape_messageFor,settings_shapeLearners[shapeIndex_messageFor].doGroupwiseComparison);
        else: 
            pass
    except ValueError:
        print('Skipping message because it is not for a known shape');
        
def touchInfoManager(pointStamped):
    touchLocation = numpy.array([pointStamped.point.x, pointStamped.point.y]);

    #map touch location to closest shape drawn
    touchCell = (touchLocation - shapeSize/2)/shapeSize;
    numRows = shapesDrawn.shape[0];
    row = (numRows -1)- int(round(touchCell[1]));
    col = int(round(touchCell[0]));
    
    try:
        shapeType_code = shapesDrawn[row,col,0];
        shapeType = currentWord[int(shapeType_code)];
        numShapes_shapeType = numpy.equal(shapesDrawn[:,:,0],shapeType_code).sum();
        shapeID = int(shapesDrawn[row,col,1]);
        
        if(shapeID > (numShapes_shapeType-1)): #touched where shape wasn't (wouldn't make it this far anyway)
            print('Ignoring touch because it wasn''t on a valid shape');

        elif(numShapes_shapeType==numRows):#last shape selected but no more space
            print('Can''t fit anymore letters on the screen');
        else:
            print('Shape touched: '+shapeType+str(shapeID))
            feedbackMessage = String();
            feedbackMessage.data = shapeType + '_' + str(shapeID);
            feedbackManager(feedbackMessage);
    except ValueError:    #@todo map to closest shape if appropriate
        print('Ignoring touch because it wasn''t on a valid shape');
        
    
   
def wordMessageManager(message):
    global shapeLearners, settings_shapeLearners, shapesDrawn, currentWord #@todo make class attributes
    
    wordToLearn = message.data;
    currentWord = wordToLearn;
        
    #clear screen
    pub_clear.publish(Empty());

    #initialise shapes on screen
    shapesDrawn = numpy.ones((3,5,2))*numpy.NaN; #3rd dim: shapeType_code, ID
    
    [shapeLearners, settings_shapeLearners] = initialiseShapeLearners(wordToLearn); 
    startShapeLearners(wordToLearn);
        

### --------------------------------------------------------------- MAIN
shapesLearnt = [];
shapeLearners = [];
currentWord = [];
settings_shapeLearners = [];
shapesDrawn = [];
if __name__ == "__main__":
    #parse arguments
    import argparse
    parser = argparse.ArgumentParser(description='Publish shapes on the \
    /shapes_to_draw topic and adapt them based on feedback received on the /shape_feedback topic');
    parser.add_argument('word', nargs='?', action="store",
                    help='a string containing the letters to be learnt (if not present, will wait for one from ROS topic)');
    parser.add_argument('--show', action='store_true', help='display plots of the shapes');

    args = parser.parse_args();
        
    if(args.show):
        plt.ion(); #to plot one shape at a time
    
    rospy.sleep(1.0); #maybe this helps the tablet not miss the first one?
    
    #listen for touch events on the tablet
    touch_subscriber = rospy.Subscriber(TOUCH_TOPIC, PointStamped, touchInfoManager);
        
    #subscribe to feedback topic with a feedback manager which will pass messages to appropriate shapeLearners
    feedback_subscriber = rospy.Subscriber(FEEDBACK_TOPIC, String, feedbackManager);

    #listen for words to write
    words_subscriber = rospy.Subscriber(WORDS_TOPIC, String, wordMessageManager);
    
    wordToLearn = args.word;
    if(wordToLearn is not None):
        message = String();
        message.data = wordToLearn;
        wordMessageManager(message);
    else:
        print('Waiting for word to write');

    rospy.spin();
