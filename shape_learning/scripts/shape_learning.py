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
t0 = 3;                   #Time allowed for the first point in traj (seconds)
dt = 0.35                   #Seconds between points in traj
delayBeforeExecuting = 3; #How far in future to request the traj be executed (to account for transmission delays and preparedness)
sizeScale = 0.04;           #Desired max dimension of shape (metres)
numDesiredShapePoints = 15.0; #Number of points to downsample the length of shapes to (not guaranteed)
tabletConnected = True;

TOUCH_TOPIC = 'touch_info';
CLEAR_SCREEN_TOPIC = 'clear_screen';
WORDS_TOPIC = 'words_to_write';
SHAPE_FINISHED_TOPIC = 'shape_finished';
GESTURE_TOPIC = 'long_touch_info'; #topic for location of 'shape good enough' gesture

#Nao parameters
NAO_IP = '192.168.1.10';
#NAO_IP = '127.0.0.1';#connect to webots simulator locally
naoConnected = True;
effector   = "RArm" #LArm or RArm

pub_traj = rospy.Publisher(SHAPE_TOPIC, Path);
pub_clear = rospy.Publisher(CLEAR_SCREEN_TOPIC, Empty);

phrases_askingForFeedback = {"Any better?","How about now?"};
phrases_actingOnFeedback_drawAgain = {"Ok, I\'ll work on the ","The "};
 
if(naoConnected):
    import robots
    from robots import naoqi_request
    nao = robots.Nao(ros=True, host=NAO_IP);
else:
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
    
    
def make_traj_msg(shape, shapeCentre, headerString):      
    
    traj = Path();
    traj.header.frame_id = headerString;
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
        paramToVary = 3;
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
        paramToVary = 3;
        datasetFile = '../res/o_dataset.txt';
    elif shapeType == 's':
        datasetFile = '../res/s_print_dataset.txt';
    elif shapeType == 'u':
        paramToVary = 4;
        datasetFile = '../res/u_dataset.txt';
    elif shapeType == 'w':
        datasetFile = '../res/w_dataset.txt';
    else:
        raise RuntimeError("Dataset is not known for shape "+ shapeType);
    settings = ShapeLearner.SettingsStruct(shape_learning = shapeType,
    paramToVary = paramToVary, doGroupwiseComparison = True, initialBounds = [], minParamDiff = 0.2);
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
    
def lookAtShape(traj):
    trajStartPosition = traj.poses[0].pose.position;
    trajStartPosition_robot = tl.transformPose("base_footprint",target)
    rospy.sleep(2.0);
    nao.look_at([trajStartPosition.x,trajStartPosition.y,trajStartPosition.z,target_frame]); #look at shape again    
    
def lookAndAskForFeedback(toSay):
    #nao.execute([naoqi_request("motion","wbEnableEffectorControl",[effector,False])])

    #put arm down
    nao.execute([naoqi_request("motion","angleInterpolationWithSpeed",["RArm",joints_standInit,0.2])])
    #nao.setpose("StandInit");
    #rospy.sleep(1.0) #so doesn't jump
    
    if(effector=="RArm"):   #person will be on our right
        nao.look_at([0.1,-0.1,0,"gaze"]);
    else:                   #person will be on our left
        nao.look_at([0.1,0.1,0,"gaze"]);   

    textToSpeech.say(toSay);
    print('NAO: '+toSay);

def relax():    
    pNames = "LArm"
    pStiffnessLists = 0.0
    pTimeLists = 1.0
    nao.execute([naoqi_request("motion","stiffnessInterpolation",[pNames, pStiffnessLists, pTimeLists])]);
    
    pNames = "RArm"
    nao.execute([naoqi_request("motion","stiffnessInterpolation",[pNames, pStiffnessLists, pTimeLists])]);

    pNames = "Head"
    nao.execute([naoqi_request("motion","stiffnessInterpolation",[pNames, pStiffnessLists, pTimeLists])]);    
    
def onShapeFinished(message):
    global shapeFinished;
    shapeFinished = True;
        
def startShapeLearners(wordToLearn):
    global shapesLearnt, shapeLearners, settings_shapeLearners, shapeFinished
    centre = [];
    if(naoConnected):
        #nao.setpose("StandInit")
        #nao.execute([naoqi_request("motion","wbEnableEffectorControl",[effector,True])])
        #rospy.sleep(0.3);
        pass
        
    #start learning        
    for i in range(len(wordToLearn)):
        shapeType = wordToLearn[i];
        print('Sending '+shapeType);
        shape_index = shapesLearnt.index(shapeType);
        [shape, paramValue] = shapeLearners[shape_index].startLearning(settings_shapeLearners[shape_index].initialBounds);
        
        centre = publishShapeAndWaitForFeedback(shape,shapeType, settings_shapeLearners[shape_index].paramToVary, paramValue);
        if(simulatedFeedback): #pretend first one isn't good enough
            publishSimulatedFeedback(0,shapeType,settings_shapeLearners[shape_index].doGroupwiseComparison); 

        elif(not tabletConnected):
            rospy.sleep(10);
            print('Shape finished');
        else:
            rospy.sleep(0.1);
            #listen for notification that the letter is finished
            shape_finished_subscriber = rospy.Subscriber(SHAPE_FINISHED_TOPIC, String, onShapeFinished);
            while(not shapeFinished):
                rospy.sleep(0.1);
            shape_finished_subscriber.unregister();
            shapeFinished = False;
            print('Shape finished.');
            
    if(naoConnected):
        lookAndAskForFeedback("What do you think?");
        rospy.sleep(0.7);
        nao.look_at([centre.x,centre.y,centre.z,FRAME]); #look at shape again    
        nao.look_at([centre.x,centre.y,centre.z,FRAME]); #look at shape again (bug in pyrobots)

     
        
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
def publishShapeAndWaitForFeedback(shape, shapeType, param, paramValue):
    trajStartPosition = Point();
    if(simulatedFeedback):
        if(args.show):
            plt.figure(1);
            ShapeModeler.normaliseAndShowShape(shape);
            time.sleep(1.3); 
    else:
        
        shapeCentre = getPositionToDrawAt(shapeType);
        headerString = shapeType+'_'+str(param)+'_'+str(paramValue);
        traj = make_traj_msg(shape, shapeCentre, headerString);
        if(naoConnected):
            trajStartPosition = traj.poses[0].pose.position;
            nao.look_at([trajStartPosition.x,trajStartPosition.y,trajStartPosition.z,FRAME]); #look at shape        
        pub_traj.publish(traj);   
    
    return trajStartPosition
            
def feedbackManager(stringReceived):
    global shapeFinished
    #todo: make class attribute
    feedback = stringReceived.data.split('_');
    shape_messageFor = feedback[0];
    feedbackData = feedback[1];
    bestShape_index = int(feedbackData);
    
    
    try:
        shapeIndex_messageFor = shapesLearnt.index(shape_messageFor);
        processMessage = True;
    except ValueError:
        processMessage = False;
    
    noNewShape = False; #usually make a new shape based on feedback
    
    if(len(feedback)>2): 
        feedbackMessage = feedback[2];
        if(feedbackMessage == 'noNewShape'):
            noNewShape = True;
        else:
            processMessage = False;
            print('Unknown message received in feedback string');   
         
    if(processMessage):    
        if(noNewShape): #just respond to feedback, don't make new shape 
            if(naoConnected):
                toSay = 'Ok, thanks for helping me';
                print('NAO: '+toSay);
                textToSpeech.say(toSay); 
            shapeLearners[shapeIndex_messageFor].respondToFeedback(bestShape_index);
            
        else:
            if(naoConnected):
                toSay = 'Ok, I\'ll work on the '+shape_messageFor;
                print('NAO: '+toSay);
                textToSpeech.say(toSay);
                #nao.setpose("StandInit")
                rospy.sleep(0.4);
                #nao.execute([naoqi_request("motion","wbEnableEffectorControl",[effector,True])])

            [converged, newShape, newParamValue] = shapeLearners[shapeIndex_messageFor].generateNewShapeGivenFeedback(bestShape_index);
            
            if(not converged):
                centre = publishShapeAndWaitForFeedback(newShape, shape_messageFor, settings_shapeLearners[shapeIndex_messageFor].paramToVary, newParamValue);
                if(simulatedFeedback):
                    bestShape_index = shapeLearners[shapeIndex_messageFor].generateSimulatedFeedback(newShape, newParamValue);
                    publishSimulatedFeedback(bestShape_index, shape_messageFor,settings_shapeLearners[shapeIndex_messageFor].doGroupwiseComparison);

                elif(not tabletConnected):
                    rospy.sleep(10);
                    print('Shape finished');
                    
                else: #wait for finished signal from tablet
                    rospy.sleep(0.1);
                    #listen for notification that the letter is finished
                    shape_finished_subscriber = rospy.Subscriber(SHAPE_FINISHED_TOPIC, String, onShapeFinished);
                    while(not shapeFinished):
                        rospy.sleep(0.1);
                    shape_finished_subscriber.unregister();
                    shapeFinished = False;
                    print('Shape finished.');
                    
                if(naoConnected):
                    lookAndAskForFeedback("How about now?");
                    rospy.sleep(0.7);
                    nao.look_at([centre.x,centre.y,centre.z,FRAME]); #look at shape again    
                    nao.look_at([centre.x,centre.y,centre.z,FRAME]); #look at shape again   
            
            else: 
                pass
    else:
        print('Skipping message because it is not for a known shape');
        
def touchInfoManager(pointStamped):
    touchLocation = numpy.array([pointStamped.point.x, pointStamped.point.y]);
    if(shapesDrawn is None):
        print('Ignoring touch because no shapes have been drawn');
    else:
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
        
def gestureManager(pointStamped):
    gestureLocation = numpy.array([pointStamped.point.x, pointStamped.point.y]);
    if(shapesDrawn is None):
        print('Ignoring touch because no shapes have been drawn');
    else:
        #map touch location to closest shape drawn
        touchCell = (gestureLocation - shapeSize/2)/shapeSize;
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

            else:
                print('Shape selected as best: '+shapeType+str(shapeID))
                feedbackMessage = String();
                feedbackMessage.data = shapeType + '_' + str(shapeID) + '_noNewShape';
                feedbackManager(feedbackMessage);
        except ValueError:    #@todo map to closest shape if appropriate
            print('Ignoring touch because it wasn''t on a valid shape');    
   
def wordMessageManager(message):
    global shapeLearners, settings_shapeLearners, shapesDrawn, currentWord, wordsLearnt #@todo make class attributes
    
    wordToLearn = message.data;
    currentWord = wordToLearn;
    
    try:
        word_index = wordsLearnt.index(currentWord);
        wordSeenBefore = True;
    except ValueError: 
        wordSeenBefore = False;
        wordsLearnt.append(currentWord);
   
    if(naoConnected):
        if(wordSeenBefore):
            toSay = currentWord+' again, ok.';
        else:
            toSay = currentWord+', alright.';
    
        print('NAO: '+toSay);
        textToSpeech.say(toSay);    
        
    #clear screen
    pub_clear.publish(Empty());

    #initialise shapes on screen
    shapesDrawn = numpy.ones((3,5,2))*numpy.NaN; #3rd dim: shapeType_code, ID
    
    [shapeLearners, settings_shapeLearners] = initialiseShapeLearners(currentWord); 
    startShapeLearners(currentWord);
        

### --------------------------------------------------------------- MAIN
shapesLearnt = [];
wordsLearnt = [];
shapeLearners = [];
currentWord = [];
settings_shapeLearners = [];
shapesDrawn = None;
shapeFinished = False;
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
    
    #listen for touch events on the tablet
    gesture_subscriber = rospy.Subscriber(GESTURE_TOPIC, PointStamped, gestureManager);
        
    #subscribe to feedback topic with a feedback manager which will pass messages to appropriate shapeLearners
    feedback_subscriber = rospy.Subscriber(FEEDBACK_TOPIC, String, feedbackManager);

    #listen for words to write
    words_subscriber = rospy.Subscriber(WORDS_TOPIC, String, wordMessageManager);
    
    if(naoConnected):
        from naoqi import ALBroker, ALProxy
        #start speech (ROS isn't working..)
        port = 9559;
        myBroker = ALBroker("myBroker", #I'm not sure that pyrobots doesn't already have one of these open called NAOqi?
            "0.0.0.0",   # listen to anyone
            0,           # find a free port and use it
            NAO_IP,      # parent broker IP
            port)        # parent broker port
        textToSpeech = ALProxy("ALTextToSpeech", NAO_IP, port)   
        textToSpeech.setLanguage('English')
        nao.setpose("StandInit");
        [temp,joints_standInit] = nao.execute([naoqi_request("motion","getAngles",["RArm",True])]);
        nao.execute([naoqi_request("motion","wbEnableEffectorControl",[effector,True])])
    wordToLearn = args.word;
    if(wordToLearn is not None):
        message = String();
        message.data = wordToLearn;
        wordMessageManager(message);
    else:
        print('Waiting for word to write');

    rospy.spin();
