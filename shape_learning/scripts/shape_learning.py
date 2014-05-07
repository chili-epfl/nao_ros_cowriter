#!/usr/bin/env python

"""
Publishes shapes specified in 'word' argument on the /shapes_to_draw 
topic and adapts them based on feedback received on the /shape_feedback 
topic. Feedback messages should be formatted as shapeTypeIndex_bestShapeIndex
eg. '0_2' for the first letter's 3rd version being the best.
"""


import numpy
from scipy import interpolate
import math
import pdb
import matplotlib.pyplot as plt
import time
from enum import Enum 

from shape_learner_manager import ShapeLearnerManager
from shape_learner import SettingsStruct
from shape_modeler import ShapeModeler #for normaliseShapeHeight()

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from std_msgs.msg import String, Empty

#Nao parameters
NAO_IP = '192.168.1.10';
#NAO_IP = '127.0.0.1';#connect to webots simulator locally
naoConnected = False;
naoSpeaking = False;
naoWriting = False;
effector   = "RArm" #LArm or RArm

#shape learning parameters
simulatedFeedback = False;  #Simulate user feedback as whichever shape is closest to goal parameter value
boundExpandingAmount = 0.2; #How much to expand the previously-learnt parameter bounds by when the letter comes up again @TODO should be relative to parameter sensitivity
numItersBeforeConsideredStuck = 1; #After how long should we consider that the user is stuck in a sub-optimal convergence?
class LearningModes(Enum):
    alwaysGood = 0;
    startsBad = 1;
    alwaysBad = 2;
    startsRandom = 3;
#define the learning mode for each shape we expect to see
learningModes = {'c': LearningModes.startsRandom,
                'e': LearningModes.startsRandom,
                'm': LearningModes.startsRandom,
                'n': LearningModes.startsRandom,
                'o': LearningModes.startsRandom,
                's': LearningModes.startsRandom,
                'u': LearningModes.startsRandom,
                'w': LearningModes.startsRandom}; 


#trajectory publishing parameters
FRAME = 'writing_surface';  #Frame ID to publish points in
FEEDBACK_TOPIC = 'shape_feedback'; #Name of topic to receive feedback on
SHAPE_TOPIC = 'write_traj'; #Name of topic to publish shapes to
if(naoWriting):
    t0 = 3;                 #Time allowed for the first point in traj (seconds)
    dt = 0.35               #Seconds between points in traj
    delayBeforeExecuting = 3;#How far in future to request the traj be executed (to account for transmission delays and preparedness)
elif(naoConnected):
    t0 = 0.05;
    dt = 0.1;
    delayBeforeExecuting = 3.5;
else:
    t0 = 0.05;
    dt = 0.1;
    delayBeforeExecuting = 3.5;
sizeScale_height = 0.035;    #Desired height of shape (metres)
sizeScale_width = 0.023;     #Desired width of shape (metres)
numDesiredShapePoints = 15.0;#Number of points to downsample the length of shapes to (not guaranteed)
numPoints_shapeModeler = 70; #Number of points used by ShapeModelers

#tablet parameters
tabletConnected = True;      #If true, will wait for shape_finished notification before proceeding to the next shape (rather than a fixed delay)
minTimeBetweenTouches = 0.1  #Seconds allowed between touches for the second one to be considered

CLEAR_SCREEN_TOPIC = 'clear_screen';
SHAPE_FINISHED_TOPIC = 'shape_finished';
USER_DRAW_SHAPES_TOPIC = 'user_shapes';

WORDS_TOPIC = 'words_to_write';
TEST_TOPIC = 'test_learning';#Listen for when test card has been shown to the robot
STOP_TOPIC = 'stop_learning';#Listen for when stop card has been shown to the robot

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
def read_traj_msg(message):
    x_shape = [];
    y_shape = [];
    for poseStamped in message.poses:
        x_shape.append(poseStamped.pose.position.x);
        y_shape.append(-poseStamped.pose.position.y);
        
    numPointsInShape = len(x_shape); 
    #make shape have the same number of points as the shape_modeler
    t_current = numpy.linspace(0, 1, numPointsInShape);
    t_desired = numpy.linspace(0, 1, numPoints_shapeModeler);
    f = interpolate.interp1d(t_current, x_shape, kind='cubic');
    x_shape = f(t_desired);
    f = interpolate.interp1d(t_current, y_shape, kind='cubic');
    y_shape = f(t_desired);
       
    shape = [];
    shape[0:numPoints_shapeModeler] = x_shape;
    shape[numPoints_shapeModeler:] = y_shape;
    shape = ShapeModeler.normaliseShapeHeight(numpy.array(shape));
    if(args.show):
        plt.figure(1);
        ShapeModeler.normaliseAndShowShape(shape);
            
            
def make_traj_msg(shape, shapeCentre, headerString):      
    
    traj = Path();
    traj.header.frame_id = headerString;
    traj.header.stamp = rospy.Time.now()+rospy.Duration(delayBeforeExecuting);
    shape = ShapeModeler.normaliseShapeHeight(shape);
    numPointsInShape = len(shape)/2;   
    
    x_shape = shape[0:numPointsInShape];
    y_shape = shape[numPointsInShape:];

    #make shape have the same number of points as the shape_modeler
    t_current = numpy.linspace(0, 1, numPointsInShape);
    t_desired = numpy.linspace(0, 1, numDesiredShapePoints);
    f = interpolate.interp1d(t_current, x_shape[:,0], kind='cubic');
    x_shape = f(t_desired);
    f = interpolate.interp1d(t_current, y_shape[:,0], kind='cubic');
    y_shape = f(t_desired);

    numPointsInShape = len(x_shape);
    
    for i in range(numPointsInShape):
        point = PoseStamped();
        point.pose.position.x = x_shape[i]*sizeScale_width;
        point.pose.position.y = -y_shape[i]*sizeScale_height;
        
        point.pose.position.x+= + shapeCentre[0];
        point.pose.position.y+= + shapeCentre[1];
        
        point.header.frame_id = FRAME;
        point.header.stamp = rospy.Time(t0+i*dt); #@todo allow for variable time between points for now
        traj.poses.append(point);

    return traj
    
###
            
         
###---------------------------------------------- WORD LEARNING SETTINGS

def generateSettings(shapeType):
    paramToVary = 2;            #Natural number between 1 and numPrincipleComponents, representing which principle component to vary from the template
    initialBounds_stdDevMultiples = [-6, 6];  #Starting bounds for paramToVary, as multiples of the parameter's observed standard deviation in the dataset
    doGroupwiseComparison = True; #instead of pairwise comparison with most recent two shapes
    initialParamValue = numpy.NaN;
    initialBounds = [numpy.NaN, numpy.NaN];
    
    if shapeType == 'a':
        paramToVary = 6;
        initialBounds_stdDevMultiples = [-5, 5];
        datasetFile = '../res/a_noHook_dataset.txt';
    elif shapeType == 'c':
        paramToVary = 4;
        initialBounds_stdDevMultiples = [-10, 10];
        datasetFile = '../res/c_dataset.txt';
        
        if(learningModes[shapeType] == LearningModes.startsRandom):
            initialBounds_stdDevMultiples = [-10, 10];
        elif(learningModes[shapeType] == LearningModes.alwaysGood):
            initialBounds_stdDevMultiples = [-4, -3];
            initialParamValue = -0.5;
        elif(learningModes[shapeType] == LearningModes.alwaysBad):
            initialBounds_stdDevMultiples = [1.5, 10];
            initialParamValue = 0.5;
        elif(learningModes[shapeType] == LearningModes.startsBad):
            initialBounds_stdDevMultiples = [-10, 10];
            initialParamValue = 0.5; 
    elif shapeType == 'd':
        datasetFile = '../res/d_cursive_dataset.txt';
    elif shapeType == 'e':
        paramToVary = 3; 
        initialBounds_stdDevMultiples = [-6, 14];
        datasetFile = '../res/e_dataset.txt';
        initialParamValue = 0.8;
    elif shapeType == 'm':
        paramToVary = 6; 
        initialBounds_stdDevMultiples = [-10, -6];
        datasetFile = '../res/m_dataset.txt';
        initialParamValue = -0.5;#0.0;
    elif shapeType == 'n':
        paramToVary = 7; 
        datasetFile = '../res/n_dataset.txt';
        initialParamValue = 0.0;
    elif shapeType == 'o':
        paramToVary = 4;
        initialBounds_stdDevMultiples = [-3.5, 3];
        datasetFile = '../res/o_dataset.txt';
    elif shapeType == 'r':
        paramToVary = 1;
        datasetFile = '../res/r_print_dataset.txt';
    elif shapeType == 's':
        datasetFile = '../res/s_print_dataset.txt';
    elif shapeType == 'u':
        paramToVary = 3;
        datasetFile = '../res/u_dataset.txt';
    elif shapeType == 'v':
        paramToVary = 6;
        datasetFile = '../res/v_dataset.txt';
    elif shapeType == 'w':
        datasetFile = '../res/w_dataset.txt';
    else:
        raise RuntimeError("Dataset is not known for shape "+ shapeType);
    settings = SettingsStruct(shape_learning = shapeType,
    paramToVary = paramToVary, doGroupwiseComparison = True, 
    datasetFile = datasetFile, initialBounds = initialBounds, 
    initialBounds_stdDevMultiples = initialBounds_stdDevMultiples,
    initialParamValue = initialParamValue, minParamDiff = 0.4);
    return settings
    
def lookAtShape(traj):
    trajStartPosition = traj.poses[0].pose.position;
    trajStartPosition_robot = tl.transformPose("base_footprint",target)
    rospy.sleep(2.0);
    nao.look_at([trajStartPosition.x,trajStartPosition.y,trajStartPosition.z,target_frame]); #look at shape again    
    
def lookAndAskForFeedback(toSay):
    if(naoWriting):
        #put arm down
        nao.execute([naoqi_request("motion","angleInterpolationWithSpeed",["RArm",joints_standInit,0.2])])
    
    if(effector=="RArm"):   #person will be on our right
        nao.look_at([0.3,-0.1,0.5,"base_link"]);
    else:                   #person will be on our left
        nao.look_at([0.3,0.1,0.5,"base_link"]);  
        
    if(naoSpeaking):
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
               
### ----------------------------------------- PUBLISH SIMULATED FEEDBACK    
def publishSimulatedFeedback(bestShape_index, shapeType_index, doGroupwiseComparison):           
        feedback = String();
        if(doGroupwiseComparison):
            feedback.data = str(shapeType_index) + '_' + str(bestShape_index);
        else:
            names = ('old', 'new');
            feedback.data = names[bestShape_index];            
        feedbackManager(feedback);
        
### ------------------------------------------------------ PUBLISH SHAPE        
def publishShapeAndWaitForFeedback(shape, shapeType, shapeType_code, param, paramValue):
    trajStartPosition = Point();
    if(simulatedFeedback):
        if(args.show):
            plt.figure(1);
            ShapeModeler.normaliseAndShowShape(shape);
            time.sleep(1.3); 
    else:
       
        try:
            display_new_shape = rospy.ServiceProxy('display_new_shape', displayNewShape);
            response = display_new_shape(shape_type_code = shapeType_code);
            shapeCentre = numpy.array([response.location.x, response.location.y]);
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        headerString = shapeType+'_'+str(param)+'_'+str(paramValue);
        traj = make_traj_msg(shape, shapeCentre, headerString);
        if(naoConnected):
            trajStartPosition = traj.poses[0].pose.position;
            nao.look_at([trajStartPosition.x,trajStartPosition.y,trajStartPosition.z,FRAME]); #look at shape        
        pub_traj.publish(traj);   
    
    return trajStartPosition
            
def feedbackManager(stringReceived):
    global shapeFinished #todo: make class attribute
    
    #convert feedback string into settings
    feedback = stringReceived.data.split('_');
    processMessage = True;
    try:
        shapeIndex_messageFor = int(feedback[0]);
    except:
        print('Shape type index must be an integer. Received ' + feedback[0]);
        processMessage = False;
        
    try:
        bestShape_index = int(feedback[1]);
    except:
        print('Best shape index must be an integer. Received ' + feedback[0]);
        processMessage = False;
    
    noNewShape = False; #usually make a new shape based on feedback
    if(len(feedback)>2): 
        feedbackMessage = feedback[2];
        if(feedbackMessage == 'noNewShape'):
            noNewShape = True;
        else:
            processMessage = False;
            print('Unknown message received in feedback string: '+feedbackMessage);   
                
    if(processMessage):    
        if(noNewShape): #just respond to feedback, don't make new shape 
            if(naoConnected):
                toSay = 'Ok, thanks for helping me';
                print('NAO: '+toSay);
                textToSpeech.say(toSay); 
            #pass feedback to shape manager
            response = wordManager.feedbackManager(shapeIndex_messageFor, bestShape_index, noNewShape);
            if(response == -1):
                print('Something\'s gone wrong');
            
        else:
            if(naoConnected):
                toSay = 'Ok, I\'ll work on the '+shape_messageFor;
                print('NAO: '+toSay);
                textToSpeech.say(toSay);
                #nao.setpose("StandInit")
                rospy.sleep(0.4);
                #nao.execute([naoqi_request("motion","wbEnableEffectorControl",[effector,True])])
            
            [numItersConverged, newShape, shapeType, shapeType_code, param, paramValue] = wordManager.feedbackManager(shapeIndex_messageFor, bestShape_index, noNewShape);
                       
            centre = publishShapeAndWaitForFeedback(newShape, shapeType, shapeType_code, param, paramValue);
            if(simulatedFeedback and numItersConverged == 0):
                bestShape_index = wordManager.generateSimulatedFeedback(shapeIndex_messageFor, newShape, paramValue);
                publishSimulatedFeedback(bestShape_index, shapeIndex_messageFor,True);

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
                
            if(numItersConverged>0):
                print("I can\'t make anymore different shapes (converged for " + str(numItersConverged) + "iterations)");
                
            if(numItersConverged >= numItersBeforeConsideredStuck):
                print("I think I'm stuck...");
                if(naoSpeaking):
                    textToSpeech.say("I\'m not sure I understand. Let\'s try again.");
                
                #change bounds back to the initial ones to hopefully get un-stuck
                wordManager.resetParameterBounds(shapeIndex_messageFor);
                
            else:
                if(naoConnected):
                    lookAndAskForFeedback("How about now?");
                    rospy.sleep(0.7);
                    nao.look_at([centre.x,centre.y,centre.z,FRAME]); #look at shape again    
                    nao.look_at([centre.x,centre.y,centre.z,FRAME]); #look at shape again   

        
def wordMessageManager(message):
    global shapeFinished, wordManager #@todo make class attribute 
    
    wordToLearn = message.data;
    wordSeenBefore = wordManager.newCollection(wordToLearn);
    if(naoConnected):
        if(wordSeenBefore):
            toSay = wordToLearn+' again, ok.';
        else:
            toSay = wordToLearn+', alright.';
    
        print('NAO: '+toSay);
        textToSpeech.say(toSay);    
        
    #clear screen
    pub_clear.publish(Empty());
    rospy.sleep(0.5);
    
    #start learning        
    for i in range(len(wordToLearn)):
        [shape, shapeType, shapeType_code, paramToVary, paramValue] = wordManager.startNextShapeLearner();
        print('Sending '+shapeType);
        centre = publishShapeAndWaitForFeedback(shape, shapeType, shapeType_code, paramToVary, paramValue);
        if(simulatedFeedback): #pretend first one isn't good enough
            publishSimulatedFeedback(0,shapeType_code,True); 

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
    

def testManager(message):
    if(naoSpeaking):
        textToSpeech.say('Ok, test time. I\'ll try my best.');

def stopManager(message):
    if(naoSpeaking):
        textToSpeech.say('Thank you for your help.');       
          
def clearScreenManager(message):
    print('Clearing display');
    try:
        clear_all_shapes = rospy.ServiceProxy('clear_all_shapes', clearAllShapes);
        resp1 = clear_all_shapes();
        print( resp1.success);
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
### --------------------------------------------------------------- MAIN
shapesLearnt = [];
wordsLearnt = [];
shapeLearners = [];
currentWord = [];
settings_shapeLearners = [];

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
         
    #subscribe to feedback topic with a feedback manager which will pass messages to appropriate shapeLearners
    feedback_subscriber = rospy.Subscriber(FEEDBACK_TOPIC, String, feedbackManager);

    #listen for words to write
    words_subscriber = rospy.Subscriber(WORDS_TOPIC, String, wordMessageManager);
    
    #listen for request to clear screen (from tablet)
    clear_subscriber = rospy.Subscriber(CLEAR_SCREEN_TOPIC, Empty, clearScreenManager);
    
    #listen for test time
    test_subscriber = rospy.Subscriber(TEST_TOPIC, Empty, testManager);
    
    #listen for when to stop
    stop_subscriber = rospy.Subscriber(STOP_TOPIC, Empty, stopManager); 
    
    #listen for user-drawn shapes
    shape_subscriber = rospy.Subscriber(USER_DRAW_SHAPES_TOPIC, Path, read_traj_msg); 
        
    #initialise display manager for shapes (manages positioning of shapes)
    from display_manager.srv import *
    rospy.wait_for_service('clear_all_shapes');
    
    rospy.sleep(1.0);   #Allow some time for the subscribers to do their thing, 
                        #or the first message will be missed (eg. first traj on tablet, first clear request locally)

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
        textToSpeech.setVolume(0.2);
        if(naoWriting):
            nao.setpose("StandInit");
            [temp,joints_standInit] = nao.execute([naoqi_request("motion","getAngles",["RArm",True])]);
            nao.execute([naoqi_request("motion","wbEnableEffectorControl",[effector,True])])
            
    #initialise word manager (passes feedback to shape learners and keeps history of words learnt)
    wordManager = ShapeLearnerManager(generateSettings);
            
    wordToLearn = args.word;
    if(wordToLearn is not None):
        message = String();
        message.data = wordToLearn;
        wordMessageManager(message);
    else:
        print('Waiting for word to write');

    rospy.spin();
