#!/usr/bin/env python
# coding: utf-8

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

from state_machine import StateMachine
from copy import deepcopy
drawingLetterSubstates = ['WAITING_FOR_ROBOT_TO_CONNECT', 'WAITING_FOR_TABLET_TO_CONNECT', 'PUBLISHING_LETTER'];

#Nao parameters
NAO_IP = '192.168.1.2';
#NAO_IP = '127.0.0.1';#connect to webots simulator locally
naoConnected = True;
naoSpeaking = True;
naoWriting = True;
effector   = "RArm" #LArm or RArm
naoLanguage = "French"; #'English' or 'French'

alternateSidesLookingAt = False; #if true, nao will look to a different side each time. 
global nextSideToLookAt
nextSideToLookAt = 'Right';

if(naoLanguage=='English'):
    introPhrase = "Hello. I'm Nao. Please show me a word to practice.";
    demo_responses = ["Okay, I'll try it like you", "So that's how you write %s", "That's a much better %s than mine", "I'll try to copy you","Let me try now","Thank you"];
    asking_phrases_after_feedback = ["Any better?", "How about now?", "Now what do you think?","Is there a difference?", "Is this one okay?", "Will you show me how?", "Did I improve?"];
    asking_phrases_after_word = ["Okay, what do you think?", "This is a hard word", "Is this how you write it?","Please help me"];
    word_responses = ["%s, okay. ", "%s seems like a good word", "Hopefully I can do well with this word", "%s. Let's try", "Okay, %s now"];
    word_responses_again = ["%s again, okay.", "I thought I already did %s", "You like to practice this word"];
    testPhrase = "Ok, test time. I'll try my best.";
    thankYouPhrase = 'Thank you for your help.';
elif(naoLanguage=='French'):
    introPhrase = "Bonjour, je m'appelle Nao. Peux-tu me montrer un mot ?";
    demo_responses = ["D'accord, j'essaye comme ça", "Ah, c'est comme ça qu'on écrit %s", "Ce %s est pas mal", "Bon, j'essaye comme toi", "Ok, à moi", "À mon tour", "Merci, je vais essayer"];
    asking_phrases_after_feedback = ["C'est mieux ?", "Et comme ça ?", "Tu en penses quoi ?", "Qu'est-ce que tu en penses ?", "Il y a une différence ?", "Ça va cette fois ?", "Je me suis amélioré ?", "Tu trouves que c'est mieux ?"];
    asking_phrases_after_word = ["Bon, qu'est ce que tu en penses ?", "Pas facile !", "C'est bien comme ça ?", "Je crois que j'ai besoin d'aide.", "Et voilà !"];
    word_responses = ["D'accord pour %s", "Ok, j'essaye %s", "Bon, je devrais y arriver", "D'accord", "%s ? ok"];
    word_responses_again = ["Encore %s ? bon, d'accord.", "Je crois que j'ai déjà fait %s", "On dirait que tu aimes bien %s !", "Encore ?"];
    testPhrase = "Ok, c'est l'heure du test. J'ai un peu peur."
    thankYouPhrase = "Merci pour ton aide !";
else:
    raise RuntimeError("Language not available");
    
demo_responses_counter = 0;
asking_phrases_after_feedback_counter = 0;
asking_phrases_after_word_counter = 0;
word_responses_counter = 0;
word_responses_again_counter = 0;
    
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
SHAPE_TOPIC_DOWNSAMPLED = 'write_traj_downsampled'; #Name of topic to publish shapes to
if(naoWriting):
    t0 = 2;                 #Time allowed for the first point in traj (seconds)
    dt = 0.25               #Seconds between points in traj
    delayBeforeExecuting = 3;#How far in future to request the traj be executed (to account for transmission delays and preparedness)
elif(naoConnected):
    t0 = 0.05;
    dt = 0.1;
    delayBeforeExecuting = 3.5;
else:
    t0 = 0.7;
    dt = 0.3;
    delayBeforeExecuting = 3.5;
sizeScale_height = 0.035;    #Desired height of shape (metres)
sizeScale_width = 0.023;     #Desired width of shape (metres)
numDesiredShapePoints = 7.0;#Number of points to downsample the length of shapes to 
numPoints_shapeModeler = 70; #Number of points used by ShapeModelers

#tablet parameters
tabletConnected = True;      #If true, will wait for shape_finished notification before proceeding to the next shape (rather than a fixed delay)
minTimeBetweenTouches = 0.1  #Seconds allowed between touches for the second one to be considered

CLEAR_SCREEN_TOPIC = 'clear_screen';
SHAPE_FINISHED_TOPIC = 'shape_finished';
USER_DRAWN_SHAPES_TOPIC = 'user_shapes';
GESTURE_TOPIC = 'gesture_info';

WORDS_TOPIC = 'words_to_write';
TEST_TOPIC = 'test_learning';#Listen for when test card has been shown to the robot
STOP_TOPIC = 'stop_learning';#Listen for when stop card has been shown to the robot
NEW_CHILD_TOPIC = 'new_child';

PUBLISH_STATUS_TOPIC = 'camera_publishing_status';
from std_msgs.msg import Bool
pub_camera_status = rospy.Publisher(PUBLISH_STATUS_TOPIC,Bool);

pub_traj = rospy.Publisher(SHAPE_TOPIC, Path);
pub_traj_downsampled = rospy.Publisher(SHAPE_TOPIC_DOWNSAMPLED, Path);
pub_clear = rospy.Publisher(CLEAR_SCREEN_TOPIC, Empty);

wordReceived = None;
if(naoConnected):
    import robots
    from robots import naoqi_request
    nao = robots.Nao(ros=True, host=NAO_IP);
else:
    rospy.init_node("shape_learner");

    
strokes=[];
def userShapePreprocessor(message):
    global strokes
    
    if(len(message.poses)==0):
        #decide what to do with strokes received
        length_longestStroke = 0;
        for stroke in strokes:
            if(stroke.shape[0]>length_longestStroke):
                longestStroke = stroke;
                length_longestStroke = stroke.shape[0];
        
        #tell how to interpret the shape intended for
        if(message._connection_header['callerid'] == '/child_tablet/interaction_manager'):
            positionToShapeMappingMethod = 'basedOnColumnOfScreen';
        else:#/android_gingerbread/interaction_manager'
            positionToShapeMappingMethod = 'basedOnClosestShapeToPosition';#'basedOnShapeAtPosition';
        onUserDrawnShapeReceived(longestStroke,positionToShapeMappingMethod); 
        strokes = [];
    else:
        print('Got stroke to write with '+str(len(message.poses))+' points');
        x_shape = [];
        y_shape = [];
        for poseStamped in message.poses:
            x_shape.append(poseStamped.pose.position.x);
            y_shape.append(-poseStamped.pose.position.y);
            
        numPointsInShape = len(x_shape); 

        shape = [];
        shape[0:numPointsInShape] = x_shape;
        shape[numPointsInShape:] = y_shape;
        
        shape = numpy.reshape(shape, (-1, 1)); #explicitly make it 2D array with only one column
        strokes.append(shape);


activeShapeForDemonstration_type = None;
def onSetActiveShapeGesture(message):
    global activeShapeForDemonstration_type
    
    gestureLocation = [message.point.x, message.point.y];
    #map gesture location to shape drawn
    try:
        shape_at_location = rospy.ServiceProxy('shape_at_location', shapeAtLocation);
        request = shapeAtLocationRequest();
        request.location.x = gestureLocation[0];
        request.location.y = gestureLocation[1];
        response = shape_at_location(request);
        shapeType_code = response.shape_type_code;
        shapeID = response.shape_id;
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
    if(shapeType_code != -1 and shapeID != -1):
        activeShapeForDemonstration_type = shapeType_code;
        print('Setting active shape to ' + wordManager.shapeAtIndexInCurrentCollection(activeShapeForDemonstration_type));

demoShapeReceived = None;
def onUserDrawnShapeReceived(path, positionToShapeMappingMethod):
    global demoShapeReceived, activeShapeForDemonstration_type
            
    location = ShapeModeler.getShapeCentre(path);
    try:
        if(positionToShapeMappingMethod == 'basedOnColumnOfScreen'):
            #map to shapelearner based on third of the screen demo was in
            if(location[0]<.21/3):
                shapeType_demoFor = 0;
            elif(location[0]>.21/3*2):
                shapeType_demoFor = 2;
            else:
                shapeType_demoFor = 1;
            
        elif( positionToShapeMappingMethod == 'basedOnShapeAtPosition'):
            shape_at_location = rospy.ServiceProxy('shape_at_location', shapeAtLocation);
            request = shapeAtLocationRequest();
            request.location.x = location[0];
            request.location.y = location[1];
            response = shape_at_location(request);
            shapeType_demoFor = response.shape_type_code;
            
        else:
            closest_shapes_to_location = rospy.ServiceProxy('closest_shapes_to_location', closestShapesToLocation);
            request = closestShapesToLocationRequest();
            request.location.x = location[0];
            request.location.y = location[1];
            response = closest_shapes_to_location(request);
            closestShapes_type = response.shape_type_code;
            if(len(closestShapes_type)>1 and activeShapeForDemonstration_type is not None):
                try: #see if active shape is in list
                    dummyIndex = closestShapes_type.index(activeShapeForDemonstration_type);
                    shapeType_demoFor = activeShapeForDemonstration_type;
                except ValueError: #just use first in list otherwise
                    shapeType_demoFor = closestShapes_type[0];
            else: #just use first in list
                shapeType_demoFor = closestShapes_type[0];
            
            #block the space from robot use
            try:
                display_shape_at_location = rospy.ServiceProxy('display_shape_at_location', displayShapeAtLocation);
                request = displayShapeAtLocationRequest();
                request.shape_type_code = shapeType_demoFor;
                request.location.x = location[0];
                request.location.y = location[1];
                #todo: allow for blocking different sized shapes
                response = display_shape_at_location(request);
                result = response.success;
                #todo: do something if unsuccessful
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
           
        '''
        index_of_location = rospy.ServiceProxy('index_of_location', indexOfLocation);
        request = indexOfLocationRequest();
        request.location.x = location[0];
        request.location.y = location[1];
        response = index_of_location(request);
        shapeIndex_demoFor = 0#response.row;
         '''
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    if(shapeType_demoFor == -1):# or response.shape_id == -1): 
        print("Ignoring demo because not for valid shape");
    else:
        if((stateMachine.get_state() == "WAITING_FOR_FEEDBACK" and demoShapeReceived is None)
        or (stateMachine.get_state() == "ASKING_FOR_FEEDBACK" and demoShapeReceived is None)): #only accept first stroke!
            demoShapeReceived = {'path': path, 'shapeType_code': shapeType_demoFor}; #replace any existing feedback with new
            print('Received demonstration');
            activeShapeForDemonstration_type = shapeType_demoFor;
        else:
            pass; #ignore feedback
    
def respondToDemonstration(infoFromPrevState):
    print('------------------------------------------ RESPONDING_TO_DEMONSTRATION');
    demoShapeReceived = infoFromPrevState['demoShapeReceived'];
    shape = demoShapeReceived['path'];
    shapeIndex_demoFor = demoShapeReceived['shapeType_code'];
    numPointsInShape = len(shape)/2;
    x_shape = shape[0:numPointsInShape];
    y_shape = shape[numPointsInShape:];
    
    if isinstance(x_shape,numpy.ndarray): #convert arrays to lists for interp1d
        x_shape = (x_shape.T).tolist()[0];
        y_shape = (y_shape.T).tolist()[0];
        
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
    shape = numpy.reshape(shape, (-1, 1)); #explicitly make it 2D array with only one column
    if(args.show):
        plt.figure(1);
        ShapeModeler.normaliseAndShowShape(shape);
    
    shapeType = wordManager.shapeAtIndexInCurrentCollection(shapeIndex_demoFor);
    if(naoSpeaking):
        global demo_responses_counter
        try:
            toSay = demo_responses[demo_responses_counter]%shapeType;
        except TypeError:
            toSay = demo_responses[demo_responses_counter];
        demo_responses_counter += 1;
        if(demo_responses_counter==len(demo_responses)):
            demo_responses_counter = 0;
        textToSpeech.say(toSay);
        print('NAO: '+toSay);  
    

    print("Received demo for " + shapeType);
    shape = wordManager.respondToDemonstration(shapeIndex_demoFor, shape);
    state_goTo = deepcopy(drawingLetterSubstates);
    nextState = state_goTo.pop(0);
    infoForNextState = {'state_goTo': state_goTo, 'state_cameFrom': "RESPONDING_TO_DEMONSTRATION",'shapesToPublish': [shape]};
    return nextState, infoForNextState
    
def make_traj_msg(shape, shapeCentre, headerString, startTime, downsample, deltaT):      
    if(startTime!=t0):
        penUpToFirst = True;
    else:
        penUpToFirst = False;
    
    traj = Path();
    traj.header.frame_id = FRAME#headerString;
    traj.header.stamp = rospy.Time.now()+rospy.Duration(delayBeforeExecuting);
    shape = ShapeModeler.normaliseShapeHeight(shape);
    numPointsInShape_orig = len(shape)/2;   
    
    x_shape = shape[0:numPointsInShape_orig];
    y_shape = shape[numPointsInShape_orig:];
    
    if(downsample):
        #make shape have the appropriate number of points
        t_current = numpy.linspace(0, 1, numPointsInShape_orig);
        t_desired = numpy.linspace(0, 1, numDesiredShapePoints);
        f = interpolate.interp1d(t_current, x_shape[:,0], kind='cubic');
        x_shape = f(t_desired);
        f = interpolate.interp1d(t_current, y_shape[:,0], kind='cubic');
        y_shape = f(t_desired);

        numPointsInShape = len(x_shape);
        downsampleFactor = numPointsInShape_orig/float(numPointsInShape);
    else:
        numPointsInShape = numPointsInShape_orig;

    for i in range(numPointsInShape):
        point = PoseStamped();

        point.pose.position.x = x_shape[i]*sizeScale_width;
        point.pose.position.y = -y_shape[i]*sizeScale_height;
        
        point.pose.position.x+= + shapeCentre[0];
        point.pose.position.y+= + shapeCentre[1];
        
        point.header.frame_id = FRAME;
        point.header.stamp = rospy.Time(startTime+i*deltaT+t0); #@todo allow for variable time between points for now

        if(penUpToFirst and i==0):
            point.header.seq = 1;

        traj.poses.append(deepcopy(point));

    if(downsample):
        return traj, downsampleFactor
    else:
        return traj
    
###
            
         
###---------------------------------------------- WORD LEARNING SETTINGS

def generateSettings(shapeType):
    paramsToVary = [2];            #Natural number between 1 and numPrincipleComponents, representing which principle component to vary from the template
    initialBounds_stdDevMultiples = numpy.array([[-6, 6]]);  #Starting bounds for paramToVary, as multiples of the parameter's observed standard deviation in the dataset
    doGroupwiseComparison = True; #instead of pairwise comparison with most recent two shapes
    initialParamValue = numpy.NaN;
    initialBounds = numpy.array([[numpy.NaN, numpy.NaN]]);
    
    if shapeType == 'a':
        paramsToVary = [6];
        initialBounds_stdDevMultiples = numpy.array([[-3, 3]]);
        datasetFile = '../res/a_noHook_dataset.txt';
        initialParamValue = 0.8; 
    elif shapeType == 'c':
        paramToVary = 4;
        initialBounds_stdDevMultiples = numpy.array([[-10, 10]]);
        datasetFile = '../res/c_dataset.txt';
        
        if(learningModes[shapeType] == LearningModes.startsRandom):
            initialBounds_stdDevMultiples = numpy.array([[-10, 10]]);
        elif(learningModes[shapeType] == LearningModes.alwaysGood):
            initialBounds_stdDevMultiples = numpy.array([[-4, -3]]);
            initialParamValue = -0.5;
        elif(learningModes[shapeType] == LearningModes.alwaysBad):
            initialBounds_stdDevMultiples = numpy.array([[1.5, 10]]);
            initialParamValue = 0.5;
        elif(learningModes[shapeType] == LearningModes.startsBad):
            initialBounds_stdDevMultiples = numpy.array([[-10, 10]]);
            initialParamValue = 0.5; 
    elif shapeType == 'd':
        datasetFile = '../res/d_cursive_dataset.txt';
    elif shapeType == 'e':
        paramToVary = 3; 
        initialBounds_stdDevMultiples = numpy.array([[-6, 14]]);
        datasetFile = '../res/e_dataset.txt';
        #initialParamValue = 0.8;
    elif shapeType == 'm':
        paramToVary = 6; 
        initialBounds_stdDevMultiples = numpy.array([[-10, -6]]);
        datasetFile = '../res/m_dataset.txt';
        initialParamValue = -0.5;#0.0;
    elif shapeType == 'n':
        paramToVary = 7; 
        datasetFile = '../res/n_dataset.txt';
        initialParamValue = 0.0;
    elif shapeType == 'o':
        paramsToVary = [4];
        initialBounds_stdDevMultiples = numpy.array([[-3.5, 3]]);
        datasetFile = '../res/o_dataset.txt';
    elif shapeType == 'r':
        paramToVary = 1;
        datasetFile = '../res/r_print_dataset.txt';
    elif shapeType == 's':
        datasetFile = '../res/s_print_dataset.txt';
    elif shapeType == 'u':
        paramsToVary = [3];
        datasetFile = '../res/u_dataset.txt';
    elif shapeType == 'v':
        paramToVary = 6;
        datasetFile = '../res/v_dataset.txt';
    elif shapeType == 'w':
        datasetFile = '../res/w_dataset.txt';
    else:
        raise RuntimeError("Dataset is not known for shape "+ shapeType);
    settings = SettingsStruct(shape_learning = shapeType,
    paramsToVary = paramsToVary, doGroupwiseComparison = True, 
    datasetFile = datasetFile, initialBounds = initialBounds, 
    initialBounds_stdDevMultiples = initialBounds_stdDevMultiples,
    initialParamValue = initialParamValue, minParamDiff = 0.4);
    return settings
    
def lookAtShape(traj):
    trajStartPosition = traj.poses[0].pose.position;
    trajStartPosition_robot = tl.transformPose("base_footprint",target)
    nao.look_at([trajStartPosition.x,trajStartPosition.y,trajStartPosition.z,target_frame]); #look at shape again 
    
def lookAtTablet():
    if(effector=="RArm"):   #tablet will be on our right
        nao.setpose({"HEAD":(-0.2, 0.08125996589660645)})   
    else: 
        nao.setpose({"HEAD":(0.2, 0.08125996589660645)}) 
    
def lookAndAskForFeedback(toSay,side):
    if(naoWriting):
        #put arm down
        nao.execute([naoqi_request("motion","angleInterpolationWithSpeed",["RArm",joints_standInit,0.2])])
    
    if(side=="Right"):   #person will be on our right
        nao.setpose({"HEAD":(-0.9639739513397217, 0.08125996589660645)})
    else:                   #person will be on our left
        nao.setpose({"HEAD":(0.9639739513397217, 0.08125996589660645)})
        
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
    shapeFinished = True; #TODO only register when appropriate
               
### ----------------------------------------- PUBLISH SIMULATED FEEDBACK    
def publishSimulatedFeedback(bestShape_index, shapeType_index, doGroupwiseComparison):           
        feedback = String();
        if(doGroupwiseComparison):
            feedback.data = str(shapeType_index) + '_' + str(bestShape_index);
        else:
            names = ('old', 'new');
            feedback.data = names[bestShape_index];            
        onFeedbackReceived(feedback);
        
### ------------------------------------------------------ PUBLISH SHAPE        
def publishShape(infoFromPrevState):
    print('------------------------------------------ PUBLISHING_LETTER'); 
    shapesToPublish = infoFromPrevState['shapesToPublish'];
    shape = shapesToPublish.pop(0); #publish next remaining shape (and remove from list)

    if(simulatedFeedback):
        if(args.show):
            plt.figure(1);
            ShapeModeler.normaliseAndShowShape(shape.path);
            time.sleep(1.3); 
    else:
       
        try:
            display_new_shape = rospy.ServiceProxy('display_new_shape', displayNewShape);
            response = display_new_shape(shape_type_code = shape.shapeType_code);
            shapeCentre = numpy.array([response.location.x, response.location.y]);
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        headerString = shape.shapeType+'_'+str(shape.paramsToVary)+'_'+str(shape.paramValues);
        [traj_downsampled, downsampleFactor] = make_traj_msg(shape.path, shapeCentre, headerString, t0, True, dt); #for robot
        downsampleFactor = float(numPoints_shapeModeler-1)/float(numDesiredShapePoints-1);
        traj = make_traj_msg(shape.path, shapeCentre, headerString, t0, False, float(dt)/downsampleFactor);
        #traj = make_traj_msg(shape.path, shapeCentre, headerString, t0, False, dt);
        trajStartPosition = traj.poses[0].pose.position;
        if(naoConnected):
            lookAtTablet();
        pub_traj_downsampled.publish(traj_downsampled);
        pub_traj.publish(traj);
           
    
    nextState = "WAITING_FOR_LETTER_TO_FINISH";
    infoForNextState = {'state_cameFrom':  "PUBLISHING_LETTER",'state_goTo': ["ASKING_FOR_FEEDBACK"],'centre': trajStartPosition};
    if(len(shapesToPublish) > 0): #more shapes to publish
        state_goTo = deepcopy(drawingLetterSubstates);#come back to publish the remaining shapes
        infoForNextState = {'state_goTo': state_goTo,'state_cameFrom': "PUBLISHING_LETTER",'shapesToPublish': shapesToPublish,'centre': trajStartPosition};
    
    return nextState, infoForNextState
    
### ------------------------------------------------------ PUBLISH WORD        
def publishWord(infoFromPrevState):
    print('------------------------------------------ PUBLISHING_WORD'); 
    shapesToPublish = infoFromPrevState['shapesToPublish'];
    
    wholeTraj = Path();
    wholeTraj_downsampled = Path();
    
    startTime = t0;
    for shape in shapesToPublish:
        
        try:
            display_new_shape = rospy.ServiceProxy('display_new_shape', displayNewShape);
            response = display_new_shape(shape_type_code = shape.shapeType_code);
            shapeCentre = numpy.array([response.location.x, response.location.y]);
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        headerString = shape.shapeType+'_'+str(shape.paramsToVary)+'_'+str(shape.paramValues);
        [traj_downsampled, downsampleFactor] = make_traj_msg(shape.path, shapeCentre, headerString, startTime, True, dt); #for robot
        
        downsampleFactor = float(numPoints_shapeModeler-1)/float(numDesiredShapePoints-1);
        traj = make_traj_msg(shape.path, shapeCentre, headerString, startTime, False, float(dt)/downsampleFactor);
        #[traj, downsampleFactor] = make_traj_msg(shape.path, shapeCentre, headerString, startTime, True, dt);
        
        wholeTraj.poses.extend(deepcopy(traj.poses));
        wholeTraj_downsampled.poses.extend(deepcopy(traj_downsampled.poses));
        startTime = traj.poses[-1].header.stamp.to_sec()+1; #start after the previous shape finishes next time
    
    wholeTraj.header = traj.header;
    wholeTraj_downsampled.header = traj.header;
    trajStartPosition = wholeTraj.poses[0].pose.position;
    if(naoConnected):
        lookAtTablet();
    pub_traj_downsampled.publish(wholeTraj_downsampled);
    pub_traj.publish(wholeTraj);  
    
    shapesToPublish = [];
    nextState = "WAITING_FOR_LETTER_TO_FINISH";
    infoForNextState = {'state_cameFrom':  "PUBLISHING_WORD",'state_goTo': ["ASKING_FOR_FEEDBACK"],'centre': trajStartPosition};

    return nextState, infoForNextState
    
    
infoToRestore_waitForShapeToFinish = rospy.Subscriber(SHAPE_FINISHED_TOPIC, String, onShapeFinished); 
global shape_finished_subscriber;
def waitForShapeToFinish(infoFromPrevState):
    global infoToRestore_waitForShapeToFinish
    #FORWARDER STATE
    
    #first time into this state preparations
    if(infoFromPrevState['state_cameFrom'] != "WAITING_FOR_LETTER_TO_FINISH"):
        print('------------------------------------------ WAITING_FOR_LETTER_TO_FINISH');
        infoToRestore_waitForShapeToFinish = infoFromPrevState;
    
    #default
    nextState = 'WAITING_FOR_LETTER_TO_FINISH';
    infoForNextState = {'state_cameFrom': 'WAITING_FOR_LETTER_TO_FINISH'};
    
    #once shape has finished
    global shapeFinished
    if(shapeFinished):
        shapeFinished = False;
        
        infoForNextState = infoToRestore_waitForShapeToFinish;
        try:
            if(infoForNextState['state_goTo'] is not None and len(infoForNextState['state_goTo'])>0):
                nextState = infoForNextState['state_goTo'].pop(0); #go to the next state requested to and remove it from the list
                #TODO make sure it actually gets executed before popping it...
        except:
            #nothing planned..
            nextState = 'WAITING_FOR_FEEDBACK';
    '''       
    #act if the tablet disconnects
    if(not tabletWatchdog.isResponsive()):
        nextState = 'WAITING_FOR_TABLET_TO_CONNECT';
        infoForNextState = {'state_goTo': ['WAITING_FOR_FEEDBACK'], 'state_cameFrom': 'WAITING_FOR_LETTER_TO_FINISH'};
        #TODO go back and re-send whatever we just send that we never got the shapeFinished message for...
    '''

    if(stopRequestReceived):
        nextState = "STOPPING";   
    
    return nextState, infoForNextState
          
def respondToFeedback(infoFromPrevState):
    print('------------------------------------------ RESPONDING_TO_FEEDBACK'); 
    global shapeFinished #todo: make class attribute
    
    stringReceived = infoFromPrevState['feedbackReceived'];
    
    nextState = "WAITING_FOR_FEEDBACK";
    infoForNextState = {'state_cameFrom': "ASKING_FOR_FEEDBACK"};
    
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
            if(naoSpeaking):
                toSay = 'Ok, thanks for helping me';
                print('NAO: '+toSay);
                textToSpeech.say(toSay); 
            #pass feedback to shape manager
            response = wordManager.feedbackManager(shapeIndex_messageFor, bestShape_index, noNewShape);
            if(response == -1):
                print('Something\'s gone wrong in the feedback manager');
            
        else:
            if(naoSpeaking):
                shape_messageFor = wordManager.shapeAtIndexInCurrentCollection(shapeIndex_messageFor);
                toSay = 'Ok, I\'ll work on the '+shape_messageFor;
                print('NAO: '+toSay);
                textToSpeech.say(toSay);
            
            [numItersConverged, newShape] = wordManager.feedbackManager(shapeIndex_messageFor, bestShape_index, noNewShape);
            
            if(numItersConverged == 0):
                state_goTo = deepcopy(drawingLetterSubstates);
                nextState = state_goTo.pop(0);
                infoForNextState = {'state_goTo': state_goTo,'state_cameFrom': "RESPONDING_TO_FEEDBACK",'shapesToPublish': [newShape]};
            else:
                pass #TODO handle convergence

    global wordReceived
    if(wordReceived is not None):
        infoForNextState['wordReceived'] = wordReceived;
        wordReceived = None;
        nextState = "RESPONDING_TO_NEW_WORD";
    global testRequestReceived
    if(testRequestReceived):
        testRequestReceived = None;
        nextState = "RESPONDING_TO_TEST_CARD";
    if(stopRequestReceived):
        nextState = "STOPPING";
    return nextState, infoForNextState
        
def respondToNewWord(infoFromPrevState):
    print('------------------------------------------ RESPONDING_TO_NEW_WORD'); 
    global shapeFinished, wordManager #@todo make class attribute 
    message = infoFromPrevState['wordReceived'];
    print("Cheers");
    wordToLearn = message.data;
    wordSeenBefore = wordManager.newCollection(wordToLearn);
    if(naoSpeaking):
        if(wordSeenBefore):
            #toSay = wordToLearn+' again, ok.';
            global word_responses_again_counter
            try:
                toSay = word_responses_again[word_responses_again_counter]%wordToLearn;
            except TypeError:
                toSay = word_responses_again[word_responses_again_counter];
            word_responses_again_counter += 1;
            if(word_responses_again_counter==len(word_responses_again)):
                word_responses_again_counter = 0;

        else:
            #toSay = wordToLearn+', alright.';
            global word_responses_counter
            try:
                toSay = word_responses[word_responses_counter]%wordToLearn;
            except TypeError:
                toSay = word_responses[word_responses_counter];
            word_responses_counter += 1;
            if(word_responses_counter==len(word_responses)):
                word_responses_counter = 0;
    
        print('NAO: '+toSay);
        textToSpeech.say(toSay);    
        
    #clear screen
    pub_clear.publish(Empty());
    rospy.sleep(0.5);
    
    #start learning    
    shapesToPublish = [];    
    for i in range(len(wordToLearn)):
        shape = wordManager.startNextShapeLearner();
        shapesToPublish.append(shape);

    nextState = 'PUBLISHING_WORD';
    infoForNextState = {'state_cameFrom': "RESPONDING_TO_NEW_WORD",'shapesToPublish': shapesToPublish};

    global wordReceived
    if(wordReceived is not None):
        infoForNextState['wordReceived'] = wordReceived;
        wordReceived = None;
        nextState = "RESPONDING_TO_NEW_WORD";
    global testRequestReceived
    if(testRequestReceived):
        testRequestReceived = None;
        nextState = "RESPONDING_TO_TEST_CARD";
    if(stopRequestReceived):
        nextState = "STOPPING";
    return nextState, infoForNextState
    
def askForFeedback(infoFromPrevState): 
    print('------------------------------------------ ASKING_FOR_FEEDBACK'); 
    centre = infoFromPrevState['centre']; 
    print(infoFromPrevState['state_cameFrom'])
    if(infoFromPrevState['state_cameFrom'] == "PUBLISHING_WORD"):
        print('Asking for feedback on word...');
        if(naoSpeaking):
            global asking_phrases_after_word_counter
            try:
                toSay = asking_phrases_after_word[asking_phrases_after_word_counter];
            except TypeError:
                toSay = asking_phrases_after_word[asking_phrases_after_word_counter];
            asking_phrases_after_word_counter += 1;
            if(asking_phrases_after_word_counter==len(asking_phrases_after_word)):
                asking_phrases_after_word_counter = 0;
            global nextSideToLookAt
            lookAndAskForFeedback(toSay,nextSideToLookAt);
            if(alternateSidesLookingAt):
                if(nextSideToLookAt == 'Left'):
                    nextSideToLookAt = 'Right';
                else:
                    nextSideToLookAt = 'Left';
            lookAtTablet();
    elif(infoFromPrevState['state_cameFrom'] == "PUBLISHING_LETTER"):
        print('Asking for feedback on letter...');
        if(naoSpeaking):
            global asking_phrases_after_feedback_counter
            try:
                toSay = asking_phrases_after_feedback[asking_phrases_after_feedback_counter];
            except TypeError:
                toSay = asking_phrases_after_feedback[asking_phrases_after_feedback_counter];
            asking_phrases_after_feedback_counter += 1;
            if(asking_phrases_after_feedback_counter==len(asking_phrases_after_feedback)):
                asking_phrases_after_feedback_counter = 0;
            global nextSideToLookAt
            lookAndAskForFeedback(toSay,nextSideToLookAt);
            if(alternateSidesLookingAt):
                if(nextSideToLookAt == 'Left'):
                    nextSideToLookAt = 'Right';
                else:
                    nextSideToLookAt = 'Left';
            lookAtTablet();
    elif(infoFromPrevState['state_cameFrom'] == "RESPONDING_TO_DEMONSTRATION"):
        print('Asking for feedback on demo response...');
        if(naoSpeaking):
            lookAndAskForFeedback("How about now?");
            lookAtTablet();
    nextState = "WAITING_FOR_FEEDBACK";
    infoForNextState = {'state_cameFrom': "ASKING_FOR_FEEDBACK"};
    global wordReceived;
    if(wordReceived is not None):
        infoForNextState['wordReceived'] = wordReceived;
        wordReceived = None;
        nextState = "RESPONDING_TO_NEW_WORD";
    global testRequestReceived;
    if(wordReceived is not None):
        testRequestReceived = None;
        nextState = "RESPONDING_TO_TEST_CARD";
    if(stopRequestReceived):
        nextState = "STOPPING";
    return nextState, infoForNextState

testRequestReceived = False;
def onTestRequestReceived(message):
    global testRequestReceived
    #TODO: DON'T RESPOND TO TEST CARD UNTIL YOU HAVE LEARNT SOMETHING
    testRequestReceived = True;
    
def respondToTestCard(infoFromPrevState):
    print('------------------------------------------ RESPONDING_TO_TEST_CARD');
    print('Show me a test word');
    if(naoSpeaking):
        textToSpeech.say(testPhrase);
    nextState = "WAITING_FOR_WORD";
    infoForNextState = {'state_cameFrom': "RESPONDING_TO_TEST_CARD"};
    return nextState, infoForNextState
    
stopRequestReceived = False;
def onStopRequestReceived(message):
    global stopRequestReceived
    stopRequestReceived = True;
    
def stopInteraction(infoFromPrevState):
    print('------------------------------------------ STOPPING');
    if(naoSpeaking):
        textToSpeech.say(thankYouPhrase);   
    if(naoConnected):
        nao.execute([naoqi_request("motion","rest",[])]);

    nextState = "EXIT";
    infoForNextState = 0;
    rospy.signal_shutdown('Interaction exited');
    return nextState, infoForNextState
          
def onClearScreenReceived(message):
    print('Clearing display');
    try:
        clear_all_shapes = rospy.ServiceProxy('clear_all_shapes', clearAllShapes);
        resp1 = clear_all_shapes();
        print( resp1.success);
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def startInteraction(infoFromPrevState):
    print('------------------------------------------ STARTING_INTERACTION');
    print('Hey I\'m Nao');
    print("Do you have any words for me to write?");
    if(naoSpeaking):
        global nextSideToLookAt
        lookAndAskForFeedback(introPhrase,nextSideToLookAt);
    nextState = "WAITING_FOR_WORD";
    infoForNextState = {'state_cameFrom': "STARTING_INTERACTION"};
    if(stopRequestReceived):
        nextState = "STOPPING";
    return nextState, infoForNextState

def onWordReceived(message):
    global wordReceived 

    if(stateMachine.get_state() == "WAITING_FOR_FEEDBACK"
    or stateMachine.get_state() == "WAITING_FOR_WORD"
    or stateMachine.get_state() == "ASKING_FOR_FEEDBACK" 
    or stateMachine.get_state() is None): #state machine hasn't started yet - word probably came from input arguments
        wordReceived = message;
        print('Received word');
    else:
        wordReceived = None; #ignore 
          
def onNewChildReceived(message):
    if(naoWriting):
            nao.setpose("StandInit");
    if(naoSpeaking):
        global nextSideToLookAt
        #toSay = "Hello. I'm Nao. Please show me a word to practice.";
        lookAndAskForFeedback(introPhrase,nextSideToLookAt);
    #clear screen
    pub_clear.publish(Empty());
    rospy.sleep(0.5);

def waitForWord(infoFromPrevState):
    global wordReceived
    
    if(infoFromPrevState['state_cameFrom'] != "WAITING_FOR_WORD"):
        print('------------------------------------------ WAITING_FOR_WORD');
        pub_camera_status.publish(True); #turn camera on
    if(infoFromPrevState['state_cameFrom'] == "STARTING_INTERACTION"):
        pass
    
    infoForNextState = {'state_cameFrom': "WAITING_FOR_WORD"};
    if(wordReceived is None):
        nextState = "WAITING_FOR_WORD";
    else:
        infoForNextState['wordReceived'] = wordReceived;
        wordReceived = None;
        nextState = "RESPONDING_TO_NEW_WORD";
        pub_camera_status.publish(False); #turn camera off
    if(stopRequestReceived):
        nextState = "STOPPING";
        pub_camera_status.publish(False); #turn camera off
    return nextState, infoForNextState
    
def onFeedbackReceived(message):
    global feedbackReceived 
    if(stateMachine.get_state() == "ASKING_FOR_FEEDBACK" 
        or stateMachine.get_state() == "WAITING_FOR_FEEDBACK" 
        or stateMachine.get_state() == "WAITING_FOR_LETTER_TO_FINISH" ):
        feedbackReceived = message; #replace any existing feedback with new
        print('Received feedback');
    elif(stateMachine.get_state() == "RESPONDING_TO_FEEDBACK"):
        feedbackReceived = None; #ignore feedback

feedbackReceived = None;
def waitForFeedback(infoFromPrevState):
    
    if(infoFromPrevState['state_cameFrom'] != "WAITING_FOR_FEEDBACK"):
        print('------------------------------------------ WAITING_FOR_FEEDBACK');
        pub_camera_status.publish(True); #turn camera on
        
    #default behaviour is to loop
    nextState = "WAITING_FOR_FEEDBACK";
    infoForNextState = {'state_cameFrom': "WAITING_FOR_FEEDBACK"};
    
    global feedbackReceived    
    if(feedbackReceived is not None):
        infoForNextState['feedbackReceived'] = feedbackReceived;
        feedbackReceived = None;
        nextState = "RESPONDING_TO_FEEDBACK";
        infoForNextState['state_goTo'] = [nextState];
        nextState = 'WAITING_FOR_ROBOT_TO_CONNECT';
        
    global demoShapeReceived    
    if(demoShapeReceived is not None):
        infoForNextState ['demoShapeReceived'] = demoShapeReceived; 
        demoShapeReceived = None;
        nextState = "RESPONDING_TO_DEMONSTRATION";    
        infoForNextState['state_goTo'] = [nextState]; #ensure robot is connected before going to that state
        nextState = 'WAITING_FOR_ROBOT_TO_CONNECT';
        
    global wordReceived
    if(wordReceived is not None):
        infoForNextState['wordReceived'] = wordReceived;
        wordReceived = None;
        nextState = "RESPONDING_TO_NEW_WORD";
        infoForNextState['state_goTo'] = [nextState]; #ensure robot is connected before going to that state
        nextState = 'WAITING_FOR_ROBOT_TO_CONNECT';
        
    global testRequestReceived
    if(testRequestReceived):
        testRequestReceived = None;
        nextState = "RESPONDING_TO_TEST_CARD";
        infoForNextState['state_goTo'] = [nextState]; #ensure robot is connected before going to that state
        nextState = 'WAITING_FOR_ROBOT_TO_CONNECT';
        
    if(stopRequestReceived):
        nextState = "STOPPING";
        
    if(nextState != 'WAITING_FOR_FEEDBACK'):
        pub_camera_status.publish(False); #turn camera off
    return nextState, infoForNextState    

#def respondToTabletDisconnect(infoFromPrevState):
 #   infoForNextState = {'state_toReturnTo': "PUBLISHING_LETTER"}
infoToRestore_waitForRobotToConnect = None;
def waitForRobotToConnect(infoFromPrevState):
    global infoToRestore_waitForRobotToConnect
    #FORWARDER STATE
    if(infoFromPrevState['state_cameFrom'] != "WAITING_FOR_ROBOT_TO_CONNECT"):
        print('------------------------------------------ waiting_for_robot_to_connect');
        infoToRestore_waitForRobotToConnect = infoFromPrevState;
    
    nextState = "WAITING_FOR_ROBOT_TO_CONNECT";
    infoForNextState = {'state_cameFrom': "WAITING_FOR_ROBOT_TO_CONNECT"};
    
    if(robotWatchdog.isResponsive()):
        infoForNextState = infoToRestore_waitForRobotToConnect;
        try:
            nextState = infoForNextState['state_goTo'].pop(0);
        except:
            import pdb; pdb.set_trace();
    
    if(stopRequestReceived):
        nextState = "STOPPING";
    return nextState, infoForNextState
    
infoToRestore_waitForTabletToConnect = None;
def waitForTabletToConnect(infoFromPrevState):
    global infoToRestore_waitForTabletToConnect
    #FORWARDER STATE
    if(infoFromPrevState['state_cameFrom'] != "WAITING_FOR_TABLET_TO_CONNECT"):
        print('------------------------------------------ waiting_for_tablet_to_connect');
        infoToRestore_waitForTabletToConnect = infoFromPrevState;
    
    nextState = "WAITING_FOR_TABLET_TO_CONNECT";
    infoForNextState = {'state_cameFrom': "WAITING_FOR_TABLET_TO_CONNECT"};
    
    if(tabletWatchdog.isResponsive()): #reconnection - send message to wherever it was going
        infoForNextState = infoToRestore_waitForTabletToConnect;
        nextState = infoForNextState['state_goTo'].pop(0);
    
    if(stopRequestReceived):
        nextState = "STOPPING";
    return nextState, infoForNextState    

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
    #feedback_subscriber = rospy.Subscriber(FEEDBACK_TOPIC, String, onFeedbackReceived);
    
    #listen for gesture representing active demo shape 
    gesture_subscriber = rospy.Subscriber(GESTURE_TOPIC, PointStamped, onSetActiveShapeGesture); 
    
    #listen for a new child signal
    new_child_subscriber = rospy.Subscriber(NEW_CHILD_TOPIC, String, onNewChildReceived);
    
    #listen for words to write
    words_subscriber = rospy.Subscriber(WORDS_TOPIC, String, onWordReceived);
    
    #listen for request to clear screen (from tablet)
    clear_subscriber = rospy.Subscriber(CLEAR_SCREEN_TOPIC, Empty, onClearScreenReceived);
    
    #listen for test time
    test_subscriber = rospy.Subscriber(TEST_TOPIC, Empty, onTestRequestReceived);
    
    #listen for when to stop
    stop_subscriber = rospy.Subscriber(STOP_TOPIC, Empty, onStopRequestReceived); 
    
    #listen for user-drawn shapes
    shape_subscriber = rospy.Subscriber(USER_DRAWN_SHAPES_TOPIC, Path, userShapePreprocessor); 
        
    #initialise display manager for shapes (manages positioning of shapes)
    from display_manager.srv import *
    print('Waiting for display manager services to become available');
    rospy.wait_for_service('clear_all_shapes');
    
    rospy.sleep(2.0);   #Allow some time for the subscribers to do their thing, 
                        #or the first message will be missed (eg. first traj on tablet, first clear request locally)
    
    from watchdog import Watchdog #TODO: Make a ROS server so that *everyone* can access the connection statuses
    tabletWatchdog = Watchdog('watchdog_clear/tablet', 0.4);
    robotWatchdog = Watchdog('watchdog_clear/robot', 0.8);
    
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
        textToSpeech.setLanguage(naoLanguage)
        #textToSpeech.setVolume(1.0);
        if(naoWriting):
            nao.setpose("StandInit");
            [temp,joints_standInit] = nao.execute([naoqi_request("motion","getAngles",["RArm",True])]);
            nao.execute([naoqi_request("motion","wbEnableEffectorControl",[effector,True])])
            
    #initialise word manager (passes feedback to shape learners and keeps history of words learnt)
    wordManager = ShapeLearnerManager(generateSettings);
            
    
    stateMachine = StateMachine();
    stateMachine.add_state("STARTING_INTERACTION", startInteraction);
    stateMachine.add_state("WAITING_FOR_ROBOT_TO_CONNECT", waitForRobotToConnect);
    stateMachine.add_state("WAITING_FOR_WORD", waitForWord);
    stateMachine.add_state("RESPONDING_TO_NEW_WORD", respondToNewWord);
    stateMachine.add_state("PUBLISHING_WORD", publishWord);
    stateMachine.add_state("PUBLISHING_LETTER", publishShape);
    stateMachine.add_state("WAITING_FOR_LETTER_TO_FINISH", waitForShapeToFinish);
    stateMachine.add_state("ASKING_FOR_FEEDBACK", askForFeedback);
    stateMachine.add_state("WAITING_FOR_FEEDBACK", waitForFeedback);
    stateMachine.add_state("RESPONDING_TO_FEEDBACK", respondToFeedback);
    stateMachine.add_state("RESPONDING_TO_DEMONSTRATION", respondToDemonstration);
    stateMachine.add_state("RESPONDING_TO_TEST_CARD", respondToTestCard);
    #stateMachine.add_state("RESPONDING_TO_TABLET_DISCONNECT", respondToTabletDisconnect);
    stateMachine.add_state("WAITING_FOR_TABLET_TO_CONNECT", waitForTabletToConnect);
    stateMachine.add_state("STOPPING", stopInteraction);
    stateMachine.add_state("EXIT", None, end_state=True);
    stateMachine.set_start("WAITING_FOR_ROBOT_TO_CONNECT");
    infoForStartState = {'state_goTo': ["STARTING_INTERACTION"], 'state_cameFrom': None};
    
    wordToLearn = args.word;
    if(wordToLearn is not None):
        message = String();
        message.data = wordToLearn;
        onWordReceived(message);
    else:
        print('Waiting for word to write');
    
    stateMachine.run(infoForStartState);    
    
    rospy.spin();

    tabletWatchdog.stop();
    robotWatchdog.stop();
