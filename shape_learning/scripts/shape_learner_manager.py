#!/usr/bin/env python

"""
Manages a collection of shape_learners, with long-term memory about the 
history of previous collections seen. An example is managing shape_learners
which represent letters, and the collections represent words. 
"""

from shape_modeler import ShapeModeler
from shape_learner import ShapeLearner

import numpy          
         
###--------------------------------------------- WORD LEARNING FUNCTIONS
# @todo make methods of a class
class ShapeLearnerManager:
    def __init__(self, generateSettingsFunction):
        self.generateSettings = generateSettingsFunction;
        self.shapesLearnt = [];
        self.shapeLearners = [];
        self.settings_shapeLearners = [];
        self.currentCollection = [];
        self.collectionsLearnt = [];
        self.nextShapeLearnerToBeStarted = 0;
        
        
    def initialiseShapeLearners(self):
        for i in range(len(self.currentCollection)):
            shapeType = self.currentCollection[i];
            
            #check if shape has been learnt before
            try:
                shapeType_index = self.shapesLearnt.index(shapeType);
                newShape = False;
            except ValueError: 
                newShape = True;
            
            if(newShape):
                settings = self.generateSettings(shapeType); 

                shapeLearner = ShapeLearner(settings);
                self.shapesLearnt.append(shapeType);
                self.shapeLearners.append(shapeLearner);
                self.settings_shapeLearners.append(settings);
            else:
                #use the bounds determined last time
                previousBounds = self.shapeLearners[shapeType_index].getParameterBounds();
                newInitialBounds = previousBounds;
                newInitialBounds[0] -= boundExpandingAmount;
                newInitialBounds[1] += boundExpandingAmount;
                self.shapeLearners[shapeIndex_messageFor].setParameterBounds(newInitialBounds);
        
                     
    def startNextShapeLearner(self):
        #start learning
        if( self.nextShapeLearnerToBeStarted < len(self.currentCollection) ):
            shapeType = self.currentCollection[self.nextShapeLearnerToBeStarted];
            shapeType_code = self.nextShapeLearnerToBeStarted;
            shape_index = self.shapesLearnt.index(shapeType);
            [shape, paramValue] = self.shapeLearners[shape_index].startLearning();
            paramToVary = self.settings_shapeLearners[shape_index].paramsToVary[0]; #USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
            self.nextShapeLearnerToBeStarted += 1;
            return shape, shapeType, shapeType_code, paramToVary, paramValue;
        else:
            print('Don\'t know what shape learner you want me to start...');
            return -1, -1, -1, -1, -1;

    def feedbackManager(self, shapeIndex_messageFor, bestShape_index, noNewShape):
        shape_messageFor = self.shapeAtIndexInCurrentCollection(shapeIndex_messageFor);
        if(shape_messageFor < 0 ):
            print('Ignoring message because not for valid shape type');
            return -1;
        else:
        
            if(noNewShape): #just respond to feedback, don't make new shape 
                self.shapeLearners[shapeIndex_messageFor].respondToFeedback(bestShape_index);
                return 1;
            else:               
                [numItersConverged, newShape, newParamValue] = self.shapeLearners[shapeIndex_messageFor].generateNewShapeGivenFeedback(bestShape_index);
            return numItersConverged, newShape, shape_messageFor, shapeIndex_messageFor, self.settings_shapeLearners[shapeIndex_messageFor].paramsToVary[0], newParamValue;#USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
    
    def respondToDemonstration(self, shapeIndex_messageFor, shape):
        shape_messageFor = self.shapeAtIndexInCurrentCollection(shapeIndex_messageFor);
        if(shape_messageFor < 0 ):
            print('Ignoring demonstration because not for valid shape type');
            return -1;
        else:
            return self.shapeLearners[shapeIndex_messageFor].respondToDemonstration(shape);
    
    def indexOfShapeInCurrentCollection(self, shapeType):
        try:
            shapeType_index = self.currentCollection.index(shapeType);
        except ValueError: #unknown shape
            shapeType_index = -1;
        return shapeType_index;
            
    def shapeAtIndexInCurrentCollection(self, shapeType_index):
        try:
            shapeType = self.currentCollection[shapeType_index];
        except ValueError: #unknown shape
            shapeType = -1;
        return shapeType;
            
    def newCollection(self, collection):
        self.currentCollection = collection;
        self.nextShapeLearnerToBeStarted = 0;
        
        try:
            collection_index = self.collectionsLearnt.index(self.currentCollection);
            collectionSeenBefore = True;
        except ValueError: 
            collectionSeenBefore = False;
            self.collectionsLearnt.append(self.currentCollection);

        self.initialiseShapeLearners(); 
        
        return collectionSeenBefore;

    def resetParameterBounds(self, shapeType_index):
        currentBounds = self.shapeLearners[shapeType_index].getParameterBounds();
               
        #change bounds back to the initial ones 
        newBounds = self.shapeLearners[shapeType_index].initialBounds;
        self.shapeLearners[shapeType_index].setParameterBounds(newBounds);
        print('Changing bounds on shape '+self.shapeAtIndexInCurrentCollection(shapeType_index)+' from '+str(currentBounds)+' to '+str(newBounds));
    
    def generateSimulatedFeedback(self, shapeType_index, newShape, newParamValue):
        return self.shapeLearners[shapeType_index].generateSimulatedFeedback(newShape, newParamValue);
