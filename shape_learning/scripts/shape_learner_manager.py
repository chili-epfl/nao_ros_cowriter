#!/usr/bin/env python

"""
Manages a collection of shape_learners, with long-term memory about the 
history of previous collections seen. An example is managing shape_learners
which represent letters, and the collections represent words. 
"""

from shape_modeler import ShapeModeler
from shape_learner import ShapeLearner

import numpy          
         
boundExpandingAmount = 0.2;
###--------------------------------------------- WORD LEARNING FUNCTIONS
# @todo make methods of a class
class ShapeLearnerManager:
    def __init__(self, generateSettingsFunction):
        self.generateSettings = generateSettingsFunction;
        self.shapesLearnt = [];
        self.shapeLearners_all = [];
        self.shapeLearners_currentCollection = [];
        self.settings_shapeLearners_all = [];
        self.settings_shapeLearners_currentCollection = [];
        self.currentCollection = [];
        self.collectionsLearnt = [];
        self.nextShapeLearnerToBeStarted = 0;
        
        
    def initialiseShapeLearners(self):
        self.shapeLearners_currentCollection = [];
        self.settings_shapeLearners_currentCollection = [];
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
                self.shapeLearners_all.append(shapeLearner);
                self.settings_shapeLearners_all.append(settings);
                self.shapeLearners_currentCollection.append(self.shapeLearners_all[-1]);
                self.settings_shapeLearners_currentCollection.append(self.settings_shapeLearners_all[-1]);
                
            else:
                #use the bounds determined last time
                previousBounds = self.shapeLearners_all[shapeType_index].getParameterBounds();
                newInitialBounds = previousBounds;
                newInitialBounds[0,0] -= boundExpandingAmount;#USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
                newInitialBounds[0,1] += boundExpandingAmount;#USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
                self.shapeLearners_all[shapeType_index].setParameterBounds(newInitialBounds);
                self.shapeLearners_currentCollection.append(self.shapeLearners_all[shapeType_index]);
                self.settings_shapeLearners_currentCollection.append(self.settings_shapeLearners_all[shapeType_index]);
        
                     
    def startNextShapeLearner(self):
        #start learning
        if( self.nextShapeLearnerToBeStarted < len(self.currentCollection) ):
            shapeType = self.currentCollection[self.nextShapeLearnerToBeStarted];
            shapeType_code = self.nextShapeLearnerToBeStarted;
            shape_index = self.indexOfShapeInCurrentCollection(shapeType);
            [shape, paramValue] = self.shapeLearners_currentCollection[shape_index].startLearning();
            paramToVary = self.settings_shapeLearners_currentCollection[shape_index].paramsToVary[0]; #USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
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
                self.shapeLearners_currentCollection[shapeIndex_messageFor].respondToFeedback(bestShape_index);
                return 1;
            else:               
                [numItersConverged, newShape, newParamValue] = self.shapeLearners_currentCollection[shapeIndex_messageFor].generateNewShapeGivenFeedback(bestShape_index);
            paramToVary = self.settings_shapeLearners_currentCollection[shapeIndex_messageFor].paramsToVary[0];
            return numItersConverged, newShape, shape_messageFor, shapeIndex_messageFor, paramToVary, newParamValue;#USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
    
    def respondToDemonstration(self, shapeIndex_messageFor, shape):
        shape_messageFor = self.shapeAtIndexInAllShapesLearnt(shapeIndex_messageFor);
        if(shape_messageFor < 0 ):
            print('Ignoring demonstration because not for valid shape type');
            return -1;
        else:
            return self.shapeLearners_currentCollection[shapeIndex_messageFor].respondToDemonstration(shape);
    
    def indexOfShapeInCurrentCollection(self, shapeType):
        try:
            shapeType_index = self.currentCollection.index(shapeType);
        except ValueError: #unknown shape
            shapeType_index = -1;
        return shapeType_index;
    
    def indexOfShapeInAllShapesLearnt(self, shapeType):
        try:
            shapeType_index = self.shapesLearnt.index(shapeType);
        except ValueError: #unknown shape
            shapeType_index = -1;
        return shapeType_index;
            
    def shapeAtIndexInCurrentCollection(self, shapeType_index):
        try:
            shapeType = self.currentCollection[shapeType_index];
        except ValueError: #unknown shape
            shapeType = -1;
        return shapeType;
        
    def shapeAtIndexInAllShapesLearnt(self, shapeType_index):
        try:
            shapeType = self.shapesLearnt[shapeType_index];
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
        currentBounds = self.shapeLearners_currentCollection[shapeType_index].getParameterBounds();
               
        #change bounds back to the initial ones 
        newBounds = self.shapeLearners_currentCollection[shapeType_index].initialBounds;
        self.shapeLearners_currentCollection[shapeType_index].setParameterBounds(newBounds);
        print('Changing bounds on shape '+self.shapeAtIndexInCurrentCollection(shapeType_index)+' from '+str(currentBounds)+' to '+str(newBounds));
    
    def generateSimulatedFeedback(self, shapeType_index, newShape, newParamValue):
        return self.shapeLearners_currentCollection[shapeType_index].generateSimulatedFeedback(newShape, newParamValue);
