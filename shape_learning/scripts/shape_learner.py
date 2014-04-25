"""
Class to use feedback to learn optimal parameters for a shape modeled by
an associated ShapeModeler.

Depends on shape_modeler and recordtype.
"""

import bisect
import numpy
from shape_modeler import ShapeModeler

#shape learning parameters
maxNumAttempts = 1000;      #Allowed attempts to draw a new shape which is significantly different to the previous one but still within the range (just a precaution; sampling should be theoretically possible)
tol = 1e-2;                 #Tolerance on convergence test


from recordtype import recordtype #for mutable namedtuple (dict might also work)

class ShapeLearner:
    SettingsStruct = recordtype('SettingsStruct', 
    ['shape_learning',  #String representing the shape which the object is learning
    'paramToVary',      #Natural number between 1 and number of parameters in the associated ShapeModeler, representing the parameter to learn
    'doGroupwiseComparison', #instead of pairwise comparison with most recent two shapes 
    'initialBounds',    #Initial acceptable parameter range
    'minParamDiff']);   #How different two shapes' parameters need to be to be published for comparison
    #@todo: make groupwise comparison/pairwise comparison different implementations of shapeLearner class

    def __init__(self, shapeModeler, settings):
        self.shapeModeler = shapeModeler;
        self.doGroupwiseComparison = settings.doGroupwiseComparison;
        self.shape_learning = settings.shape_learning;
        self.paramToVary = settings.paramToVary;
        self.minParamDiff = settings.minParamDiff;
        
        self.converged = False;
        self.numIters = 0;
                        
### ----------------------------------------------------- START LEARNING        
    def startLearning(self, startingBounds):
        self.bounds = startingBounds;
        
        #make initial shape
        [shape, paramValue] = self.shapeModeler.makeRandomShapeFromUniform(self.paramToVary, self.bounds);
        self.bestParamValue = paramValue;
        print('Bounds: '+str(self.bounds));
        print('Test param: '+str(self.bestParamValue));
        if(self.doGroupwiseComparison):
            self.params_sorted = [self.bounds[0], self.bounds[1]];
            bisect.insort(self.params_sorted, self.bestParamValue);
            self.shapeToParamMapping = [self.bestParamValue];
        else:
            self.newParamValue = paramValue;
        
        return shape, paramValue;
 
### ----------------------------------------------- MAKE DIFFERENT SHAPE        
    def makeShapeDifferentTo(self, paramValue):
        #make new shape to compare with
        [newShape, newParamValue] = self.shapeModeler.makeRandomShapeFromTriangular(self.paramToVary, self.bounds, self.bestParamValue);
        #ensure it is significantly different
        numAttempts = 1;
        while(abs(newParamValue - paramValue) < self.minParamDiff and numAttempts < maxNumAttempts):
            [newShape, newParamValue] = self.shapeModeler.makeRandomShapeFromTriangular(self.paramToVary, self.bounds, self.bestParamValue);
            numAttempts+=1;      
        
        if(numAttempts>=maxNumAttempts): #couldn't find a 'different' shape in range
            print('Oh no!'); #this should be prevented by the convergence test below
        
        #store it as an attempt
        if(self.doGroupwiseComparison):
            bisect.insort(self.params_sorted, newParamValue);
            self.shapeToParamMapping.append(newParamValue);
            
        return newShape, newParamValue   

### ---------------------------------------- GENERATE SIMULATED FEEDBACK
    def generateSimulatedFeedback(self, shape, newParamValue):
        #code in place of feedback from user: go towards goal parameter value
        goalParamValue =  numpy.float64(0);#-1.5*parameterVariances[self.paramToVary-1];
        if(self.doGroupwiseComparison):
            errors = numpy.ndarray.tolist(abs(self.shapeToParamMapping-goalParamValue));
            bestShape_idx = errors.index(min(errors));
        else:   
            errors = [abs(self.bestParamValue- goalParamValue), abs(newParamValue- goalParamValue)];
            bestShape_idx = errors.index(min(errors));
        return bestShape_idx        

### ------------------------------------------------ RESPOND TO FEEDBACK                
    def respondToFeedback(self, bestShape):
        #update bestParamValue based on feedback received
        if(self.doGroupwiseComparison):
            self.bestParamValue = self.shapeToParamMapping[bestShape];
            print('Chosen param value: ' + str(self.bestParamValue));
            bestParamValue_index = bisect.bisect(self.params_sorted,self.bestParamValue) - 1; #indexing seems to start at 1 with bisect
            self.bounds = [self.params_sorted[bestParamValue_index-1],self.params_sorted[bestParamValue_index+1] ];
            #restrict bounds if they were caused by other shapes, because it must be sufficiently different to said shape(s)
            if((bestParamValue_index-1) > 0): #not the default min
                self.bounds[0] += self.minParamDiff;
            if((bestParamValue_index+1) < (len(self.params_sorted)-1)): #not the default max
                self.bounds[1] -= self.minParamDiff;

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

        
### ------------------------------------------------------------ ITERATE      
    def generateNewShapeGivenFeedback(self, bestShape):        
        #------------------------------------------- respond to feedback
        self.respondToFeedback(bestShape); #update bounds and bestParamValue
        
        #----------------------------------------- check for convergence
        #continue if there are more shapes to try which are different enough
        if((abs(self.bounds[1]-self.bestParamValue)-self.minParamDiff < tol) and (abs(self.bestParamValue-self.bounds[0])-self.minParamDiff) < tol):
            self.converged = True;
        
        #-------------------------------------------- continue iterating
        self.numIters+=1;
               
        #try again if shape is not good enough
        if(not self.converged):
            [newShape, newParamValue] = self.makeShapeDifferentTo(self.bestParamValue);
            self.newParamValue = newParamValue;
            print('Bounds: '+str(self.bounds));
            print('Test param: '+str(newParamValue));         
            return self.converged, newShape, newParamValue
            
        else:          
            print('Converged');  
            return self.converged, bestShape, self.bestParamValue    
            
    def getParameterBounds(self):
        return self.bounds;
