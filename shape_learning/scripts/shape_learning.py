#!/usr/bin/env python

"""
Reads data from 'file' argument, publishes shapes on the 
/shapes_to_draw topic and adapt them based on feedback received on the 
/shape_feedback topic'
"""

import rospy
import numpy
import pdb
import matplotlib.pyplot as plt

from nav_msgs.msg import Path
import time
from shape_modeler import ShapeModeler

paramToVary = 4;            #Natural number between 1 and numPrincipleComponents, representing which principle component to vary from the template
numPrincipleComponents = 5; #Number of principle components to keep during PCA of dataset
maxNumAttempts = 500;       #Allowed attempts to draw a new shape which is significantly different to the previous one but still within the range
diffThresh = 0.15;           #How different two shapes' parameters need to be to be published for comparison
stdDevMultiples = [-2, 4];  #Starting bounds for paramToVary, as multiples of the parameter's observed standard deviation in the dataset
simulatedFeedback = True;   #Simulate user feedback as whichever shape is closest to goal parameter value


###
    
tol = 1e-2;                 #Tolerance on convergence test
if __name__ == "__main__":

    import argparse
    parser = argparse.ArgumentParser(description='Publish shapes on the /shapes_to_draw topic and adapt them based on feedback received on the /shape_feedback topic');
    parser.add_argument('file', action="store",
                    help='a .txt file containing a dataset of the shape')
    parser.add_argument('--show', action='store_true', help='display plots of the shapes')

    args = parser.parse_args()
    
    shapeModeler = ShapeModeler();
    shapeModeler.makeDataMatrix(args.file);
    shapeModeler.performPCA(numPrincipleComponents);

    parameterVariances = shapeModeler.getParameterVariances();
    bounds = numpy.array(stdDevMultiples)*parameterVariances[paramToVary-1];
    
    if(args.show):
        plt.ion() #to plot one shape at a time

    #make initial shape
    [shape, bestParamValue] = shapeModeler.makeRandomShapeFromUniform(paramToVary, bounds);

    #publish shape and get feedback
    if(simulatedFeedback):
        #pretend that first one wasn't good enough
        converged = False;

    #while shape is not good enough
    while(not converged):
        
        #make new shape to compare with
        [newShape, newParamValue] = shapeModeler.makeRandomShapeFromTriangular(paramToVary, bounds, bestParamValue);
        #ensure it is significantly different
        numAttempts = 1;
        while(abs(newParamValue - bestParamValue) < diffThresh):# and numAttempts < maxNumAttempts):
            [newShape, newParamValue] = shapeModeler.makeRandomShapeFromTriangular(paramToVary, bounds, bestParamValue);
            numAttempts+=1;      
        
        if(numAttempts>=maxNumAttempts): #couldn't find a 'different' shape in range
            print('Oh no!'); #this should be prevented by the convergence test below
         
        #publish shape and get feedback
        if(simulatedFeedback):
            #temporary code in place of feedback from user: go towards goal parameter value
            if(args.show):
                plt.figure(1)
                ShapeModeler.normaliseAndShowShape(newShape);
                time.sleep(1.3)
            goalParamValue =  -1.5*parameterVariances[paramToVary-1];
            errors = [abs(bestParamValue- goalParamValue), abs(newParamValue- goalParamValue)];
            bestShape = errors.index(min(errors));

        #restrict limits
        if( bestShape == 1 ):   #new shape is better
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
            
    print('Converged')  
    if(args.show):      
        plt.show(block=True)
        
