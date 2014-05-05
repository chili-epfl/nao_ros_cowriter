#!/usr/bin/env python

from display_manager.srv import *
import rospy
from shape_display_manager import ShapeDisplayManager

def handle_clear_all_shapes(request):
    shapeDisplayManager.clearAllShapes();
    print('Shapes cleared');
    response = clearAllShapesResponse();
    response.success.data = True; #probably not necessary
    return response;
    
def handle_display_new_shape(request):
    response = displayNewShapeResponse();

    location = shapeDisplayManager.displayNewShape(request.shape_type_code);
    response.location.x = location[0];
    response.location.y = location[1];
    
    print('Shape added');
    return response;
    
def handle_shape_at_location(request):
    response = shapeAtLocationResponse();
    location = [request.location.x, request.location.y];
    [response.shape_type_code, response.shape_id] = shapeDisplayManager.shapeAtLocation(location);
    print('Shape location returned');
    return response;

def handle_possible_to_display(request):
    response = isPossibleToDisplayNewShapeResponse();
    response.is_possible.data = shapeDisplayManager.isPossibleToDisplayNewShape(request.shape_type_code);
    print('If possible returned');
    return response;

def display_manager_server():
    rospy.init_node('display_manager_server')
    clear_service = rospy.Service('clear_all_shapes', clearAllShapes, handle_clear_all_shapes)
    print "Ready to clear all shapes."
    
    display_shape_service = rospy.Service('display_new_shape', displayNewShape, handle_display_new_shape)
    print "Ready to display new shapes."
    
    display_shape_service = rospy.Service('shape_at_location', shapeAtLocation, handle_shape_at_location)
    print "Ready to determine shape at location."
    
    possible_to_display_service = rospy.Service('possible_to_display_shape', isPossibleToDisplayNewShape, handle_possible_to_display)
    print "Ready to determine is shape fits."
    rospy.spin()

if __name__ == "__main__":
    shapeDisplayManager = ShapeDisplayManager();
    
    display_manager_server()
    print('shut down');
