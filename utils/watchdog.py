#!/usr/bin/env python

"""Classes for managing watchdog timers over ROS topics.
"""

import rospy
from std_msgs.msg import Empty
from threading import Timer

class Watchdog:
    """Listens for watchdog timer clears over a topic and if the timer 
    overflows then a warning is raised that device responsible for 
    clearing the timer has not performed as expected.
    """
    def __init__(self, clearTopic, timeout_sec, userHandler=None):
        self.clearTopic = clearTopic;
        self.timeout_sec = timeout_sec;
        self.handler = userHandler if userHandler is not None else self.defaultHandler;
        self.subscriber = rospy.Subscriber(self.clearTopic, Empty, self.onClear);
        
        print('Starting new timer');
        self.responsive = True;
        self.timer = Timer(self.timeout_sec, self.handler);
        self.timer.start();
    
    def onClear(self, message):
        self.timer.cancel(); #clear timer
        
        if(not self.responsive):
            print('Re-connection');
        self.responsive = True;
        self.timer = Timer(self.timeout_sec, self.handler); #schedule new timeout
        self.timer.start();
    
    def defaultHandler(self):
        print('Haven\'t received a clear on \'' + self.clearTopic + '\' topic');
        self.responsive = False;
        
    def isResponsive(self):
        return self.responsive
        
class WatchdogClearer:
    """Publishes watchdog timer clears over a topic.
    """    
    def __init__(self, clearTopic, timeBetweenClears_sec):#TO DO: make timer work with ms precision...
        self.clearTopic = clearTopic;
        self.timeBetweenClears_sec = timeBetweenClears_sec;
        self.publisher = rospy.Publisher(self.clearTopic, Empty);
        
        print('Starting new timer');
        self.timer = Timer(self.timeBetweenClears_sec, self.clearWatchdog);
        self.timer.start();
    
    def clearWatchdog(self):
        self.timer.cancel();
        self.publisher.publish(Empty()); #clear watchdog
        self.timer = Timer(self.timeBetweenClears_sec, self.clearWatchdog); #schedule next clear
        self.timer.start();
    
    def stop(self):
        self.timer.cancel();
        print('Stopping publishing clears');
        
    def restart(self):
        print('Restarting publishing clears');
        self.timer = Timer(self.timeBetweenClears_sec, self.clearWatchdog);
        self.timer.start();
