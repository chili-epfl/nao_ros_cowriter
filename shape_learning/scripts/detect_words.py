#!/usr/bin/env python
'''
Publish word to write based on chilitag detected.
'''

import rospy
import tf
from std_msgs.msg import String, Empty

tags_words_mapping = {'tag_5':'cow', 'tag_6':'son','tag_7':'cue','tag_8':'new','tag_9':'use','tag_10':'test','tag_11':'stop'};

WORDS_TOPIC = 'words_to_write';
STOP_TOPIC = 'stop_learning';
TEST_TOPIC = 'test_learning';

pub_words = rospy.Publisher(WORDS_TOPIC, String)
pub_stop = rospy.Publisher(STOP_TOPIC, Empty)
pub_test = rospy.Publisher(TEST_TOPIC, Empty)
if __name__=="__main__":
    rospy.init_node("word_detector")    

    tf_listener = tf.TransformListener(True, rospy.Duration(1))
    rospy.sleep(.5)
    rate = rospy.Rate(10)
    prevTagDetected = [];
    while not rospy.is_shutdown():
        #print('searching');
        for tag in tags_words_mapping:
            
            try:
                tf_listener.waitForTransform("v4l_frame", tag, rospy.Time.now(), rospy.Duration(0.1))
                t = tf_listener.getLatestCommonTime("v4l_frame", tag)
                (trans,rot) = tf_listener.lookupTransform("v4l_frame", tag, t)
                tagDetected = True;
                print('found tag');
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                tagDetected = False;
            '''
            tagDetected = tf_listener.frameExists(tag);  
            if(tagDetected):
                print('Found a tag');
                pass
            ''' 
            if(tagDetected and not tag==prevTagDetected):
                prevTagDetected = tag;
                wordToPublish = tags_words_mapping[tag];
                print(wordToPublish);
                if(wordToPublish == 'stop'):
                    message = Empty();
                    pub_stop.publish(message);
                elif(wordToPublish == 'test'):
                    message = Empty();
                    pub_test.publish(message);
                else:
                    message = String();
                    message.data = wordToPublish;
                    pub_words.publish(message); 

    rate.sleep()
