#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from webots_ros.srv import set_float
from pupil_ros.msg import gaze_positions
import webots_ros
from std_msgs.msg import String

ROBOT_NODE_NAME = 'undefined'

#-------------------------------------------------------------

def setName(data):
    ROBOT_NODE_NAME = data;
    print ('name set to: '+ str(ROBOT_NODE_NAME))
    rospy.set_param('robot_name', str(ROBOT_NODE_NAME))
    if (ROBOT_NODE_NAME != 'undefined'):
        rospy.signal_shutdown('read name')
        return


def reader():
    rospy.init_node('reader')
    rospy.Subscriber("model_name",String,setName)
    rospy.spin()

if __name__=='__main__':
        print "nameReader"
        reader()
        print 'NameReader off'
