#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from std_msgs.msg import String

ROBOT_NODE_NAME = 'undefined'

def setName(data):
#   String published by Webots has certain structure.
#   Has to be split to get to proper name
    read_string = str(data);
    ROBOT_NODE_NAME = read_string.split('\"')[1];
    print ('robot name set to: '+ ROBOT_NODE_NAME)
    rospy.set_param('robot_name', str(ROBOT_NODE_NAME))

#   if name was changed and param was set, shut ROS node down
    if (ROBOT_NODE_NAME != 'undefined' and rospy.get_param('robot_name') == ROBOT_NODE_NAME):
        rospy.signal_shutdown('read name correctly')
        return


def reader():
    rospy.init_node('reader')
    rospy.Subscriber("model_name",String,setName)
    rospy.spin()

if __name__=='__main__':
        print "nameReader on"
        reader()
        print 'NameReader off'
