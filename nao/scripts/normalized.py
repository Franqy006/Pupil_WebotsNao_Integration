#! /usr/bin/env python
import sys
import rospy
from webots_ros.srv import set_float
from nao.msg import gaze_positions
from std_msgs.msg import String

try:
    ROBOT_NODE_NAME = rospy.get_param('robot_name')
except:
    print ('ERROR:robot_name rosparam read did not work')

moveRightArm = rospy.ServiceProxy(ROBOT_NODE_NAME+'/RShoulderPitch/set_position', set_float)
moveLeftArm = rospy.ServiceProxy(ROBOT_NODE_NAME+'/LShoulderPitch/set_position', set_float)

def callback(data):
    global id2
    global id1
    id2 = data.header.seq
#   only one input per 15 causes action
    if (id2-id1)<15:
        return
    y = data.gazes[0].norm_pos.y
    x = data.gazes[0].norm_pos.x
    if x<0.2:
        moveLeftArm(-2+(1+2*y))
        print("moving left arm to:"+ str(-2+(1+2*y)))
    if x>0.5:
        moveRightArm(-1+2*y)
        print("moving right arm to:"+ str(-1+2*y))
    id1 = id2

def listener():
    rospy.init_node('normalized_control')
    rospy.Subscriber("pupil_capture/gaze",gaze_positions,callback)
    rospy.spin()

if __name__=='__main__':
        print "normalized control start"
        global id1
        id1 = 0
        global id2
        id2 = 0
        listener()
