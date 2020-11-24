#! /usr/bin/env python
import sys
import rospy
from webots_ros.srv import set_float
from pupil_ros_plugin.msg import gaze_positions
import webots_ros
from std_msgs.msg import String

read_name = rospy.get_param('robot_name')
ROBOT_NODE_NAME = read_name.split('\"')[1]
print ROBOT_NODE_NAME
ruszR = rospy.ServiceProxy(ROBOT_NODE_NAME+'/RShoulderPitch/set_position', set_float)
ruszL = rospy.ServiceProxy(ROBOT_NODE_NAME+'/LShoulderPitch/set_position', set_float)
#-------------------------------------------------------------
def callback(data):
    global id2
    global id1
    id2 = data.header.seq
#    print str(id2-id1)
    if (id2-id1)<15:
        return
    y = data.gazes[0].norm_pos.y
    x = data.gazes[0].norm_pos.x
    if x<0.2:
        ruszL(-2+(1+2*y))
        print("ruch lewa"+ str(-2+(1+2*y)))
    if x>0.5:
        ruszR(-1+2*y)
        print("ruch prawa"+ str(-1+2*y))
    id1 = id2

def listener():
    rospy.init_node('sterownik_nao')
    rospy.Subscriber("pupil_capture/gaze",gaze_positions,callback)
    rospy.spin()

if __name__=='__main__':
        print "start sterownika"
        global id1
        id1 = 0
        global id2
        id2 = 0
        listener()
