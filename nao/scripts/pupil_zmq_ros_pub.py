#!/usr/bin/env python2.7

#Pupil_ZMQ_ROS Publisher
#ZMQ subscriber to receive gaze and world
#Start ROS node to publish gaze_positions, and world image
#Using customized ROS msg: gaze_positions, gaze

#Standard imports
import zmq
import sys
import roslib
roslib.load_manifest('nao')
import rospy
import numpy as np
import cv2
#Specific imports
from msgpack import loads
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nao.msg import gaze_positions, gaze
from std_msgs.msg import Header
from geometry_msgs.msg import Point

#Convert the first three elements of a tuple to ROS Geometry Point Message
def tupleToPoint(tup):
    p = Point()
    if tup:
        l = len(tup)
        if l == 0:
            return p
        elif l == 1:
            p.x = tup[0]
            return p
        elif l == 2:
            p.x, p.y = tup[0], tup[1]
        else:
            p.x, p.y, p.z = tup[0], tup[1], tup[2]
    return p

#Pupil_ZMQ_ROS: a standalone class to interface Pupil ZMQ messages and ROS environment.
class Pupil_ZMQ_ROS(object):
    def __init__(self, addr='localhost', req_port='50020'): #Port can change
        #Initialize Pupil_ZMQ_ROS object, init ZMQ and ROS
        self.zmq_req = None
        self.zmq_sub = None
        self.ros_gaze_publisher = None
        self.ros_pupil_publisher = None
        self.ros_world_img_publisher = None
        self.cv_bridge = CvBridge()
        self.ros_started = True
        self.seq = 0
        self.init_zmq(addr, req_port)
        self.init_ros()


    #Initialize ZMQ subscriber
    def init_zmq(self, addr, req_port):
        context = zmq.Context()
        self.zmq_req = context.socket(zmq.REQ)
        self.zmq_req.connect("tcp://%s:%s" %(addr,req_port))
        
        # ask for the sub port
        self.zmq_req.send('SUB_PORT')
        sub_port = self.zmq_req.recv()
        
	# open a sub port to listen to pupil
        self.zmq_sub = context.socket(zmq.SUB)
        self.zmq_sub.connect("tcp://%s:%s" %(addr,sub_port))

	# set zmq subscriptions to topics
        self.zmq_sub.setsockopt(zmq.SUBSCRIBE, 'gaze')
        self.zmq_sub.setsockopt(zmq.SUBSCRIBE, 'frame')
        print ('Pupil_ZMQ_ROS: zmq environment initialized')

    #Initialize ROS node, four publishers on four topics: gaze, pupil, world and surface
    def init_ros(self):
        try:
            rospy.init_node('Pupil_ZMQ_ROS', anonymous=True)
            self.ros_gaze_publisher = rospy.Publisher('/pupil_capture/gaze', gaze_positions, queue_size=10)
            self.ros_world_img_publisher = rospy.Publisher('/pupil_capture/world', Image, queue_size=2)
            self.ros_started = True
            self.seq = 0
            print ('Pupil_ZMQ_ROS: ros environment initialized')
        
        #ROS except
        except rospy.ROSInterruptException as e:
            self.ros_started = False
            self.seq = 0
            print ('Pupil_ZMQ_ROS: unable to start ros node:'+ e)

    #Spinning loop: receive ZMQ data and publish to ROS topics
    def spin(self):
        print 'spin'
        if not self.ros_started:
            print ('Pupil_ZMQ_ROS: ros not started')
            return
        
        # rospy.is_shutdown check inside while loop to enable Ctrl-C termination    
        while True:
            if rospy.is_shutdown():
                break
                
            # receive message from ZMQ subscriber
            zmq_multipart =  self.zmq_sub.recv_multipart()
            zmq_topic, zmq_raw_msg = zmq_multipart[0], zmq_multipart[1]
            
            # ROS header message
            header = Header()
            header.seq = self.seq
            header.stamp = rospy.get_rostime()
            header.frame_id = "Pupil_ZMQ_ROS"
            zmq_msg = loads(zmq_raw_msg, strict_map_key=False)

            #Gaze data after combining pupil data and gaze mapping plugin
            if 'gaze' in zmq_topic :
                gaze_msg = gaze_positions()
                gaze_info_list = []
                gaze_info = gaze()
                gaze_info.confidence = zmq_msg['confidence']
                gaze_info.norm_pos = tupleToPoint(zmq_msg.get('norm_pos'))
                gaze_info.gaze_point_3d = tupleToPoint(zmq_msg.get('gaze_point_3d'))
                gaze_info.gaze_normal_3d = tupleToPoint(zmq_msg.get('gaze_normal_3d'))
                gaze_info.eye_center_3d = tupleToPoint(zmq_msg.get('eye_center_3d'))
                gaze_info.pupil_timestamp = zmq_msg['timestamp']
                gaze_info_list.append(gaze_info)
                gaze_msg.gazes = gaze_info_list
                gaze_msg.header = header
                self.ros_gaze_publisher.publish(gaze_msg)

            #Scan for topic name:frame.world    
            if 'frame.world' in zmq_topic:
                if zmq_msg['format'] == 'bgr':
                    cv_img = np.frombuffer(zmq_multipart[2], dtype=np.uint8).reshape( zmq_msg['height'], zmq_msg['width'], 3)
                    world_image_msg = self.cv_bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                    world_image_msg.header = header
                    self.ros_world_img_publisher.publish(world_image_msg)

        # Disable ROS interface
        self.ros_started = False


#Main spin
if __name__ == "__main__":
    zmq_ros_pub = None
    if len(sys.argv) >= 3:
        zmq_ros_pub = Pupil_ZMQ_ROS(sys.argv[1], sys.argv[2])
    elif len(sys.argv) == 2:
        zmq_ros_pub = Pupil_ZMQ_ROS(sys.argv[1])
    else:
        zmq_ros_pub = Pupil_ZMQ_ROS()

    
    # Spinning on ZMQ messages, terminated by Ctrl-C
    zmq_ros_pub.spin()

