#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
from webots_ros.srv import set_float
from nao.msg import gaze_positions
from numpy import array, rot90
from sensor_msgs.msg import Image
import cv2
import numpy
from cv_bridge import CvBridge
from numpy import mean, binary_repr, zeros
import math

try:
    ROBOT_NODE_NAME = rospy.get_param('robot_name')
except:
    print ('ERROR:robot_name rosparam read did not work')

MARKER_SIZE = 5
cv_bridge = CvBridge()

moveRightArm = rospy.ServiceProxy(ROBOT_NODE_NAME+'/RShoulderPitch/set_position', set_float)
moveLeftArm = rospy.ServiceProxy(ROBOT_NODE_NAME+'/LShoulderPitch/set_position', set_float)

prev = [0,0,0]
gaze_x = 0.01
gaze_y = 0.01
last_marker = 0

def rightArmUp():
    moveRightArm(-1)
def rightArmDown():
    moveRightArm(1.5)
def leftArmUp():
    moveLeftArm(-1)
def leftArmDown():
    moveLeftArm(1.5)

switcher = {
    1: rightArmDown,
    5: rightArmUp,
    11: leftArmDown,
    19: leftArmUp}

def make_move(id):
    global last_marker
    prev[0]=prev[1]
    prev[1]=prev[2]
    prev[2]=id
    func = switcher.get(id, lambda : 'wrong index')
    if(prev[0]==prev[1]==prev[2]):
        if(last_marker != id):
            print ("making move for marker_id = " + str(id))
            func()
            last_marker = id
        else:
            print ('marker did not change, no action requested')

class HammingMarker(object):
    def __init__(self, id, contours=None):
        self.id = id
        self.contours = contours

    def __repr__(self):
        return '<Marker id={} center={}>'.format(self.id, self.center)

    @property
    def center(self):
        if self.contours is None:
            return None
        center_array = mean(self.contours, axis=0).flatten()
        return (int(center_array[0]), int(center_array[1]))

    def draw_contour(self, img, color=(0, 255, 0), linewidth=5):
        cv2.drawContours(img, [self.contours], -1, color, linewidth)

    def highlite_marker(self, img, contour_color=(0, 255, 0), text_color=(255, 0, 0), linewidth=5):
        self.draw_contour(img, color=contour_color, linewidth=linewidth)
        cv2.putText(img, str(self.id), self.center, cv2.FONT_HERSHEY_SIMPLEX, 2, text_color)
        cv2.circle(img, self.center, 10, (0,255,255), 10)

    @classmethod
    def generate(cls):
        return HammingMarker(id=randint(4096))

    @property
    def id_as_binary(self):
        return binary_repr(self.id, width=12)

#Matrix of coordinates that should be black for the marker to be valid
BORDER_3_COORDINATES = [
    [0, 0], [0, 1], [0, 2], [0, 3], [0, 4],
    [1, 0], [1, 4],
    [2, 0], [2, 4],
    [3, 0], [3, 4],
    [4, 0], [4, 1], [4, 2], [4, 3], [4, 4],
]

#Array of found markers
FOUND_MARKERS = []


#Checks if a marker is valid and rotates it upright
def validate_3(marker):
    for crd in BORDER_3_COORDINATES:
        if marker[crd[0], crd[1]] != 0.0:
            raise ValueError('Border contians not entirely black parts.')

    rotation = 0
    if marker[3][1] == 0.0:
        rotation = 0
    elif marker[1][1] == 0.0:
        rotation = 1
    elif marker[1][3] == 0.0:
        rotation = 2
    elif marker[3][3] == 0.0:
        rotation = 3
    else:
        print('invalid!')
        raise ValueError('Not valid marker.')

    marker = rot90(marker, k=rotation)

    if marker[3][3] == 0.0 or marker[1][1] == 0.0 or marker[1][3] == 0.0:
         raise ValueError('Not valid marker.')

    return marker

#Calculate the marker id
def get_marker_id(marker):
    marker_id = 31-(marker[2][2]*(2**0) + marker[2][1]*(2**1) + marker[3][2]*(2**2) + marker[2][3]*(2**3) + marker[1][2]*(2**4))
    return marker, marker_id


#Detect the marker id's and coordinates in an image, using a threshold
def detect_markers(img, threshold, error):
    global last_marker
    image = cv_bridge.imgmsg_to_cv2(img, 'bgr8')
    height,width, _ = image.shape
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 10, 100)

    #Find contours in image
    contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[-2:]

    # We only keep the long enough contours
    min_contour_length = min(width, height) / 50
    contours = [contour for contour in contours if len(contour) > min_contour_length]
    warped_size = 25
    canonical_marker_coords = array(((0, 0),
                                     (warped_size - 1, 0),
                                     (warped_size - 1, warped_size - 1),
                                     (0, warped_size - 1)),
                                    dtype='float32')
    #Initialize
    markers_list = []
    FOUND_MARKERS = []

    #Loop through found contours
    for contour in contours:
        #Skip if contour is open or invalid length
        approx_curve = cv2.approxPolyDP(contour, len(contour) * 0.01, True)
        if not (len(approx_curve) == 4 and cv2.isContourConvex(approx_curve)):
            continue

        #Warp the contour to a square and convert to grayscale
        sorted_curve = array(cv2.convexHull(approx_curve, clockwise=False),
                             dtype='float32')
        persp_transf = cv2.getPerspectiveTransform(sorted_curve, canonical_marker_coords)
        warped_img = cv2.warpPerspective(image, persp_transf, (warped_size, warped_size))
        warped_gray = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)

        #Convert to binary matrix using a threshold. The binary matrix is a 5x5 matrix, where the first and last row and column must be 1 (= black)
        _, warped_bin = cv2.threshold(warped_gray, threshold, 255, cv2.THRESH_BINARY)
        marker = warped_bin.reshape(
            [MARKER_SIZE, int(warped_size / MARKER_SIZE), MARKER_SIZE, int(warped_size / MARKER_SIZE)]
        )
        marker = marker.mean(axis=3).mean(axis=1)
        marker[marker < 127] = 0
        marker[marker >= 127] = 1

        try:
            #Check if the marker is valid, and calculate the id of the marker
            marker = validate_3(marker)
            marker, marker_id = get_marker_id(marker)
            #Skip if marker has already been found
            if FOUND_MARKERS.__contains__(marker_id):
                raise ValueError('Marker already found.')
            else:
                FOUND_MARKERS.append(marker_id)

            #Add found marker to list of markers as HammingMarker obj
            markers_list.append(HammingMarker(id=marker_id, contours=approx_curve))
        except ValueError:
            continue

    for mark in markers_list:
        if(abs(((float(mark.center[0]+20))/(width))-gaze_x) < error and abs(1-( float(mark.center[1])/(height))-gaze_y) < error):
            print('posX ='+ str(mark.center[0]))
            print('posy ='+ str(mark.center[1]))
            make_move(mark.id)
            print('-----')

    return markers_list

def callback(data):
    detect_markers(data,50,0.09)

def gaze_callback(data):
    global gaze_x
    global gaze_y
    gaze_x = data.gazes[0].norm_pos.x
    gaze_y = data.gazes[0].norm_pos.y

def listener():
    rospy.init_node('marker_controler')
    rospy.Subscriber("pupil_capture/world",Image,callback)
    rospy.Subscriber("pupil_capture/gaze",gaze_positions,gaze_callback)
    rospy.spin()

if __name__=='__main__':
        print "marker control start"
        listener()
