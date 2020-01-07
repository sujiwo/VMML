#!/usr/bin/python

import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from vision_mapper.srv import place_recognizer

bridge = CvBridge()


# XXX: The server may need proper image size
def runPlaceRecognizer(inputFilePath):
    image = cv2.imread(inputFilePath)
    msg = bridge.cv2_to_imgmsg(image, "bgr8")
    server = rospy.ServiceProxy('place_recognizer', place_recognizer)
    placeResp = server(msg)
    return placeResp.keyframeId


if __name__=='__main__':
    res = runPlaceRecognizer(sys.argv[1])
    print(res)
    pass
