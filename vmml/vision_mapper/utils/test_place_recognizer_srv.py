#!/usr/bin/python2

import sys
import os
import pickle
from time import time
import rospy
import cv2
from copy import copy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from vision_mapper.srv import place_recognizer

bridge = CvBridge()

keyframeDir = "./test_dir2/keyframes/"

def runPlaceRecognizerImg(imageArray):
    msg = bridge.cv2_to_imgmsg(imageArray, "bgr8")
    t1 = time()
    server = rospy.ServiceProxy('place_recognizer', place_recognizer)
    placeResp = server(msg)
    t2 = time()
    timeRun = t2-t1
    return placeResp.keyframeId, timeRun


# XXX: The server may need proper image size
def runPlaceRecognizer(inputFilePath):
    image = cv2.imread(inputFilePath)
    msg = bridge.cv2_to_imgmsg(image, "bgr8")
    t1 = time()
    server = rospy.ServiceProxy('place_recognizer', place_recognizer)
    placeResp = server(msg)
    t2 = time()
    timeRun = t2-t1
    return placeResp.keyframeId, timeRun

def combineImage(src1, src2, src2Id):
    src2Id = str(src2Id)
    cv2.putText(src2, src2Id, (30,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))
    if (src1.shape==(0,0)):
        return copy(src2)
    else:
        return cv2.hconcat([src1, src2])
        pass
    pass


if __name__=='__main__':
#     res = runPlaceRecognizer(sys.argv[1])
#     print(res)

    testResults = {}
    
    for i in range(1, len(sys.argv)):
        queryId = int(os.path.basename(os.path.splitext(sys.argv[i])[0]))
        pathId = sys.argv[i]
        
        qResult, timeRun = runPlaceRecognizer(pathId)
#         testResults[queryId] = qResult
        
#         evaluateImage = np.zeros((0,0), dtype=np.uint8)
#         qImage = cv2.resize(cv2.imread(pathId), (800,450))
#         evaluateImage = combineImage(evaluateImage, qImage, "Query") 
#         
#         for qr in qResult:
#             kfPath = keyframeDir + str(qr) + ".png"
#             kfImage = cv2.resize(cv2.imread(kfPath), (800,450))
#             evaluateImage = combineImage(evaluateImage, kfImage, qr)
#         cv2.imwrite(str(queryId)+".png", evaluateImage)
        testResults[queryId]={'id': queryId, 'ans': qResult, 'time': timeRun}
        print(queryId, len(qResult), timeRun)
        print(qResult)
    
    pickle.dump(testResults, open("place_recognizer_results.pickle", "wb"))
    pass
