import cv2
import numpy as np


def detectShadow(image):
    imageLuv = cv2.cvtColor(image, cv2.COLOR_BGR2LUV)
    segmentResult=cv2.pyrMeanShiftFiltering(imageLuv, 9, 15, None, 2)
    
    pass


def removeShadow():
    pass