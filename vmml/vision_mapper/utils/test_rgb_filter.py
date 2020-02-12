#!/usr/bin/python

import rospy
import cv_bridge
import cv2
from sensor_msgs.msg import Image
from copy import copy
import numpy as np


imageSub = None
publisher = None
bridge = None
_alpha = 0.3975


def imageConvert(imageRgb):
    alpha=0.3975
    imgf = np.array(imageRgb, dtype=np.float32) / 255.0
    #                    green                     blue                                red
    imgf = 0.5 + np.log(imgf[:,:,1]) - alpha*np.log(imgf[:,:,0]) - (1-alpha)*np.log(imgf[:,:,2])
    return np.array(imgf*255.0, dtype=np.uint8)

def imageConvertBayer(imageBayer, alpha):
    height=imageBayer.shape[0]/2
    width=imageBayer.shape[1]/2
    grey=np.zeros((height, width), dtype=np.float32)
    for i in range(height):
        for j in range(width):
            pr=i*2
            pc=j*2
            g1=float(imageBayer[pr,pc])/255.0
            r=float(imageBayer[pr,pc+1])/255.0
            b=float(imageBayer[pr+1,pc])/255.0
            g2=float(imageBayer[pr+1,pc+1])/255.0
            g=(g1+g2)/2.0
            iv = 0.5 + np.log10(g) - alpha*np.log10(b) - (1-alpha)*np.log10(r)
            grey[i,j] = iv
    # Normalize
    grey = np.clip(grey, 0.0, 1.0)
    return np.array(grey*255, dtype=np.uint8)


def Retinex(image, sigma, gain, offset):
    fA = image.astype(np.float32)
    min_nonzero = np.min(fA[np.nonzero(fA)])
    fA[fA==0.0] = min_nonzero
    fB = np.log(fA)
    
    A = copy(image)
    cv2.GaussianBlur(A, (0,0), sigma, A)
    fA = A.astype(np.float32)
    fC = np.log(fA)
    
    fA = fB - fC
    # XXX: need better formula
    return cv2.convertScaleAbs(fA, None, gain, offset)


def MultiScaleRetinexColorRestoration(image):
    fA = image
    pass

def imageHandler(imageMsg):
    cImage = bridge.imgmsg_to_cv2(imageMsg, 'passthrough')
    cImage = imageConvertBayer(cImage, _alpha)
    msg = bridge.cv2_to_imgmsg(cImage, 'mono8')
    msg.header.stamp=imageMsg.header.stamp
    publisher.publish(msg)
    pass


# For publishing images to ROS
# if __name__=='__main__':
# 
#     rospy.init_node("imgprocx", anonymous=True)
# 
#     imageSub = rospy.Subscriber("/front_rgb/image_raw", Image, imageHandler, queue_size=1)
#     publisher = rospy.Publisher("/front_rgb/image_rgb", Image, queue_size=1)
#     bridge = cv_bridge.CvBridge()
# 
#     rospy.spin()


# For testing directly
if __name__=='__main__':
    
    image=cv2.imread('/home/sujiwo/VmmlWorkspace/test_dir2/queries/14539.png')
    imageRetx = Retinex(image, 10, 256, 256)
    
    pass


