import cv2
import rosbag
import cv_bridge


def convert(bagpath, topic, fps, outputpath):
    
    mbag = rosbag.Bag(bagpath, mode='r')
    fccx = cv2.VideoWriter_fourcc('X','2','6','4')
    video = None
    brg = cv_bridge.CvBridge()
#     video = cv2.VideoWriter(outputpath, fccx, fps, )
    
    for topic, msg, tm in mbag.read_messages(topics=[topic]):
        
        image = brg.imgmsg_to_cv2(msg, 'bgr8');
        
        if (video is None):
            video = cv2.VideoWriter(outputpath, fccx, fps, (image.shape[1], image.shape[0]))
            
        video.write(image)
        
    video.release()    
    return
