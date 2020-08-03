import multiprocessing
from math import ceil, floor
import cv2
import rosbag
import cv_bridge
import ying
import RandomAccessBag
from twisted.internet.test.reactormixins import process


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


def _workerProcess(image):
    image_enh = ying.Ying_2017_CAIP(image)
    return image_enh

def enhanceRawBag(bagInputPath, bagOutputPath, hz=0):
    global _imageBag, _bridge
    
    rdBagList=RandomAccessBag.RandomAccessBag.getAllConnections(bagInputPath)
    for c in rdBagList:
        if c.type()=="sensor_msgs/Image":
            imageBag = c
            break
    if hz>0:
        imageBag.desample(hz)
    bridge = cv_bridge.CvBridge()
    bagWriter = RandomAccessBag.rosbag.Bag(bagOutputPath, mode="w")

    c = multiprocessing.cpu_count()/2    
    processor=multiprocessing.Pool(processes=c)
    processor.num=c
    
    for n in range( int(ceil(float(len(imageBag)) / float(processor.num))) ):
        targets = range(n*processor.num, min((n+1)*processor.num, len(imageBag)))
        images = [bridge.imgmsg_to_cv2(imageBag[t], "bgr8") for t in targets]
        waiter = processor.map_async(_workerProcess, images)
        waiter.wait()
        results = waiter.get()
        for res in range(len(results)):
            imgmsg = bridge.cv2_to_imgmsg(results[res], "bgr8")
            t = imageBag.messageTime(targets[res])
            bagWriter.write(imageBag.topic(), imgmsg, t)

    bagWriter.close()
    return



if __name__=="__main__":
    enhanceRawBag("/tmp/sample-original.bag", "/tmp/sample-enhanced-ying.bag")
    
    pass


