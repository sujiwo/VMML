import rospy
import rosbag
import numpy as np
from bisect import bisect


class RandomAccessBag:
    """RandomAccessBag is a wrapper for ROS Bag that allows random access to any message within a single topic"""
    
    def __init__ (self, bagFd, topic, start_time=None, end_time=None):
#         assert(type(bagFd)==rosbag.bag.Bag)
        if (isinstance(bagFd, str)):
            self.bagFd = rosbag.Bag(bagFd, mode="r")
        else:
            assert(type(bagFd)==rosbag.bag.Bag)
            self.bagFd = bagFd
        for cn in self.bagFd._connections.values():
            if cn.topic==topic:
                self.connection = cn
                break
            
        startTimeR = self.bagFd._chunks[0].start_time
        if (isinstance(start_time, float) or isinstance(start_time, int)):
            start_time = startTimeR + rospy.Duration.from_sec(start_time)
        if (isinstance(end_time, float) or isinstance(end_time, int)):
            end_time = startTimeR + rospy.Duration.from_sec(end_time)
            
        self.entries = []
        for entry in self.bagFd._get_entries([self.connection], start_time, end_time):
            self.entries.append(entry)
        self.entries.sort(key=lambda en: en.time)
        self.timestamps = [ent.time for ent in self.entries]
        
    def topic(self):
        return self.connection.topic
    
    def type(self):
        return self.connection.datatype
        
    def __len__(self):
        return len(self.entries)
    
    def __getitem__(self, i):
        entry = self.entries[i]
        return self.bagFd._read_message(entry.position).message
    
    def messageTime(self, i):
        return self.timestamps[i]
    
    def hz(self):
        """Get message frequency"""
        dt = (self.entries[-1].time - self.entries[0].time).to_sec()
        return float(len(self.entries)) / dt
    
    def _getEntryAtDurationSecond(self, fSec):
        return self.entries[self._getIndexAtDurationSecond(fSec)]
    
    def _getIndexAtDurationSecond(self, fSec):
        tm = self.entries[0].time + rospy.Duration.from_sec(fSec)
        return bisect(self.timestamps, tm)
    
    def desample(self, hz, onlyMsgList=False):
        """Reduce frequency of the messages by removing redundant entries. May be inaccurate"""
        lengthInSeconds = (self.entries[-1].time - self.entries[0].time).to_sec()
        messages = []
        indices = []
        tInterval = 1.0 / float(hz)
        posWk = 0
        for twork in np.arange(0.0, lengthInSeconds, 1.0):
            tMax = min(twork+1.0, lengthInSeconds)
            tm = twork + tInterval
            while tm < tMax:
                idx = self._getIndexAtDurationSecond(tm)
                ent = self.entries[idx]
                messages.append(ent)
                indices.append(idx)
                tm += tInterval
        if onlyMsgList==True:
            return indices
        self.entries = messages
        self.timestamps = [ent.time for ent in self.entries]
    
    @staticmethod
    def getAllConnections(bagFd):
        if (isinstance(bagFd, str)):
            bagFd = rosbag.Bag(bagFd, mode="r")
        rdBags = []
        for c in bagFd._connections.values():
            rdBags.append(RandomAccessBag(bagFd, c.topic))
        return rdBags
    

from cv_bridge import CvBridge

class ImageBag(RandomAccessBag):
    """Access ROS bag that contains sensor_msgs/Image or sensor_msgs/CompressedImage, returns numpy Array directly"""
    
    def __init__ (self, bagFd, topic, start_time=None, end_time=None):
        RandomAccessBag.__init__(self, bagFd, topic, start_time, end_time)
        if (self.connection.datatype!="sensor_msgs/Image" and self.connection.datatype!="sensor_msgs/CompressedImage"):
            raise TypeError("Requested topic is not of Image type")
        if (self.connection.datatype=="sensor_msgs/CompressedImage"):
            self.isCompressed = True
        else:
            self.isCompressed = False
        self.bridge = CvBridge()
        
    def __getitem__(self, i):
        entryMsg = RandomAccessBag.__getitem__(self, i)
        if (self.isCompressed):
            return self.bridge.compressed_imgmsg_to_cv2(entryMsg, "bgr8")
        else:
            return self.bridge.imgmsg_to_cv2(entryMsg, "bgr8")
    
    
if __name__ == "__main__":
    queryBag=ImageBag('/media/sujiwo/PlaceRecognition/ouster64-prep-4.bag', '/front_rgb/image_raw')
    img = queryBag[13713]
    
    pass
    
    