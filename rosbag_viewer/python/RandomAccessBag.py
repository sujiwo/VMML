import rospy
import rosbag
import numpy as np
from bisect import bisect
from numbers import Number


class RandomAccessBag(object):
    """
    RandomAccessBag is a wrapper for ROS Bag that allows random access to any message within a single topic
    
    Attributes
    ----------
    - entries: list of bagEntry as index to each message's position in the file 
    - timestamps: Timestamp at each message's recording time
    """
    hasDesampled = False
    
    def __init__ (self, bagFd, topic):
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
            
#         self.entries = []
#         for entry in self.bagFd._get_entries([self.connection]):
#             self.entries.append(entry)
        self.entries = [e for e in self.bagFd._get_entries([self.connection])]
        self.entries.sort(key=lambda en: en.time)
        self.timestamps = [ent.time for ent in self.entries]
        
    def topic(self):
        return self.connection.topic
    
    def type(self):
        return self.connection.datatype
    
    def __repr__(self):
        return "ROS Bag of type=`{}', topic=`{}'".format(self.type(), self.topic())
        
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
        # Special entry 0
        if tm==self.entries[0].time:
            return 0
        return bisect(self.timestamps, tm)
    
    """Reduce frequency of the messages by removing redundant entries. May be inaccurate
    @param hz: Frequency of sampling. Must be less than original message frequence; otherwise undefined.
    @param onlyMsgList: Whether to return only list of position or change the message list
    @param startTime: offset of time (in seconds) from start of bag, default to 0
    @param stopTime: offset of time (seconds) from start of bag to tag end of sampling, default to -1 (end of bag)
    """
    def desample(self, hz=-1, onlyMsgList=False, startTime=0.0, stopTime=-1):
        if (self.hasDesampled==True):
            raise RuntimeError("Random bag cannot be desampled more than once")
        
        if isinstance(startTime, Number) and isinstance(stopTime, Number):
            if (startTime==0.0):
                startTime = self.entries[0].time
                td = 0.0
            else:
                td = float(startTime)
                startTime = self.entries[0].time + rospy.Duration.from_sec(startTime)
            if (stopTime==-1):
                stopTime = self.entries[-1].time
            else:
                stopTime = self.entries[0].time + rospy.Duration.from_sec(stopTime)
            lengthInSeconds = (stopTime - startTime).to_sec()
            if (lengthInSeconds<0):
                raise ValueError("Error: stopTime is less than startTime")
            
        elif hasattr(startTime, "to_sec") and hasattr(stopTime, "to_sec"):
            if startTime < self.entries[0].time or stopTime > self.entries[-1].time:
                raise ValueError("Requested time is out of range")
            else:
                startTime = (startTime-self.entries[0].time).to_sec()
                stopTime = (stopTime-self.entries[0].time).to_sec()
                return self.desample(hz, onlyMsgList, startTime, stopTime)
            pass
        else:
            raise ValueError("Unknown values for start time or stop time")
        
        messages = []
        indices = []
        if hz==-1:
            startIdx = self._getIndexAtDurationSecond(td)
            stopIdx = self._getIndexAtDurationSecond(td+lengthInSeconds)
            indices = range(startIdx, stopIdx+1)
            messages = [self.entries[i] for i in indices]
        else:
            tInterval = 1.0 / float(hz)
            for twork in np.arange(td, td+lengthInSeconds, 1.0):
                tMax = min(twork+1.0, td+lengthInSeconds)
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
        hasDesampled = True
    
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
    
    def __init__ (self, bagFd, topic):
        super(ImageBag, self).__init__(bagFd, topic)
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
        
    def __repr__(self):
        return "ROS Image Bag, topic=`{}'".format(self.topic())

    
    
if __name__ == "__main__":
    queryBag=ImageBag('/Data/MapServer/Logs/vls128-conv.bag', '/front_rgb/image_raw')
    samples = queryBag.desample(5.0, True, 138.03, 267.22)
    
    pass
    
    