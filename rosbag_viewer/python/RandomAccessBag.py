import rospy
import rosbag
import numpy as np
from bisect import bisect


class RandomAccessBag:
    """RandomAccessBag is a wrapper for ROS Bag that allows random access to any message within a single topic"""
    
    def __init__ (self, bagFd, topic, start_time=None, end_time=None):
        assert(type(bagFd)==rosbag.bag.Bag)
        self.bagFd = bagFd
        for cn in self.bagFd._connections.values():
            if cn.topic==topic:
                self.connection = cn
                break
            
        startTimeR = bagFd._chunks[0].start_time
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
    
    def hz(self):
        dt = (self.entries[-1].time - self.entries[0].time).to_sec()
        return float(len(self.entries)) / dt
    
    def _getEntryAtDurationSecond(self, fSec):
        tm = self.entries[0].time + rospy.Duration.from_sec(fSec)
        tp = bisect(self.timestamps, tm)
        return self.entries[tp]
    
    def desample(self, hz):
        """Reduce frequency of the messages by removing redundant entries. May be inaccurate"""
        lengthInSeconds = (self.entries[-1].time - self.entries[0].time).to_sec()
        messages = []
        tInterval = 1.0 / float(hz)
        posWk = 0
        for twork in np.arange(0.0, lengthInSeconds, 1.0):
            tMax = min(twork+1.0, lengthInSeconds)
            tm = twork + tInterval
            while tm < tMax:
                ent = self._getEntryAtDurationSecond(tm)
                messages.append(ent)
                tm += tInterval
        self.entries = messages
        self.timestamps = [ent.time for ent in self.entries]
    
    @staticmethod
    def getAllConnections(bagFd):
        rdBags = []
        for c in bagFd._connections.values():
            rdBags.append(RandomAccessBag(bagFd, c.topic))
        return rdBags
    
    
if __name__ == "__main__":
    bagFd = rosbag.Bag("/media/sujiwo/VisionMapTest/ready/ouster64-conv.bag")
    rdBag = RandomAccessBag(bagFd, "/front_rgb/image_raw")
    rdBag.desample(10.0)
    
    pass
#     for topic, msg, timestamp in bagFd.read_messages("/front_rgb/image_raw"):
#         print(topic)
#         print(timestamp)
#         h = msg.header
#         pass
