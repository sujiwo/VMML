#!/usr/bin/python

# How To Use this script
# 1. Run the driver node from command prompt
# 2. Modify variables source_topic and output_topic:
#    - source_topic is set to the topic(s) that are available in bag and required by driver
#    - output_topic is set to topic published by driver
# 3. Run this script with parameters: <input bag> <output bag> (must be different)
# 4. Reindex the resulting bag

import rosbag
import rospy
import sys
import threading
from sensor_msgs.point_cloud2 import PointCloud2

bagWrite = None
bagWriteLock = threading.Semaphore()

# Modify these two vars
# source_topic = ["/ouster_driver/lidar_packets", "/ouster_driver/imu_packets"]
# output_topic = "/points_raw"
source_topic = ["/velodyne_packets"]
output_topic = "/velodyne_points"

source_messageType = []
realMessageTimestamp = None

def driverOutputCallback(message):
    bagWriteLock.acquire()
    bagWrite.write(output_topic, message, realMessageTimestamp)
    bagWriteLock.release()


if (__name__=="__main__"):
    
    rospy.init_node("lidar_conversion")
    mBag = rosbag.Bag(sys.argv[1], mode='r')
    bagWrite = rosbag.Bag(sys.argv[2], mode='w')
    
    # Probe message types
    j = 0
    for rowz in mBag.read_messages(topics=source_topic):
        if (rowz.topic==source_topic[j]):
            source_messageType.append(rowz.message.__class__)
            j+=1
        if (len(source_topic)==len(source_messageType)):
            break
    
#     Build publishers specific for each topic 
    senders = [rospy.Publisher(source_topic[i], source_messageType[i], queue_size=1) for i in range(len(source_topic))]
    receiver = rospy.Subscriber(output_topic, PointCloud2, driverOutputCallback, queue_size=1)
    
    i=0
    for rows in mBag.read_messages():
        
        realMessageTimestamp = rows.timestamp
        try:
            topicNum = source_topic.index(rows.topic)
            senders[topicNum].publish(rows.message)
        except ValueError:
            bagWriteLock.acquire()
            bagWrite.write(rows.topic, rows.message, rows.timestamp)
            bagWriteLock.release()
        print(i)
        i+=1
    
    bagWrite.close()    
    mBag.close()
    
    