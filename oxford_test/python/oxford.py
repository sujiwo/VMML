#!/usr/bin/python

import numpy as np
import cv2
import csv
import rospy
import rospkg
import sys
if sys.version_info[0]==2:
    from pathlib2 import Path as FsPath
elif sys.version_info[0]==3:
    from pathlib import Path as FsPath
from nav_msgs.msg import Path as Trajectory
from tf import transformations as tfx
from bisect import bisect
from octree import Octree, OctNode
import pcl


def makePointCloudSingle(point):
    cloud = pcl.PointCloud()
    cloud.from_list([[point[0], point[1], point[2]]])
    return cloud


class OxfordDataset:
    
    pkgpath = FsPath(rospkg.RosPack().get_path("oxford_test"))
    distortionModel = None
    
    # These values are constants to shift X/Y coordinates to more `reasonable' values
    OriginCorrectionEasting = -620248.53
    OriginCorrectionNorthing = -5734882.47

    def __init__(self, psrc):
        self.timestamps = []
        self.path = FsPath(psrc)
        if (self.path.exists()==False):
            raise RuntimeError("Oxford dataset not found")
        
        # Open timestamp file
        with open(str(self.path/'stereo.timestamps'), 'r') as fp:
            for cnt, line in enumerate(fp):
                timestamp = line.split(' ')[0]
                self.timestamps.append(timestamp)

        # Load model
        centerLut = self.pkgpath/"model"/"stereo_narrow_left_distortion_lut.bin"
        centerIntrinsic = self.pkgpath/"model"/"stereo_narrow_left.txt";
        lutFd = open(str(centerLut), 'rb')
        lutFd.seek(0, 2);
        lutFdSize = lutFd.tell()
        lutFd.seek(0)
        table = np.zeros((2, lutFdSize/16), dtype=np.float64)
        buf = lutFd.read(lutFdSize/2)
        table[0] = np.frombuffer(buf, dtype=np.float64, count=table.shape[1])
        buf = lutFd.read(lutFdSize/2)
        table[1] = np.frombuffer(buf, dtype=np.float64, count=table.shape[1])
        
        img0 = self.getRaw(0)
        self.width = img0.shape[1]
        self.height = img0.shape[0]
        self.distortion_LUT_center_x = np.float32(table[0]).reshape((self.height, self.width))
        self.distortion_LUT_center_y = np.float32(table[1]).reshape((self.height, self.width))
        
        return
        
    def __len__ (self):
        return len(self.timestamps)
    
    def __getitem__ (self, i):
        img = self.getRaw(i)
        img = cv2.cvtColor(img, cv2.COLOR_BAYER_GB2RGB)
        img = cv2.remap(img, self.distortion_LUT_center_x, self.distortion_LUT_center_y, cv2.INTER_LINEAR)
        return img
    
    @staticmethod
    def timeFromMicrosecond(microsec):
        ts_microsec = int(microsec)
        return rospy.Time(int(ts_microsec/1e6), nsecs=(ts_microsec%1e6)*1e3)
    
    def getTime(self, i):
        return OxfordDataset.timeFromMicrosecond(self.timestamps[i])
    
    def getRaw(self, i):
        img = cv2.imread(str(self.getImagePath(i)), cv2.IMREAD_GRAYSCALE)
        return img
        
    def getImagePath(self, i):
        return self.path / "stereo/centre" / (str(self.timestamps[i])+'.png')
    
#  * These are the columns of INS pose table that are currently used:
#  0 - timestamp
#  1 - easting
#  2 - northing
#  3 - altitude
#  4 - roll
#  5 - pitch
#  6 - yaw
#  7 - velocity_east
#  8 - velocity_north
#  9 - velocity_down
#  10 - latitude
#  11 - longitude
    def getIns(self, correctOrigin=False):
        insFilePath = self.path / "gps" / "ins.csv"
        insTable = np.loadtxt(str(insFilePath), skiprows=1, usecols=[0,6,5,4,12,13,14,10,9,11,2,3], delimiter=',')
        
        # Correct for ROS
        insTable[:,7] = -insTable[:,7]
        insTable[:,8] = -insTable[:,8]
        
        if (correctOrigin==True):
            insTable[:,1] += OxfordDataset.OriginCorrectionEasting
            insTable[:,2] += OxfordDataset.OriginCorrectionNorthing
        
        return insTable
    
    def getImagePathFromINS(self, correctOrigin=False, source='ins'):
        if (source=='ins'):
            ins = self.getIns(correctOrigin)
        elif (source=='ground_truth'):
            ins = self.getGroundTruth(correctOrigin)
        
        def createPose(i):
            px = ins[i]
            return {
                'position': np.array([px[1], px[2], px[3]]), 
                'orientation': tfx.quaternion_from_euler(px[4], px[5], px[6]),
                'velocity': np.linalg.norm(px[7:10])}
            
        def createPoseInterpolate(i, j, ratio):
            pose_i = createPose(i)
            pose_j = createPose(j)
            position = pose_i['position'] + ratio*(pose_j['position']-pose_i['position'])
            orientation = tfx.quaternion_slerp(pose_i['orientation'], pose_j['orientation'], ratio)
            v_i = ins[i, 7:10]
            v_j = ins[j, 7:10]
            v = v_i + ratio*(v_j - v_i)
            return {
                'position': position,
                'orientation': orientation,
                'velocity': np.linalg.norm(v)}
        
        poses = []
        for i in range(len(self.timestamps)):
            ctimestamp = int(self.timestamps[i])
            if (ctimestamp<=ins[0,0]):
                pose = createPose(0)
                poses.append(pose)
            elif (ctimestamp>=ins[-1,0]):
                pose = createPose(-1)
                poses.append(pose)
            else:
                pt = bisect(ins[:,0], ctimestamp)
                ratio = (ctimestamp-ins[pt-1,0]) / (ins[pt,0]-ins[pt-1,0])
                poses.append(createPoseInterpolate(pt-1, pt, ratio))
        return poses
        
    def getImagePathFromInsAsArray(self, correctOrigin=False):
        track__ = self.getImagePathFromINS(correctOrigin)
        track = [[p['position'][0],p['position'][1],p['position'][2],
                  p['orientation'][0],p['orientation'][1],p['orientation'][2],p['orientation'][3]] for p in track__ ]
        return np.array(track)
    
    def getGroundTruth(self, correctOrigin=False):
        rtkPath = (self.path / '..' / 'ground_truth_rtk' / 'rtk' / self.path.name / 'rtk.csv').resolve()
        if (rtkPath.is_file()==False):
            raise IOError("Ground truth not available for "+self.path.name)
        
        gtTable = np.loadtxt(str(rtkPath), delimiter=',', skiprows=1, usecols=[0,5,4,3,11,12,13,9,8,10,1,2])
        
        # Correct for ROS
        gtTable[:,7] = -gtTable[:,7]
        gtTable[:,8] = -gtTable[:,8]
        
        if (correctOrigin==True):
            gtTable[:,1] += OxfordDataset.OriginCorrectionEasting
            gtTable[:,2] += OxfordDataset.OriginCorrectionNorthing

        return gtTable
    
    def getOctree(self, correctOrigin=False):
        
        class FrameNode(object):
            def __init__(self, id, position):
                self.id=id
                self.position = position
        
        framePositions = self.getImagePathFromInsAsArray(correctOrigin)
        
        # Find limits of coordinate
        x_max = np.max(framePositions[:,0])
        x_min = np.min(framePositions[:,0])
        x_dif = x_max-x_min
        y_max = np.max(framePositions[:,1])
        y_min = np.min(framePositions[:,1])
        y_dif = y_max-y_min
        z_max = np.max(framePositions[:,2])
        z_min = np.min(framePositions[:,2])
        z_dif = z_max-z_min
        
        octree = Octree(np.max([x_dif, y_dif, z_dif]), 
            origin=(x_min+x_dif/2, y_min+y_dif/2, z_min+z_dif/2))
        for i in range(len(self)):
            position = (framePositions[i,0], framePositions[i,1], framePositions[i,2])
            frame = FrameNode(i, position)
            octree.insertNode(position, frame)
         
        return octree 
    
    def getOctree2(self, correctOrigin=False):
        framePositions = self.getImagePathFromInsAsArray(correctOrigin)
        cloud = pcl.PointCloud()
        cloud.from_array(np.float32(framePositions[:,0:3]))
        kdt = cloud.make_kdtree_flann()
        return kdt



if __name__ == "__main__":
    dataset = OxfordDataset("/media/sujiwo/VisionMapTest/Oxford-RobotCar/2014-11-18-13-20-12")
    oct = dataset.getImagePathFromINS(True, source='ground_truth')
    pass
    
    
    