#!/usr/bin/python

import numpy as np
import cv2
import csv
import rospy
import rospkg
from pathlib2 import Path as FsPath
from nav_msgs.msg import Path as Trajectory
from tf import transformations as tfx
from bisect import bisect


class OxfordDataset:
    
    path = None
    pkgpath = FsPath(rospkg.RosPack().get_path("oxford_test"))
    timestamps = []
    distortionModel = None
    
    # These values are constants to shift X/Y coordinates to more `reasonable' values
    OriginCorrectionEasting = -620248.53
    OriginCorrectionNorthing = -5734882.47

    def __init__(self, psrc):
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
                'orientation': tfx.quaternion_from_euler(px[6], px[7], px[8]),
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
            if (ctimestamp<ins[0,0]):
                pose = createPose(0)
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



if __name__ == "__main__":
    dataset = OxfordDataset("/media/sujiwo/VisionMapTest/Oxford-RobotCar/2015-03-17-11-08-44")
    gtrtk = dataset.getGroundTruth()
    pass
    
    
    