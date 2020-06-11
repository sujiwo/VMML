#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 10 11:28:34 2020

@author: sujiwo
"""

import numpy as np
from scipy.stats import randint
from pylab import plot, scatter
import oxford


def buildSamples(dataset, ratio=0.01):
    sampleNum = int(np.ceil(len(dataset)*ratio))
    samples = randint.rvs(0, len(dataset), size=sampleNum)

    # Try plot
    ins = dataset.getImagePathFromInsAsArray(True)
    
    plot(ins[:,0], ins[:,1])
    samplePts = np.array([ins[r,0:3] for r in samples ])
    scatter(samplePts[:,0], samplePts[:,1], s=9.0, c='Red')

    return samples

def searchAround(kdtSource, point, radius=5.0):
    cloudSingle = oxford.makePointCloudSingle(point)
    return kdtSource.radius_search_for_cloud(cloudSingle, radius, max_nn=10000)
    pass

def getGroundTruthForSample(datasetTrainTrajectory, datasetTrainKdtree, datasetTestTrajectory, dsTestItemId, maxDistance=5.0):
    testPose = datasetTestTrajectory[dsTestItemId]
    
    cloudSingle = oxford.makePointCloudSingle(testPose['position'])
    # Force large numbers
    targetPointsChk = [p for p in 
        datasetTrainKdtree.radius_search_for_cloud(cloudSingle, maxDistance, max_nn=10000)[0][0] 
        if p!=0]
    
    validTargetPoints = []
    for ptId in targetPointsChk:
        pose = datasetTrainTrajectory[ptId]
        rpyTest = oxford.tfx.euler_from_quaternion(testPose['orientation'])
        rpyTrain = oxford.tfx.euler_from_quaternion(pose['orientation'])
        
        # 5 degrees
        if (abs(rpyTest[2]-rpyTrain[2]) < 0.0872665):
            validTargetPoints.append(ptId)
    
    return validTargetPoints

def getGroundTruthForSamples(datasetTrain, datasetTest):
    pass


if (__name__=='__main__'):
    train = oxford.OxfordDataset('/media/sujiwo/VisionMapTest/Oxford-RobotCar/2014-11-18-13-20-12')
    test  = oxford.OxfordDataset('/media/sujiwo/VisionMapTest/Oxford-RobotCar/2015-04-24-08-15-07')
    trainTrack = train.getImagePathFromINS(True)
    kdtree = train.getOctree2(True)
    testTrack  = test.getImagePathFromINS(True)
    x = getGroundTruthForSample(trainTrack, kdtree, testTrack, 880)
    
    pass