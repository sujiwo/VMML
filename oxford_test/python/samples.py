#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 10 11:28:34 2020

@author: sujiwo
"""

import numpy as np
from scipy.stats import randint
from pylab import plot, scatter


def buildSamples(dataset, ratio=0.01):
    sampleNum = int(np.ceil(len(dataset)*ratio))
    samples = randint.rvs(0, len(dataset), size=sampleNum)

    # Try plot
    ins = dataset.getImagePathFromInsAsArray(True)
    
    plot(ins[:,0], ins[:,1])
    samplePts = np.array([ins[r,0:3] for r in samples ])
    scatter(samplePts[:,0], samplePts[:,1], s=9.0, c='Red')

    return samples

def getGroundTruthForSamples(datasetTrain, datasetTest):
    pass