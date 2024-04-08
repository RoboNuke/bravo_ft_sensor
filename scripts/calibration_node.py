import rospy

import numpy as np



class FTCalibration:
    def __init__(self):
        pass

    def getRawData(self, n): # number of data points
        jntAngles = [] # list of joint angles
        gts = [] # list of transformed gravity (pre rotation)
        fs = [] # list of force readings
        ts = [] # list of torque readings
        # while not enough points
            # get random robot position
            # move to position
            # wait to settle
            # get gravity transform
            # get force-torque reading
        return gts, fs, ts, jntAngles
    
    def getRotLS(self, gt):
        # transformed gravity vector
        pass

    def estGRot(self, gts, fs):
        # perform least squares to estimate ft_frame rotation
        return 0.0, 1.0 # rotation and ls r

    def getForceLS(self, gs):
        # transformed+rotated gravity
        pass
    
    def estM(self, gs, fs):
        return 0.0, np.array([0.0, 0.0, 0.0]), 1.0 # mass and biases and ls r

    def getTorqueLS(self, g, m):
        # transf/rot gravity and mass
        pass

    def estCOG(self, gs, m, ts):
        return np.array([0,0,0.0]), np.array([0,0,0.0]), 1.0 # cog and torque bias and ls r
    
    def fullCalibration(self):
        gts, fs, ts, jntAngles = self.getRawData(self.requiredPts)
        # estimate gravity rotation
        theta, rt = self.estGRot(gts, fs)
        # estimate mass and force bias
        gs = [] # rotated gts 
        m, fBias, rm = self.estM(gs, fs)
        # estimate cog and torque bias
        cog, tBias, rcog = self.estCOG(gs, m, ts)
        # store calibration
        # store raw data
        return rt, rm, rcog # quality of the 3 different regressions


