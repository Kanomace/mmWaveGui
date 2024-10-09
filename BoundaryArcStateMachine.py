from os.path import exists
from os import mkdir
import math
import numpy as np

NO_PRESENCE_MOTION_STATE = 0
MINOR_PRESENCE_MOTION_STATE = 1
MAJOR_PRESENCE_MOTION_STATE = 2

# The majorBoundaryArcStateMachineType can technically support both the major motion 
# and the major motion configs. The arguments are exactly the same with the exception
# of major2majorThre and major2emptyThre. In this implementation, since major mode and
# major mode are not supported simultaneously, the major2majorThre transitions the
# state machine from major mode to empty mode.

class majorBoundaryArcStateMachineType:
        
    def __init__(self):
        try:
            self.currentState = NO_PRESENCE_MOTION_STATE
            self.nextState = NO_PRESENCE_MOTION_STATE
            self.majorToEmptyCounter = 0
            self.majorPointThre1 = 0
            self.majorPointThre2 = 0
            self.majorSNRThre2 = 0
            self.majorPointHistThre1 = 0
            self.majorPointHistThre2 = 0
            self.majorSNRHistThre2 = 0
            self.majorHistBufferSize = 0
            self.majorToEmptyThre = 0
            self.rMin = 0
            self.rMin = 0
            self.thetaMin = 0
            self.thetaMax = 0
            self.zMin = 0
            self.zMax = 0
            self.history = []
            self.BoundaryArcIndex = 0
        except:
            print("Boundary Arc initialization failed")
    def getState(self):
        return self.currentState

    def configure(self, majorPointThre1, majorPointThre2, majorSNRThre2, majorPointHistThre1, majorPointHistThre2, \
        majorSNRHistThre2, majorHistBufferSize, majorToEmptyThre, rMin, rMax, thetaMin, thetaMax, zMin, zMax, BoundaryArcIndex):
        try:
            self.majorPointThre1 = majorPointThre1
            self.majorPointThre2 = majorPointThre2
            self.majorSNRThre2 = majorSNRThre2
            self.majorPointHistThre1 = majorPointHistThre1
            self.majorPointHistThre2 = majorPointHistThre2
            self.majorSNRHistThre2 = majorSNRHistThre2
            self.majorHistBufferSize = majorHistBufferSize
            self.majorToEmptyThre = majorToEmptyThre
            self.rMin = rMin
            self.rMax = rMax
            self.thetaMin = thetaMin
            self.thetaMax = thetaMax
            self.zMin = zMin
            self.zMax = zMax
            self.BoundaryArcIndex = BoundaryArcIndex
        except:
            print("Minor Boundary Arc initialization failed")
    
    def step(self, clusters):
        totalSNR = 0
        totalnumPoints = 0
        for cluster in clusters:
            if (cluster['r'] > self.rMin and cluster['r'] < self.rMax and cluster['theta']> self.thetaMin \
                and cluster['theta'] < self.thetaMax and cluster['z'] > self.zMin and cluster['z'] < self.zMax):
                totalSNR = totalSNR + cluster['snr']
                totalnumPoints = totalnumPoints + cluster['numPoints']
        
        self.history.append([totalnumPoints, totalSNR])

        if(len(self.history) > self.majorHistBufferSize):
            self.history.pop(0)
    
        self.compute_state()
    
    def compute_state(self):
        if (self.currentState == NO_PRESENCE_MOTION_STATE):
            if(self.history[-1][0] >= self.majorPointThre1):
                self.nextState = MINOR_PRESENCE_MOTION_STATE
                self.majorToEmptyCounter = 0
            elif(self.history[-1][0] >= self.majorPointThre2 and self.history[-1][1] >= self.majorSNRThre2):
                self.nextState = MINOR_PRESENCE_MOTION_STATE
                self.majorToEmptyCounter = 0
            elif(sum(self.history[:][0]) >= self.majorPointHistThre1):
                self.nextState = MINOR_PRESENCE_MOTION_STATE
                self.majorToEmptyCounter = 0
            elif(sum(self.history[:][0]) >= self.majorPointHistThre2 and sum(self.history[:][1]) >= self.majorSNRHistThre2):
                self.nextState = MINOR_PRESENCE_MOTION_STATE
                self.majorToEmptyCounter = 0
        elif(self.currentState == MINOR_PRESENCE_MOTION_STATE):
            if(self.history[-1][0] >= self.majorPointThre1):
                self.nextState = MINOR_PRESENCE_MOTION_STATE
                self.majorToEmptyCounter = 0
            elif(self.history[-1][0] >= self.majorPointThre2 and self.history[-1][1] >= self.majorSNRThre2):
                self.nextState = MINOR_PRESENCE_MOTION_STATE
                self.majorToEmptyCounter = 0
            elif(sum(self.history[:][0]) >= self.majorPointHistThre1):
                self.nextState = MINOR_PRESENCE_MOTION_STATE
                self.majorToEmptyCounter = 0
            elif(sum(self.history[:][0]) >= self.majorPointHistThre2 and sum(self.history[:][1]) >= self.majorSNRHistThre2):
                self.nextState = MINOR_PRESENCE_MOTION_STATE
                self.majorToEmptyCounter = 0
            elif(self.majorToEmptyCounter == self.majorToEmptyThre):
                self.nextState = NO_PRESENCE_MOTION_STATE
            else:
                self.majorToEmptyCounter = self.majorToEmptyCounter + 1
        
        self.currentState = self.nextState

    def getBoundaryArcIndex(self):
        return self.BoundaryArcIndex