# ----- Imports -------------------------------------------------------
# Standard imports
import random
import numpy as np
import time

# PyQT imports
from PyQt5.QtCore import QDateTime, Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
        QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
        QSlider, QSpinBox, QStyleFactory, QTableWidget, QTabWidget, QTextEdit,
        QVBoxLayout, QWidget, QFileDialog)
from PyQt5.QtGui import QPainter, QColor, QFont
import pyqtgraph as pg
import pyqtgraph.opengl as gl

# Local Imports
from gui_parser import uartParser
from gui_common import *
from graphUtilities import *


class parseUartThread(QThread):
        fin = pyqtSignal('PyQt_PyObject')

        def __init__(self, uParser):
                QThread.__init__(self)
                self.parser = uParser

        def run(self):
                if(self.parser.parserType == "SingleCOMPort"):
                    outputDict = self.parser.readAndParseUartSingleCOMPort()
                else:
                    outputDict = self.parser.readAndParseUartDoubleCOMPort()
                self.fin.emit(outputDict)

class sendCommandThread(QThread):
        done = pyqtSignal()
        def __init__(self, uParser, command):
                QThread.__init__(self)
                self.parser = uParser
                self.command = command

        def run(self):
            self.parser.sendLine(self.command)
            self.done.emit()

class updateQTTargetThread3D(QThread):
    done = pyqtSignal()

    def __init__(self, pointCloud, targets, scatter, pcplot, numTargets, ellipsoids, coords, classifierOut=[], zRange=[-3, 3], colorGradient=[], pointColorMode="", drawTracks=True, trackColorMap=None, pointBounds=False):
        QThread.__init__(self)
        self.pointCloud = pointCloud
        self.targets = targets
        self.scatter = scatter
        self.pcplot = pcplot
        self.colorArray = ('r','g','b','w')
        self.numTargets = numTargets
        self.ellipsoids = ellipsoids
        self.coordStr = coords
        self.classifierOut = classifierOut
        self.zRange = zRange
        self.colorGradient = colorGradient
        self.pointColorMode = pointColorMode
        self.drawTracks = drawTracks
        self.trackColorMap = trackColorMap
        self.pointBounds = pointBounds
        # This ignores divide by 0 errors when calculating the log2
        np.seterr(divide = 'ignore')

    def drawTrack(self, track, trackColor):
        # Get necessary track data
        tid = int(track[0])
        x = track[1]
        y = track[2]
        z = track[3]

        track = self.ellipsoids[tid]
        mesh = getBoxLinesCoords(x,y,z)
        track.setData(pos=mesh,color=trackColor,width=2,antialias=True,mode='lines')
        track.setVisible(True)

    # Return transparent color if pointBounds is enabled and point is outside pointBounds
    # Otherwise, color the point depending on which color mode we are in    
    def getPointColors(self, i):
        if (self.pointBounds['enabled']) :
            xyz_coords = self.pointCloud[i,0:3]
            if (   xyz_coords[0] < self.pointBounds['minX']
                or xyz_coords[0] > self.pointBounds['maxX']
                or xyz_coords[1] < self.pointBounds['minY']
                or xyz_coords[1] > self.pointBounds['maxY']
                or xyz_coords[2] < self.pointBounds['minZ']
                or xyz_coords[2] > self.pointBounds['maxZ']
                ) :
                return pg.glColor((0,0,0,0))

        # Color the points by their SNR
        if (self.pointColorMode == COLOR_MODE_SNR):
            snr = self.pointCloud[i,4]
            # SNR value is out of expected bounds, make it white
            if (snr < SNR_EXPECTED_MIN) or (snr > SNR_EXPECTED_MAX):
                return pg.glColor('w')
            else:
                return pg.glColor(self.colorGradient.getColor((snr-SNR_EXPECTED_MIN)/SNR_EXPECTED_RANGE))

        # Color the points by their Height
        elif (self.pointColorMode == COLOR_MODE_HEIGHT):
            zs = self.pointCloud[i, 2]

            # Points outside expected z range, make it white
            if (zs < self.zRange[0]) or (zs > self.zRange[1]):
                return pg.glColor('w')
            else:
                colorRange = self.zRange[1]+abs(self.zRange[0]) 
                zs = self.zRange[1] - zs 
                return pg.glColor(self.colorGradient.getColor(abs(zs/colorRange)))

        # Color Points by their doppler
        elif(self.pointColorMode == COLOR_MODE_DOPPLER):
            doppler = self.pointCloud[i,3]
            # Doppler value is out of expected bounds, make it white
            if (doppler < DOPPLER_EXPECTED_MIN) or (doppler > DOPPLER_EXPECTED_MAX):
                return pg.glColor('w')
            else:
                return pg.glColor(self.colorGradient.getColor((doppler-DOPPLER_EXPECTED_MIN)/DOPPLER_EXPECTED_RANGE))
     
        # Color the points by their associate track
        elif (self.pointColorMode == COLOR_MODE_TRACK):
            trackIndex = int(self.pointCloud[i, 6])
            # trackIndex of 253, 254, or 255 indicates a point isn't associated to a track, so check for those magic numbers here
            if (trackIndex == TRACK_INDEX_WEAK_SNR or trackIndex == TRACK_INDEX_BOUNDS or trackIndex == TRACK_INDEX_NOISE):
                return pg.glColor('w')
            else:
                # Catch any errors that may occur if track or point index go out of bounds
                try:
                    return self.trackColorMap[trackIndex]
                except Exception as e:
                    print (e)
                    return pg.glColor('w')

        # Unknown Color Option, make all points green
        else:
            return pg.glColor('g')

    def run(self):
        # Clear all previous targets
        for e in self.ellipsoids:
            if (e.visible()):
                e.hide()
        
        # Create a list of just X, Y, Z values to be plotted
        toPlot = self.pointCloud[:, 0:3]

        # Determine the size of each point based on its SNR
        with np.errstate(divide='ignore'):
            size = np.log2(self.pointCloud[:, 4])
        
        # Each color is an array of 4 values, so we need an numPoints*4 size 2d array to hold these values
        pointColors = np.zeros((self.pointCloud.shape[0], 4))

        # Set the color of each point
        for i in range(self.pointCloud.shape[0]):
            pointColors[i] = self.getPointColors(i)

        # Plot the points
        self.scatter.setData(pos=toPlot, color=pointColors, size=size)

        # Graph the targets
        try:
            if (self.drawTracks):
                if (self.targets is not None):
                    for track in self.targets:
                        trackID = int(track[0])
                        trackColor = self.trackColorMap[trackID]
                        self.drawTrack(track,trackColor)
        except:
             print("Unable to draw all tracks, ignoring and continuing execution...")

        self.done.emit()
