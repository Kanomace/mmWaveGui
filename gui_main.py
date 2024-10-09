# ----- Imports -------------------------------------------------------

# Standard Imports
import sys
import numpy as np
import time
import math
import struct
import os
import string
import serial
import serial.tools.list_ports
import statistics
import warnings
import random
import copy
import collections

# PyQt5 Imports
from PyQt5.QtCore import QDateTime, Qt, QTimer, QThread, pyqtSignal, QSize
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
        QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
        QSlider, QSpinBox, QStyleFactory, QTableWidget, QTableWidgetItem, QTabWidget, QTextEdit,
        QVBoxLayout, QWidget, QFileDialog, QButtonGroup, QFormLayout, QFrame, QSpacerItem)
from PyQt5.QtGui import QPixmap
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.pgcollections import OrderedDict
from collections import deque
from gl_classes import GLTextItem
from sklearn.cluster import DBSCAN

# Local File Imports
from gui_threads import *
from gui_parser import uartParser
from graphUtilities import *
from gui_common import *
from cachedData import *
from fall_detection import FallDetection, fallDetectionSliderClass
from BoundaryArcStateMachine import *

# ----- Defines -------------------------------------------------------
compileGui = 0

# MajorMotionStateMachines can be used to detect presence off-chip
majorMotionStateMachines = []
# Change this value to 0 to take presence/motion detection outputs 
# directly from the radar device instead of computing them in the python code 
OFF_CHIP_PRESENCE_DETECTION_ENABLED = 1

# CachedData holds the data from the last configuration run for faster prototyping and testing
cachedData = cachedDataType()
# Only when compiling
if (compileGui):
    from fbs_runtime.application_context.PyQt5 import ApplicationContext


# Create a list of N distict colors, visible on the black GUI background, for our tracks
# The format for a single color is (r,g,b,a) -> normalized from 0-255 to 0-1
# LUT based on Kelly's 22 Colors of Max Contrast, slightly adjusted for better visibility on black background (https://sashamaps.net/docs/resources/20-colors/)
# Only the first 21 colors are guaranteed to be highly distinct. After that colors are generated, but not promised to be visually distinct.
def get_trackColors(n):
    # Modified LUT of Kelly's 22 Colors of Max Contrast
    modKellyColors = [
        # (255, 255, 255, 255),   # White
        # (  0,   0,   0, 255),   # Black
        # (169, 169, 169, 255),   # Gray
        (230,  25,  75, 255),   # Red
        ( 60, 180,  75, 255),   # Green
        (255, 225,  25, 255),   # Yellow
        ( 67,  99, 216, 255),   # Blue
        (245, 130,  49, 255),   # Orange
        (145,  30, 180, 255),   # Purple
        ( 66, 212, 244, 255),   # Cyan
        (240,  50, 230, 255),   # Magenta
        (191, 239,  69, 255),   # Lime
        (250, 190, 212, 255),   # Pink
        ( 70, 153, 144, 255),   # Teal
        (220, 190, 255, 255),   # Lavender
        (154,  99,  36, 255),   # Brown
        (255, 250, 200, 255),   # Beige
        (128,   0,   0, 255),   # Maroon
        (170, 255, 195, 255),   # Mint
        (128, 128,   0, 255),   # Olive
        (255, 216, 177, 255),   # Apricot
        (  0,   0, 117, 255)    # Navy
    ]
    
    # Generate normalized version of Kelly colors
    modKellyColorsNorm = []
    for tup in modKellyColors: 
        modKellyColorsNorm.append(tuple(ti/255 for ti in tup))
    
    # Create the output color list
    trackColorList = []
    for i in range(n):
        # If within the length of the LUT, just grab values
        if i < len(modKellyColorsNorm):
            trackColorList.append(modKellyColorsNorm[i])
        # Otherwise, generate a color from the average of two randomly selected colors, and add the new color to the list
        else:  
            (r_2, g_2, b_2, _) = modKellyColorsNorm[random.randint(0,len(modKellyColorsNorm)-1)]
            (r_1, g_1, b_1, _) = modKellyColorsNorm[random.randint(0,len(modKellyColorsNorm)-1)]
            r_gen = (r_2 + r_1) / 2
            g_gen = (g_2 + g_1) / 2
            b_gen = (b_2 + b_1) / 2
            modKellyColorsNorm.append((r_gen, g_gen, b_gen , 1.0))
            trackColorList.append(    (r_gen, g_gen, b_gen , 1.0))

    return trackColorList

def next_power_of_2(x):  
    return 1 if x == 0 else 2**(x - 1).bit_length()

class Window(QDialog):
    def __init__(self, parent=None, size=[]):
        super(Window, self).__init__(parent)
        # set window toolbar options, and title
        self.setWindowFlags(
            Qt.Window |
            Qt.CustomizeWindowHint |
            Qt.WindowTitleHint |
            Qt.WindowMinimizeButtonHint |
            Qt.WindowMaximizeButtonHint |
            Qt.WindowCloseButtonHint
        )
        self.setWindowTitle("mmWave Industrial Visualizer")

        if (0): #set to 1 to save terminal output to logFile, set 0 to show terminal output
            ts = time.localtime()
            terminalFileName = str('logData/logfile_'+ str(ts[2]) + str(ts[1]) + str(ts[0]) + '_' + str(ts[3]) + str(ts[4]) +'.txt')
            sys.stdout = open(terminalFileName, 'w')

        print('Python is ', struct.calcsize("P")*8, ' bit')
        print('Python version: ', sys.version_info)

        # TODO bypass serial read function to also log to a file

        self.frameTime = 50
        self.graphFin = 1
        self.hGraphFin = 1
        self.threeD = 1
        self.lastFramePoints = np.zeros((5,1))
        self.plotTargets = 1
        self.frameNum = 0
        self.profile = {'startFreq': 60.25, 'numLoops': 64, 'numTx': 3, 'sensorHeight':3, 'maxRange':10, 'az_tilt':0, 'elev_tilt':0, 'enabled':0}
        self.chirpComnCfg = {'DigOutputSampRate':23, 'DigOutputBitsSel':0, 'DfeFirSel':0, 'NumOfAdcSamples':128, 'ChirpTxMimoPatSel':4, 'ChirpRampEndTime':36.1, 'ChirpRxHpfSel':1}
        self.chirpTimingCfg = {'ChirpIdleTime':8, 'ChirpAdcSkipSamples':24, 'ChirpTxStartTime':0, 'ChirpRfFreqSlope':47.95, 'ChirpRfFreqStart':60}
        self.guiMonitor = {'pointCloud':1, 'rangeProfile':0, 'NoiseProfile':0, 'rangeAzimuthHeatMap':0, 'rangeDopplerHeatMap':0, 'statsInfo':0}
        self.channelCfg = {'RX':3, 'TX':2}
        self.sigProcChain = {'majorMotionEnabled':1}
        self.clutterRemoval = 0
        self.measureRangeBiasAndRxChanPhase = {'enabled':0, 'centerDist':0,'searchRange':0}
        self.rangeRes = 0
        self.rangeAxisVals = np.zeros(int(self.chirpComnCfg['NumOfAdcSamples']/2))
        self.sensorHeight = 1.5
        self.numFrameAvg = 10
        self.configSent = 0
        self.previousFirstZ = -1
        self.yzFlip = 0
        self.mpdZoneType = None
        self.vitalsIWRL6432 = 0

        self.trackColorMap = None
        self.prevConfig = DEMO_NAME_OOB
        self.vitalsPatientData = []
        self.pointBounds = {'enabled':False, 'minX':0, 'maxX':0, 'minY':0, 'maxY':0, 'minZ':0, 'maxZ':0}
     
        # Flag to indicate if the last frame was parsed incorrectly to ignore the error on the subsequent frame when 
        # the number of points won't be consistent with the tracker data
        self.lastFrameErrorFlag = False
        #color gradients
        # TODO Simplify color gradients
        self.Gradients = OrderedDict([
    ('bw', {'ticks': [(0.0, (0, 0, 0, 255)), (1, (255, 255, 255, 255))], 'mode': 'rgb'}),
    ('hot', {'ticks': [(0.3333, (185, 0, 0, 255)), (0.6666, (255, 220, 0, 255)), (1, (255, 255, 255, 255)), (0, (0, 0, 0, 255))], 'mode': 'rgb'}),
    ('jet', {'ticks': [(1, (166, 0, 0, 255)), (0.32247191011235954, (0, 255, 255, 255)), (0.11348314606741573, (0, 68, 255, 255)), (0.6797752808988764, (255, 255, 0, 255)), (0.902247191011236, (255, 0, 0, 255)), (0.0, (0, 0, 166, 255)), (0.5022471910112359, (0, 255, 0, 255))], 'mode': 'rgb'}),
    ('summer', {'ticks': [(1, (255, 255, 0, 255)), (0.0, (0, 170, 127, 255))], 'mode': 'rgb'} ),
    ('space', {'ticks': [(0.562, (75, 215, 227, 255)), (0.087, (255, 170, 0, 254)), (0.332, (0, 255, 0, 255)), (0.77, (85, 0, 255, 255)), (0.0, (255, 0, 0, 255)), (1.0, (255, 0, 127, 255))], 'mode': 'rgb'}),
    ('winter', {'ticks': [(1, (0, 255, 127, 255)), (0.0, (0, 0, 255, 255))], 'mode': 'rgb'}),
    ('spectrum2', {'ticks': [(1.0, (255, 0, 0, 255)), (0.0, (255, 0, 255, 255))], 'mode': 'hsv'}),
    ('heatmap', {'ticks': [ (1, (255, 0, 0, 255)), (0, (131, 238, 255, 255))], 'mode': 'hsv'})
])
        cmap = 'heatmap'
        if (cmap in self.Gradients):
            self.gradientMode = self.Gradients[cmap]
        self.zRange = [-3, 3]
        self.plotHeights = 1
        # Gui size
        if (size):
            left = 50
            top = 50
            width = math.ceil(size.width()*0.9)
            height = math.ceil(size.height()*0.9)
            self.setGeometry(left, top, width, height)
        # Persistent point cloud
        self.previousClouds = []

        self.hearPlotData = []
        self.breathPlotData = []

        # Set up graph pyqtgraph
        self.init3dGraph()
        self.initColorGradient()
        self.init1dGraph()

        # Add connect options
        self.initConnectionPane()
        self.initStatsPane()
        self.initPlotControlPane()
        self.initFallDetectPane()
        self.initConfigPane()
        self.initSensorPositionPane()
        self.initBoundaryBoxPane()

        # Set the layout
        # Create tab for different graphing options
        self.graphTabs = QTabWidget()
        self.graphTabs.addTab(self.pcplot, '3D Plot')
        self.graphTabs.addTab(self.rangePlot, 'Range Plot')
        self.graphTabs.currentChanged.connect(self.whoVisible)

        self.gridlay = QGridLayout()
        self.gridlay.addWidget(self.comBox, 0,0,1,1)
        self.gridlay.addWidget(self.statBox, 1,0,1,1)
        self.gridlay.addWidget(self.configBox,2,0,1,1)
        self.gridlay.addWidget(self.plotControlBox,3,0,1,1)
        self.gridlay.addWidget(self.fallDetectionOptionsBox,4,0,1,1)
        self.gridlay.addWidget(self.spBox,5,0,1,1)
        self.gridlay.addWidget(self.boxTab,6,0,1,1)
        self.gridlay.setRowStretch(7,1) # Added to preserve spacing
        self.gridlay.addWidget(self.graphTabs,0,1,8,1)
        self.gridlay.addWidget(self.colorGradient, 0, 2, 8, 1)

        self.gridlay.setColumnStretch(0,1)
        self.gridlay.setColumnStretch(1,3)
        self.setLayout(self.gridlay)

        # Set up parser
        self.parser = uartParser(type=self.configType.currentText())

        # Check cached data for previously used demo and device to set as default options
        deviceName = cachedData.getCachedDeviceName()
        if (deviceName != ""):
            try:
                self.deviceType.setCurrentIndex(DEVICE_LIST.index(deviceName))
                if (deviceName == "IWR6843" or deviceName == "IWR1843"):
                    self.parserType = "DoubleCOMPort"
                elif (deviceName == "IWRL6432"):
                    self.parserType = "SingleCOMPort"
                elif (deviceName == "IWRL1432"):
                    self.parserType = "SingleCOMPort"
            except:
                print("Device not found. Using default option")
                self.deviceType.setCurrentIndex(0)
        demoName = cachedData.getCachedDemoName()
        if (demoName != ""):
            try:
                if (self.deviceType.currentText() in DEVICE_LIST[0:2]):
                    self.configType.setCurrentIndex(x843_DEMO_TYPES.index(demoName))
                if (self.deviceType.currentText() in DEVICE_LIST[2:3]):
                    self.configType.setCurrentIndex(x432_DEMO_TYPES.index(demoName))
                if (self.deviceType.currentText() in DEVICE_LIST[3:]):
                    self.configType.setCurrentIndex(xWRL1432_DEMO_TYPES.index(demoName))

            except:
                print("Demo not found. Using default option")
                self.configType.setCurrentIndex(0)

    def initConnectionPane(self):
        self.comBox = QGroupBox('Connect to Com Ports')
        self.cliCom = QLineEdit('')
        self.dataCom = QLineEdit('')
        self.connectStatus = QLabel('Not Connected')
        self.connectButton = QPushButton('Connect')
        self.saveBinaryBox = QCheckBox('Save UART')
        self.connectButton.clicked.connect(self.connectCom)
        self.configType = QComboBox()
        self.deviceType = QComboBox()

        # TODO Add fall detection support
        # TODO Add replay support
        self.configType.addItems(x843_DEMO_TYPES)
        self.configType.currentIndexChanged.connect(self.onChangeConfigType)        
        self.deviceType.addItems(DEVICE_LIST)
        self.deviceType.currentIndexChanged.connect(self.onChangeDeviceType)
        self.comLayout = QGridLayout()
        self.comLayout.addWidget(QLabel('Device:'),0,0)
        self.comLayout.addWidget(self.deviceType,0,1)
        self.comLayout.addWidget(QLabel('CLI COM:'),1,0)
        self.comLayout.addWidget(self.cliCom,1,1)
        self.comLayout.addWidget(QLabel('DATA COM:'),2,0)
        self.comLayout.addWidget(self.dataCom,2,1)
        self.comLayout.addWidget(QLabel('Demo:'),3,0)
        self.comLayout.addWidget(self.configType,3,1)
        self.comLayout.addWidget(self.connectButton,4,0)
        self.comLayout.addWidget(self.connectStatus,4,1)
        self.comLayout.addWidget(self.saveBinaryBox,5,0)
        self.saveBinaryBox.stateChanged.connect(self.saveBinaryBoxChanged)

        self.comBox.setLayout(self.comLayout)
        self.configType.setCurrentIndex(0) # initialize this to a stable value

        # Find all Com Ports
        serialPorts = list(serial.tools.list_ports.comports())

        # Find default CLI Port and Data Port
        for port in serialPorts:
            if (CLI_XDS_SERIAL_PORT_NAME in port.description or CLI_SIL_SERIAL_PORT_NAME in port.description):
                print(f'\tCLI COM Port found: {port.device}')
                comText = port.device
                comText = comText.replace("COM", "")
                self.cliCom.setText(comText)

            elif (DATA_XDS_SERIAL_PORT_NAME in port.description or DATA_SIL_SERIAL_PORT_NAME in port.description):
                print(f'\tData COM Port found: {port.device}')
                comText = port.device
                comText = comText.replace("COM", "")
                self.dataCom.setText(comText)


    def initStatsPane(self):
        self.statBox = QGroupBox('Statistics')
        self.frameNumDisplay = QLabel('Frame: 0')
        self.plotTimeDisplay = QLabel('Average Plot Time: 0 ms')
        self.numPointsDisplay = QLabel('Points: 0')
        self.numTargetsDisplay = QLabel('Targets: 0')
        self.avgPower = QLabel('Average Power: 0 mw')
        self.statsLayout = QVBoxLayout()
        self.statsLayout.addWidget(self.frameNumDisplay)
        self.statsLayout.addWidget(self.plotTimeDisplay)
        self.statsLayout.addWidget(self.numPointsDisplay)
        self.statsLayout.addWidget(self.numTargetsDisplay)
        self.statsLayout.addWidget(self.avgPower)
        self.statBox.setLayout(self.statsLayout)

    def fallDetDisplayChanged(self, newState):
        if(newState == 2):
            self.fallDetectionOptionsBox.setVisible(True)
        else:
            self.fallDetectionOptionsBox.setVisible(False)

    def saveBinaryBoxChanged(self, newState):
        if(newState == 2):
            self.parser.setSaveBinary(True)
        else:
            self.parser.setSaveBinary(False)

    def initPlotControlPane(self):
        self.plotControlBox = QGroupBox('Plot Controls')
        self.pointColorMode = QComboBox()
        self.pointColorMode.addItems([COLOR_MODE_SNR, COLOR_MODE_HEIGHT, COLOR_MODE_DOPPLER, COLOR_MODE_TRACK])
        self.plotTracks = QCheckBox('Plot Tracks')
        self.displayFallDet = QCheckBox('Detect Falls')
        self.displayFallDet.stateChanged.connect(self.fallDetDisplayChanged)
        self.persistentFramesInput = QComboBox()
        self.persistentFramesInput.addItems([str(i) for i in range(1, MAX_PERSISTENT_FRAMES + 1)])
        self.persistentFramesInput.setCurrentIndex(2)
        self.plotControlLayout = QFormLayout()
        self.plotControlLayout.addRow("Color Points By:",self.pointColorMode)
        self.plotControlLayout.addRow(self.plotTracks, self.displayFallDet)
        self.plotControlLayout.addRow("# of Persistent Frames",self.persistentFramesInput)
        self.plotControlBox.setLayout(self.plotControlLayout)
        # Initialize button values
        self.plotTracks.setChecked(True)

    def updateFallDetectionSensitivity(self):
        self.fallDetection.setFallSensitivity(((self.fallDetSlider.value() / self.fallDetSlider.maximum()) * 0.4) + 0.4) # Range from 0.4 to 0.8

    def initFallDetectPane(self):
        self.fallDetectionOptionsBox = QGroupBox('Fall Detection Sensitivity')
        self.fallDetLayout = QGridLayout()
        self.fallDetSlider = fallDetectionSliderClass(Qt.Horizontal)
        self.fallDetSlider.setTracking(True)
        self.fallDetSlider.setTickPosition(QSlider.TicksBothSides)
        self.fallDetSlider.setTickInterval(10)
        self.fallDetSlider.setRange(0, 100)
        self.fallDetSlider.setSliderPosition(50)
        self.fallDetSlider.valueChanged.connect(self.updateFallDetectionSensitivity)
        self.lessSensitiveLabel = QLabel("Less Sensitive")
        self.fallDetLayout.addWidget(self.lessSensitiveLabel,0,0,1,1)
        self.moreSensitiveLabel = QLabel("More Sensitive")
        self.fallDetLayout.addWidget(self.moreSensitiveLabel,0,10,1,1)
        self.fallDetLayout.addWidget(self.fallDetSlider,1,0,1,11)
        self.fallDetectionOptionsBox.setLayout(self.fallDetLayout)
        if(self.displayFallDet.checkState() == 2):
            self.fallDetectionOptionsBox.setVisible(True)
        else:
            self.fallDetectionOptionsBox.setVisible(False)

    def initConfigPane(self):
        self.configBox = QGroupBox('Configuration')
        self.selectConfig = QPushButton('Select Configuration')
        self.sendConfig = QPushButton('Start and Send Configuration')
        self.start = QPushButton("Start without Send Configuration ")
        self.selectConfig.clicked.connect(self.selectCfg)
        self.sendConfig.clicked.connect(self.sendCfg)
        self.start.clicked.connect(self.startApp)     
        self.configLayout = QVBoxLayout()
        self.configLayout.addWidget(self.selectConfig)
        self.configLayout.addWidget(self.sendConfig)
        self.configLayout.addWidget(self.start)      
        #self.configLayout.addStretch(1)
        self.configBox.setLayout(self.configLayout)


    def setControlLayout(self):
        self.controlBox = QGroupBox('Control')
        self.rangecfar = QSlider(Qt.Horizontal)
        self.azcfar = QSlider(Qt.Horizontal)
        self.snrthresh = QSlider(Qt.Horizontal)
        self.pointsthresh = QSlider(Qt.Horizontal)
        self.gatinggain = QSlider(Qt.Horizontal)
        self.controlLayout = QVBoxLayout()
        self.rangelabel = QLabel('Range CFAR Threshold: ')
        self.azlabel = QLabel('Azimuth CFAR Threshold: ')
        self.snrlabel = QLabel('SNR Threshold: ')
        self.pointslabel = QLabel('Points Threshold: ')
        self.gatinglabel = QLabel('Gating Gain: ')
        self.controlLayout.addWidget(self.rangelabel)
        self.controlLayout.addWidget(self.rangecfar)
        self.controlLayout.addWidget(self.azlabel)
        self.controlLayout.addWidget(self.azcfar)
        self.controlLayout.addWidget(self.snrlabel)
        self.controlLayout.addWidget(self.snrthresh)
        self.controlLayout.addWidget(self.pointslabel)
        self.controlLayout.addWidget(self.pointsthresh)
        self.controlLayout.addWidget(self.gatinglabel)
        self.controlLayout.addWidget(self.gatinggain)
        self.controlBox.setLayout(self.controlLayout)


    # Boundary box control section
    def setBoxControlLayout(self, name):
        # Set up one boundary box control
        boxControl = QGroupBox(name)
        
        description = QLabel('')
        # Input boxes
        lx = QLineEdit('-6')
        rx = QLineEdit('6')
        ny = QLineEdit('0')
        fy = QLineEdit('6')
        bz = QLineEdit('-6')
        tz = QLineEdit('6')
        enable = QCheckBox()

        # Set up color options
        color = QComboBox()
        color.addItem('Blue', 'b')
        color.addItem('Red', 'r')
        color.addItem('Green', 'g')
        color.addItem('Yellow', 'y')
        color.addItem('Cyan', 'c')
        color.addItem('Magenta', 'm')
        # color.addItem('Black', 'k')
        color.addItem('White', 'w')
        
        boxConLayout = QGridLayout()

        boxConLayout.addWidget(QLabel('Description:'),0,0,1,1)
        boxConLayout.addWidget(description,0,1,1,2)
        boxConLayout.addWidget(QLabel('Left X'),1,0,1,1)
        boxConLayout.addWidget(lx,1,1,1,1)
        boxConLayout.addWidget(QLabel('Right X'),1,2,1,1)
        boxConLayout.addWidget(rx,1,3,1,1)
        boxConLayout.addWidget(QLabel('Near Y'),2,0,1,1)
        boxConLayout.addWidget(ny,2,1,1,1)
        boxConLayout.addWidget(QLabel('Far Y'),2,2,1,1)
        boxConLayout.addWidget(fy,2,3,1,1)
        boxConLayout.addWidget(QLabel('Bottom Z'),3,0,1,1)
        boxConLayout.addWidget(bz,3,1,1,1)
        boxConLayout.addWidget(QLabel('Top Z'),3,2,1,1)
        boxConLayout.addWidget(tz,3,3,1,1)
        boxConLayout.addWidget(QLabel('Color'),4,0,1,1)
        boxConLayout.addWidget(color,4,1,1,1)
        boxConLayout.addWidget(QLabel('Enable Box'),4,2,1,1)
        boxConLayout.addWidget(enable,4,3,1,1)
        boxControl.setLayout(boxConLayout)
        boundList = [lx,rx,ny,fy,bz,tz]

        # Connect onchange listeners
        for text in boundList:
            text.textEdited.connect(self.onChangeBoundaryBox)
        enable.stateChanged.connect(self.onChangeBoundaryBox)
        color.currentIndexChanged.connect(self.onChangeBoundaryBox)
        # Return dictionary of all related controls for this box
        return {'name':name, 'boxCon':boxControl, 'boundList':boundList, 'checkEnable':enable, 'description':description, 'color':color}


    def initSensorPositionPane(self):
        self.az_tilt = QLineEdit('0')
        self.elev_tilt = QLineEdit('0')
        self.s_height = QLineEdit(str(self.profile['sensorHeight']))
        self.spLayout = QGridLayout()
        
        self.spLayout.addWidget(QLabel('Azimuth Tilt'),0,0,1,1)
        self.spLayout.addWidget(self.az_tilt,0,1,1,1)
        self.spLayout.addWidget(QLabel('Elevation Tilt'),1,0,1,1)
        self.spLayout.addWidget(self.elev_tilt,1,1,1,1)
        self.spLayout.addWidget(QLabel('Sensor Height'),2,0,1,1)
        self.spLayout.addWidget(self.s_height,2,1,1,1)
        
        self.spBox = QGroupBox('Sensor Position')
        self.spBox.setLayout(self.spLayout)
        self.s_height.textEdited.connect(self.onChangeSensorPosition)
        self.az_tilt.textEdited.connect(self.onChangeSensorPosition)
        self.elev_tilt.textEdited.connect(self.onChangeSensorPosition)
        # Force an update so that sensor is at default postion
        self.onChangeSensorPosition()


    def onChangeConfigType(self):
        newConfig = self.configType.currentText()
        cachedData.setCachedDemoName(newConfig)
        print('Demo Changed to: ' + newConfig)
        
        # First, undo any changes that the last demo made
        # These should be the inverse of the changes made in 2nd part of this function

        # Undo OOB
        if (self.prevConfig == DEMO_NAME_OOB):
            # Unlock plot tracks
            self.plotTracks.setChecked(True)
            self.plotTracks.setDisabled(True)
        elif(self.prevConfig  == DEMO_NAME_x432_OOB):
            # Unlock plot tracks
            self.plotTracks.setChecked(True)
            self.plotTracks.setDisabled(True)
        # Undo 3D People Tracking
        elif (self.prevConfig == DEMO_NAME_3DPC):
            self.pointColorMode.setCurrentText(COLOR_MODE_SNR)
            self.displayFallDet.setChecked(False)
            self.displayFallDet.setDisabled(True)
        # Undo Vitals
        elif (self.prevConfig == DEMO_NAME_VITALS):
            self.vitalsPane.setVisible(False)
            self.pointColorMode.setCurrentText(COLOR_MODE_SNR)
        elif (self.prevConfig == DEMO_NAME_x432_VITAL_SIGNS):
            self.vitalsPane.setVisible(False)
            self.pointColorMode.setCurrentText(COLOR_MODE_SNR)
            self.vitalsIWRL6432 = 0
        # Undo Long Range People Detection
        elif (self.prevConfig == DEMO_NAME_LRPD):
            self.pointColorMode.setCurrentText(COLOR_MODE_SNR)
        # Undo Mobile Tracker
        elif (self.prevConfig == DEMO_NAME_MT):
            self.pointColorMode.setCurrentText(COLOR_MODE_SNR)
        # Undo Small Obstacle
        elif (self.prevConfig == DEMO_NAME_SOD or self.prevConfig == DEMO_NAME_x432_SOD):
            # Unlock boundary box config
            for box in self.boundaryBoxes:
                if ('occZone' in box['name']):
                    # Unlock each text field
                    for textBox in box['boundList']:
                        textBox.setDisabled(False)
                    # Unlock enable box
                    box['checkEnable'].setDisabled(False)
                    box['color'].setDisabled(False)

            # Unlock sensor position config
            self.spBox.setDisabled(False)
        # Undo Gesture 
        elif (self.prevConfig == DEMO_NAME_GESTURE or self.prevConfig == DEMO_NAME_x432_GESTURE):
            self.graphTabs.removeTab(0)
            self.init3dGraph()
            self.init1dGraph()
            self.graphTabs.addTab(self.pcplot, '3D Plot')
            self.graphTabs.addTab(self.rangePlot, 'Range Plot')
            self.graphTabs.currentChanged.connect(self.whoVisible)
            if self.configType.currentText() == DEMO_NAME_x432_GESTURE:
                self.gestureSetupBox.hide()
            self.plotControlBox.show()
            self.spBox.show()
            self.boxTab.show()
        # Undo Surface Classification 
        elif (self.prevConfig == DEMO_NAME_SURFACE or self.prevConfig == DEMO_NAME_x432_SURFACE):
            if (self.prevConfig == DEMO_NAME_x432_SURFACE):
                self.graphTabs.removeTab(0)
            self.graphTabs.removeTab(0)
            self.init3dGraph()
            self.graphTabs.addTab(self.pcplot, '3D Plot')
            self.init1dGraph()
            self.graphTabs.addTab(self.rangePlot, 'Range Plot')
            self.graphTabs.currentChanged.connect(self.whoVisible)
            self.surfaceSetupBox.hide()
            self.plotControlBox.show()
            self.spBox.show()
            self.boxTab.show()
        # Undo Level Sensing 
        elif (self.prevConfig == DEMO_NAME_x432_LEVEL_SENSING or self.prevConfig == DEMO_NAME_xWRL1432_LEVEL_SENSING):
            self.graphTabs.removeTab(0)
            self.graphTabs.currentChanged.connect(self.whoVisible)
            self.graphTabs.removeTab(0)
            self.init3dGraph()
            self.graphTabs.addTab(self.pcplot, '3D Plot')
            self.init1dGraph()
            self.graphTabs.addTab(self.rangePlot, 'Range Plot')
            self.plotControlBox.show()
            self.spBox.show()
            self.boxTab.show()
        # Undo True Ground Speed
        elif (self.prevConfig == DEMO_NAME_x432_GROUND_SPEED):
            self.graphTabs.removeTab(0)
            self.init3dGraph()
            self.init1dGraph()
            self.graphTabs.addTab(self.pcplot, '3D Plot')
            self.graphTabs.addTab(self.rangePlot, 'Range Plot')
            self.graphTabs.currentChanged.connect(self.whoVisible)
            self.displayFallDet.setDisabled(False)

        # Now, apply any specific GUI changes for the new demo
        # Configure for Out of Box
        if (newConfig == DEMO_NAME_OOB):
            # Lock plot tracks off
            self.plotTracks.setChecked(False)
            self.plotTracks.setDisabled(False)
        # Configure for x432 Out of Box
        elif (newConfig == DEMO_NAME_x432_OOB):
            self.pointColorMode.setCurrentText(COLOR_MODE_TRACK)
        # Configure for 3D People Counting
        elif (newConfig == DEMO_NAME_3DPC):
            self.pointColorMode.setCurrentText(COLOR_MODE_TRACK)
            self.displayFallDet.setDisabled(False)
        # Configure For Vitals
        elif (newConfig == DEMO_NAME_VITALS):
            self.initVitalsPlots()
            self.gridlay.addWidget(self.vitalsPane, 0, 3, 8, 1)
            self.pointColorMode.setCurrentText(COLOR_MODE_TRACK)
            self.vitalsPane.setVisible(True)
        elif (newConfig == DEMO_NAME_x432_VITAL_SIGNS):
            self.vitalsIWRL6432 = 1
            self.initVitalsPlots()
            self.gridlay.addWidget(self.vitalsPane, 0, 3, 8, 1)
            self.pointColorMode.setCurrentText(COLOR_MODE_TRACK)
            self.vitalsPane.setVisible(True)
        # Configure for Long Range People Detection
        elif (newConfig == DEMO_NAME_LRPD):
            self.pointColorMode.setCurrentText(COLOR_MODE_TRACK)
        # Configure for Mobile Tracker
        elif (newConfig == DEMO_NAME_MT):
            self.pointColorMode.setCurrentText(COLOR_MODE_TRACK)
        # Configure for Small Obstacle
        elif (newConfig == DEMO_NAME_SOD or newConfig == DEMO_NAME_x432_SOD):
            # Lock boundary boxes for occ state machine
            for box in self.boundaryBoxes:
                if ('occZone' in box['name']):
                    # Lock each text field
                    for textBox in box['boundList']:
                        textBox.setDisabled(True)
                    # Lock enable box
                    box['checkEnable'].setDisabled(True)
                    box['color'].setDisabled(True)

            # Lock sensor position config
            self.s_height.setText('1')
            self.az_tilt.setText('0')
            self.elev_tilt.setText('0')
            self.spBox.setDisabled(True)
            self.onChangeSensorPosition()
        # Configure for Gesture
        elif (newConfig == DEMO_NAME_GESTURE or newConfig == DEMO_NAME_x432_GESTURE):
            # IWR6843 Gesture Recognition Demo
            if newConfig == DEMO_NAME_GESTURE:
                # Probability and count thresholds for post processing of neural network outputs
                # [No Gesture, Gesture1, Gesture2, ...]
                self.probabilityThresholds = [0.99, 0.6, 0.6, 0.6, 0.6, 0.9, 0.9, 0.6, 0.6, 0.99]
                self.countThresholds = [4, 4, 4, 4, 4, 9, 9, 4, 4, 8]
                self.numGestures = 9
                self.contGestureFramecount = 10
                # List of gesture strings
                self.gestureList = ['  No Gesture   ', 
                                    ' Left to Right ', 
                                    ' Right to Left ',
                                    '  Up to Down   ', 
                                    '  Down to Up   ', 
                                    '   CW Twirl    ', 
                                    '   CCW Twirl   ', 
                                    '      On       ', 
                                    '      Off      ', 
                                    '     Shine     ']
                self.sumProbs = [0] * len(self.gestureList) * GESTURE_FEATURE_LENGTH
                self.gesture_featurePlots = {}
                self.gesture_featureVals = {'dopplerAvgVals': [], 'rangeAvgVals': [], 'numPointsVals': []}
            # IWRL6432 Gesture Recognition Demo
            elif newConfig == DEMO_NAME_x432_GESTURE:
                # self.currentGestureMode = GESTURE_GESTURE_MODE_x432
                self.numGestures = 7
                # List of gesture strings
                self.gestureList = ['  No Gesture   ', 
                                    ' Left to Right ', 
                                    ' Right to Left ', 
                                    '  Up to Down   ', 
                                    '  Down to Up   ', 
                                    '     Push      ', 
                                    '     Pull      ']

                self.gesture_featureVals = {'dopplerAvgVals': [], 'rangeAvgVals': [], 'numPointsVals': []}
                self.gesture_featurePlots = {}
    
            self.currFramegesture = -1
            self.prevFramegesture = -1

            self.lastFrameProcd = -1

            for i in range(self.graphTabs.count()):
                self.graphTabs.removeTab(0)

            self.initGestureTab()
            self.graphTabs.addTab(self.gestureTab, 'Gesture')
            self.graphTabs.currentChanged.connect(self.whoVisible)
            self.plotControlBox.hide()
            self.spBox.hide()
            self.boxTab.hide()

        # Configure for Surface Classification
        elif (newConfig == DEMO_NAME_SURFACE or newConfig == DEMO_NAME_x432_SURFACE):
            # List of most recent frames of data collected. We will base our classification off of this
            self.surfaceLatestResults = deque(100*[0], 100)
            # List of surfaces
            self.surfaceList = ['Not Grass', 'Grass']
            self.currFrameClassification = -1

            self.init3dGraph()
            self.init1dGraph()

            for i in range(self.graphTabs.count()):
                self.graphTabs.removeTab(0)

            self.initSurfaceClassificationTab()
            self.graphTabs.addTab(self.surfaceTab, 'Surface Classification')
            if (newConfig == DEMO_NAME_x432_SURFACE):
                self.graphTabs.addTab(self.rangePlot, 'Range Plot')
            self.graphTabs.currentChanged.connect(self.whoVisible)
                        
            self.plotControlBox.hide()
            self.spBox.hide()
            self.boxTab.hide()

        elif (newConfig == DEMO_NAME_x432_LEVEL_SENSING or newConfig == DEMO_NAME_xWRL1432_LEVEL_SENSING): 
            for i in range(self.graphTabs.count()):
                self.graphTabs.removeTab(0)
                    
            self.Peak1 = 0
            self.Peak2 = 0
            self.Peak3 = 0
            self.Peak1Magnitude = 0
            self.Peak2Magnitude = 0
            self.Peak3Magnitude = 0
            self.peakValues = []
                            
            self.initLevelSensingGraph()        
            self.graphTabs.addTab(self.levelsensingTab, 'Level Sensing')
            self.graphTabs.currentChanged.connect(self.whoVisible)

            self.plotControlBox.hide()
            self.spBox.hide()
            self.boxTab.hide()

        elif (newConfig == DEMO_NAME_x432_GROUND_SPEED):
            
            self.speedPlots = {}
            self.speedVals = {'speedVals': []}
            
            for i in range(self.graphTabs.count()):
                self.graphTabs.removeTab(0)
            
            self.initGroundSpeedTab()
            self.graphTabs.addTab(self.groundSpeedTab, 'True Ground Speed')
            self.graphTabs.currentChanged.connect(self.whoVisible)
            self.displayFallDet.setDisabled(True)

        # Save this so that the next time we change configs we know what to undo
        self.prevConfig = newConfig

    # Callback function to reset settings when device is changed
    def onChangeDeviceType(self):
        newDevice = self.deviceType.currentText()
        cachedData.setCachedDeviceName(newDevice)
        print('Device Changed to: ' + newDevice)
        if(newDevice in DEVICE_LIST[0:2]):
            self.configType.currentIndexChanged.disconnect()     
            self.dataCom.setEnabled(True)
            self.configType.clear()
            self.configType.addItems(x843_DEMO_TYPES)
            self.parser.parserType = "DoubleCOMPort" # DoubleCOMPort refers to xWRx843 parts
            self.configType.setCurrentIndex(-1)
            self.configType.currentIndexChanged.connect(self.onChangeConfigType)
            self.configType.setCurrentIndex(0)

        if(newDevice in DEVICE_LIST[2:3]):
            self.configType.currentIndexChanged.disconnect()
            self.dataCom.setText(self.cliCom.text())
            self.dataCom.setEnabled(False)
            self.configType.clear()
            self.configType.addItems(x432_DEMO_TYPES)
            self.parser.parserType = "SingleCOMPort" # SingleCOMPort refers to xWRLx432 parts
            self.configType.setCurrentIndex(-1)
            self.configType.currentIndexChanged.connect(self.onChangeConfigType)
            self.configType.setCurrentIndex(0)
            
        if(newDevice in DEVICE_LIST[3:]):
            self.configType.currentIndexChanged.disconnect()
            self.dataCom.setText(self.cliCom.text())
            self.dataCom.setEnabled(False)
            self.configType.clear()
            self.configType.addItems(xWRL1432_DEMO_TYPES)
            self.parser.parserType = "SingleCOMPort" # SingleCOMPort refers to xWRLx432 parts
            self.configType.setCurrentIndex(-1)
            self.configType.currentIndexChanged.connect(self.onChangeConfigType)
            self.configType.setCurrentIndex(0)
            
            
    # Gets called whenever the sensor position box is modified
    def onChangeSensorPosition(self):
        try:
            newHeight = float(self.s_height.text())
            newAzTilt = float(self.az_tilt.text())
            newElevTilt = float(self.elev_tilt.text())
        except:
            print("Error in gui_main.py: Failed to update sensor position")
            return
        command = "sensorPosition " + self.s_height.text() + " " + self.az_tilt.text() + " " + self.elev_tilt.text() + " \n"
        # self.cThread = sendCommandThread(self.parser,command)
        # self.cThread.start(priority=QThread.HighestPriority-2)

        # Update Profile info
        self.profile['sensorHeight'] = newHeight

        # Move evmBox to new position
        self.evmBox.resetTransform()
        self.evmBox.rotate(-1*newElevTilt,1,0,0)
        self.evmBox.rotate(-1*newAzTilt,0,0,1)
        self.evmBox.translate(0,0,newHeight)


    def initBoundaryBoxPane(self):
        # Set up all boundary box controls
        self.boundaryBoxes = []
        self.boxTab = QTabWidget()
        self.addBoundBox('pointBounds')
    

    # For live tuning when available
    def onChangeBoundaryBox(self):
        index = 0
        for box in self.boundaryBoxes:
            # Update dimensions of box
            try:
                # If it's a radial box we need to render it differently than the other boxes
                if('mpdArc' in box['name']):
                    rl = float(box['boundList'][0].text())
                    rr = float(box['boundList'][1].text())
                    tl = float(box['boundList'][2].text())
                    tr = float(box['boundList'][3].text())
                    zl = float(box['boundList'][4].text())
                    zr = float(box['boundList'][5].text())

                    boxLines = getBoxArcs(rl,tl,zl,rr,tr,zr)
                    boxColor = pg.glColor(box['color'].itemData(box['color'].currentIndex()))
                    self.boundaryBoxViz[index].setData(pos=boxLines,color=boxColor,width=2,antialias=True,mode='lines')
                    self.boundaryBoxViz[index].setVisible(True)

                else:
                    xl = float(box['boundList'][0].text())
                    xr = float(box['boundList'][1].text())
                    yl = float(box['boundList'][2].text())
                    yr = float(box['boundList'][3].text())
                    zl = float(box['boundList'][4].text())
                    zr = float(box['boundList'][5].text())
                    
                    boxLines = getBoxLines(xl,yl,zl,xr,yr,zr)
                    boxColor = pg.glColor(box['color'].itemData(box['color'].currentIndex()))
                    self.boundaryBoxViz[index].setData(pos=boxLines,color=boxColor,width=2,antialias=True,mode='lines')
                    # Update visibility
                    if (box['checkEnable'].isChecked()):
                        self.boundaryBoxViz[index].setVisible(True)
                        if ('pointBounds' in box['name']) :
                            self.pointBounds['enabled'] = True
                            self.pointBounds['minX'] = xl
                            self.pointBounds['maxX'] = xr
                            self.pointBounds['minY'] = yl
                            self.pointBounds['maxY'] = yr
                            self.pointBounds['minZ'] = zl
                            self.pointBounds['maxZ'] = zr
                        
                    else:
                        self.boundaryBoxViz[index].setVisible(False)
                        if ('pointBounds' in box['name']) :
                            self.pointBounds['enabled'] = False

                index = index + 1
            except:
                # You get here if you enter an invalid number
                # When you enter a minus sign for a negative value, you will end up here before you type the full number
                pass

    def initVitalsPlots(self):
        self.vitalsPane = QGroupBox('Vital Signs')
        vitalsPaneLayout = QGridLayout()
        self.vitals = []

        for i in range(MAX_VITALS_PATIENTS):
            patientDict = {}
            patientName = 'Patient' + str(i+1)
            
            # Initialize the pane and layout
            patientPane = QGroupBox(patientName)
            patientPaneLayout = QGridLayout()

            # Set up basic labels so we can edit their appearance
            statusLabel = QLabel('Patient Status:')
            breathLabel = QLabel('Breath Rate:')
            heartLabel = QLabel('Heart Rate:')
            rangeBinLabel = QLabel('Range Bin:')

            # Set up patient vitals plot
            patientDict['plot'] = pg.PlotWidget()
            patientDict['plot'].setBackground('w')
            patientDict['plot'].showGrid(x=True,y=True)
            patientDict['plot'].invertX(True)
            
            if(self.vitalsIWRL6432 == 1):
                patientDict['plot'].setXRange(0, NUM_VITALS_FRAMES_IN_PLOT_IWRL6432, padding=0.01)
                patientDict['plot'].setYRange(0,120,padding=0.1)
                patientDict['plot'].getPlotItem().setLabel('left', 'Hear Rate and Breath Rate per minute')
                patientDict['plot'].getPlotItem().setLabel('bottom', 'Vital Signs Frame Number')
            else:
                patientDict['plot'].setXRange(0,NUM_VITALS_FRAMES_IN_PLOT,padding=0.01)
                patientDict['plot'].setYRange(-1,1,padding=0.1)
                
            patientDict['plot'].setMouseEnabled(False,False)
            patientDict['heartGraph'] = pg.PlotCurveItem(pen=pg.mkPen(width=3, color='r'))
            patientDict['breathGraph'] = pg.PlotCurveItem(pen=pg.mkPen(width=3, color='b'))
            patientDict['plot'].addItem(patientDict['heartGraph'])
            patientDict['plot'].addItem(patientDict['breathGraph'])

            # Set up all other patient data fields
            patientDict['breathRate'] = QLabel('Undefined')
            patientDict['heartRate'] = QLabel('Undefined')
            patientDict['status'] = QLabel('Undefined')
            patientDict['rangeBin'] = QLabel('Undefined')
            patientDict['name'] = patientName
            
            # Format text to make it attractive
            labelFont = QFont('Arial', 16)
            labelFont.setBold(True)
            dataFont = (QFont('Arial', 12))
            heartLabel.setFont(labelFont)
            breathLabel.setFont(labelFont)
            statusLabel.setFont(labelFont)
            rangeBinLabel.setFont(labelFont)
            patientDict['breathRate'].setStyleSheet('color: blue')
            patientDict['heartRate'].setStyleSheet('color: red')
            patientDict['status'].setFont(dataFont)
            patientDict['breathRate'].setFont(dataFont)
            patientDict['heartRate'].setFont(dataFont)
            patientDict['rangeBin'].setFont(dataFont)

            # Put the widgets into the layout
            patientPaneLayout.addWidget(patientDict['plot'],2,0,1,4)
            patientPaneLayout.addWidget(statusLabel,0,0,alignment=Qt.AlignHCenter)
            patientPaneLayout.addWidget(patientDict['status'],1,0,alignment=Qt.AlignHCenter)
            patientPaneLayout.addWidget(breathLabel,0,1,alignment=Qt.AlignHCenter)
            patientPaneLayout.addWidget(patientDict['breathRate'],1,1,alignment=Qt.AlignHCenter)
            patientPaneLayout.addWidget(heartLabel,0,2,alignment=Qt.AlignHCenter)
            patientPaneLayout.addWidget(patientDict['heartRate'],1,2,alignment=Qt.AlignHCenter)
            patientPaneLayout.addWidget(rangeBinLabel,0,3,alignment=Qt.AlignHCenter)
            patientPaneLayout.addWidget(patientDict['rangeBin'],1,3,alignment=Qt.AlignHCenter)

            patientPane.setLayout(patientPaneLayout)
            patientDict['pane'] = patientPane

            # Make patient vitals data accessable by other functions
            self.vitals.append(patientDict)

            if (i != 0):
                patientPane.setVisible(False)

            # Add this patient to the overall vitals pane
            vitalsPaneLayout.addWidget(patientPane,i,0)
        
        self.vitalsPane.setLayout(vitalsPaneLayout)


    def initColorGradient(self):
        self.colorGradient = pg.GradientWidget(orientation='right')
        self.colorGradient.restoreState(self.gradientMode)
        self.colorGradient.setVisible(False)


    def init3dGraph(self):
        # Create plot
        self.pcplot = gl.GLViewWidget()
        # Sets background to a pastel grey
        self.pcplot.setBackgroundColor(70, 72, 79)
        # Create the background grid
        self.gz = gl.GLGridItem()
        self.pcplot.addItem(self.gz)

        # Create scatter plot for point cloud
        self.scatter = gl.GLScatterPlotItem(size=5)
        self.scatter.setData(pos=np.zeros((1,3)))
        self.pcplot.addItem(self.scatter)
        
        # Create box to represent EVM
        evmSizeX = 0.0625
        evmSizeZ = 0.125
        verts = np.empty((2,3,3))
        verts[0,0,:] = [-evmSizeX, 0, evmSizeZ]
        verts[0,1,:] = [-evmSizeX,0,-evmSizeZ]
        verts[0,2,:] = [evmSizeX,0,-evmSizeZ]
        verts[1,0,:] = [-evmSizeX, 0, evmSizeZ]
        verts[1,1,:] = [evmSizeX, 0, evmSizeZ]
        verts[1,2,:] = [evmSizeX, 0, -evmSizeZ]
        self.evmBox = gl.GLMeshItem(vertexes=verts,smooth=False,drawEdges=True,edgeColor=pg.glColor('r'),drawFaces=False)
        self.pcplot.addItem(self.evmBox)

        # Initialize other elements
        self.boundaryBoxViz = []
        self.coordStr = []
        self.classifierStr = []
        self.ellipsoids = []

    def initGesturePhysicalSetupPane(self):
        self.gestureSetupBox = QGroupBox('Physical Setup')

        self.gestureSetupGrid = QGridLayout()
        self.gestureSetupImg = QPixmap('images/IWRL6432_gesture_setup2.jpg')
        self.gestureImgLabel = QLabel()
        self.gestureImgLabel.setPixmap(self.gestureSetupImg)
        self.gestureSetupGrid.addWidget(self.gestureImgLabel, 1, 1)

        instructionsLabel = QLabel()
        instructionsLabel.setText("Stand 2m away, directly in front of the radar.")
        self.gestureSetupGrid.addWidget(instructionsLabel, 2, 1)
        self.gestureSetupBox.setLayout(self.gestureSetupGrid)

        self.gridlay.addWidget(self.gestureSetupBox,3,0,1,1)


    def initGestureTab(self):
        if self.configType.currentText() == DEMO_NAME_x432_GESTURE:
            self.initGesturePhysicalSetupPane()

        self.gestureTab = QWidget()
        vboxGesture = QVBoxLayout()

        hboxOutput = QHBoxLayout()

        vBoxStatus = QVBoxLayout()

        vboxDetectedGesture = QVBoxLayout()
        self.gestureOutput = QLabel("Undefined", self)
        self.gestureOutput.setAlignment(Qt.AlignCenter)
        self.gestureOutput.setStyleSheet('background-color: rgb(70, 72, 79); color: white; font-size: 60px; font-weight: bold')
        font = QFont()
        font.setPointSize(int(self.width() / 20))
        self.gestureOutput.setFont(font)
        vboxDetectedGesture.addWidget(self.gestureOutput, 1)
        vBoxStatus.addLayout(vboxDetectedGesture)

        # TODO: Add when feature is supported in demo code
        # vboxMode = QVBoxLayout()
        # self.gestureMode = QLabel("Undefined")
        # self.gestureMode.setAlignment(Qt.AlignCenter)
        # self.gestureMode.setStyleSheet('background-color: green; color: white; font-size: 60px; font-weight:bold')
        # vboxMode.addWidget(self.gestureMode,2)
        # vBoxStatus.addLayout(vboxMode, 50)
        hboxOutput.addLayout(vBoxStatus, 35)
        vboxGesture.addLayout(hboxOutput, 35)

        if self.configType.currentText() == DEMO_NAME_x432_GESTURE:
            vBoxFeatures = QVBoxLayout()
            pen = pg.mkPen(color='b', width=2, style=Qt.SolidLine)
            self.gesture_featurePlots['dopplerAvgPlot'] = pg.PlotWidget()
            self.gesture_featurePlots['dopplerAvgPlot'].setBackground((70, 72, 79))
            self.gesture_featurePlots['dopplerAvgPlot'].showGrid(x=True, y=True)
            self.gesture_featurePlots['dopplerAvgPlot'].setYRange(-15, 15)
            self.gesture_featurePlots['dopplerAvgPlot'].setXRange(1, 30)
            self.gesture_featurePlots['dopplerAvgPlot'].setTitle('Doppler Avg')
            self.gesture_featurePlots['dopplerAvgPlot'].plot(self.gesture_featureVals['dopplerAvgVals'], pen=pen)
            vBoxFeatures.addWidget(self.gesture_featurePlots['dopplerAvgPlot'])
    
            self.gesture_featurePlots['rangeAvgPlot'] = pg.PlotWidget()
            self.gesture_featurePlots['rangeAvgPlot'].setBackground((70, 72, 79))
            self.gesture_featurePlots['rangeAvgPlot'].showGrid(x=True, y=True)
            self.gesture_featurePlots['rangeAvgPlot'].setYRange(25, 36)
            self.gesture_featurePlots['rangeAvgPlot'].setXRange(1, 30)
            self.gesture_featurePlots['rangeAvgPlot'].setTitle('Range Avg')
            self.gesture_featurePlots['rangeAvgPlot'].plot(self.gesture_featureVals['rangeAvgVals'], pen=pen)
            vBoxFeatures.addWidget(self.gesture_featurePlots['rangeAvgPlot'])
    
            self.gesture_featurePlots['numPointsPlot'] = pg.PlotWidget()
            self.gesture_featurePlots['numPointsPlot'].setBackground((70, 72, 79))
            self.gesture_featurePlots['numPointsPlot'].showGrid(x=True, y=True)
            self.gesture_featurePlots['numPointsPlot'].setYRange(0, 200)
            self.gesture_featurePlots['numPointsPlot'].setXRange(1, 30)
            self.gesture_featurePlots['numPointsPlot'].setTitle('Num Points > threshold')
            self.gesture_featurePlots['numPointsPlot'].plot(self.gesture_featureVals['numPointsVals'], pen=pen)
            vBoxFeatures.addWidget(self.gesture_featurePlots['numPointsPlot'])
            vboxGesture.addLayout(vBoxFeatures, 65)

        self.gestureFontSize = '60px' 

        self.gestureTimer = QTimer()
        self.gestureTimer.setInterval(1000)
        self.gestureTimer.timeout.connect(self.resetGestureDisplay)

        self.gestureTab.setLayout(vboxGesture)
        
    def initGroundSpeedTab(self):
        
        self.groundSpeedTab = QWidget()
        vboxGroundSpeed = QVBoxLayout()

        vboxDetectedSpeed = QVBoxLayout()
        vboxDetectedSpeedMph = QVBoxLayout()
        self.speedOutput = QLabel("Undefined", self)
        self.speedOutputMph = QLabel("Undefined", self)
        self.speedOutput.setAlignment(Qt.AlignCenter)
        self.speedOutputMph.setAlignment(Qt.AlignCenter)
        self.speedOutput.setStyleSheet('background-color: rgb(70, 72, 79); color: white; font-size: 60px; font-weight: bold')
        self.speedOutputMph.setStyleSheet('background-color: rgb(70, 72, 79); color: white; font-size: 60px; font-weight: bold')
        font = QFont()
        font.setPointSize(int(self.width() / 20))
        self.speedOutput.setFont(font)
        self.speedOutputMph.setFont(font)
        vboxDetectedSpeed.addWidget(self.speedOutput, 1)
        vboxDetectedSpeedMph.addWidget(self.speedOutputMph, 1)
        vboxGroundSpeed.addLayout(vboxDetectedSpeed)
        vboxGroundSpeed.addLayout(vboxDetectedSpeedMph)

        vBoxFeatures = QVBoxLayout()
        pen = pg.mkPen(color='b', width=2, style=Qt.SolidLine)
        self.speedPlots['avgSpeedPlot'] = pg.PlotWidget()
        self.speedPlots['avgSpeedPlot'].setBackground((70, 72, 79))
        self.speedPlots['avgSpeedPlot'].showGrid(x=True, y=True)
        self.speedPlots['avgSpeedPlot'].setYRange(-7, 7)
        self.speedPlots['avgSpeedPlot'].setXRange(1, 30)
        self.speedPlots['avgSpeedPlot'].setTitle('True Ground Speed')
        self.speedPlots['avgSpeedPlot'].plot(self.speedVals['speedVals'], pen=pen)
        vBoxFeatures.addWidget(self.speedPlots['avgSpeedPlot'])

        vboxGroundSpeed.addLayout(vBoxFeatures)

        self.groundSpeedTab.setLayout(vboxGroundSpeed)

    def resetGestureDisplay(self):
        # TODO: Add when feature is supported in demo code
        # # presence mode
        # if(self.currentGestureMode == GESTURE_PRESENCE_MODE_x432):
        #     self.gestureOutput.setStyleSheet(f'background-color: black; color: white; font-size: {self.gestureFontSize}; font-weight: bold')
        #     self.gestureOutput.setText('Waiting for Presence...')
        # # gesture mode
        # elif(self.currentGestureMode == GESTURE_GESTURE_MODE_x432):
        #     self.gestureOutput.setStyleSheet(f'background-color: red; color: white; font-size: {self.gestureFontSize}; font-weight: bold')
        #     self.gestureOutput.setText(self.gestureList[0])
        self.gestureOutput.setStyleSheet(f'background-color: rgb(70, 72, 79); color: white; font-size: {self.gestureFontSize}; font-weight: bold')
        self.gestureOutput.setText(self.gestureList[0])
        self.gestureTimer.stop()

    def gestureHandler(self, gesture):
        self.updateGestureDisplay(self.gestureList[gesture])
        # TODO: Add additional functionality based on detected gesture
    
    # TODO: Add when feature is supported in demo code
    # def gesturePresenceHandler(self, gesturePresence):
    #     #if gesture/presence mode switched, 
    #     if(self.currentGestureMode != gesturePresence):
    #         if(gesturePresence==GESTURE_PRESENCE_MODE_x432):
    #             self.gestureMode.setStyleSheet('background-color: green; color: white; font-size: 60px; font-weight:bold')
    #             self.gestureMode.setText("Low Power Mode")
    #         elif(gesturePresence==GESTURE_GESTURE_MODE_x432):
    #             self.gestureMode.setStyleSheet('background-color: orange; color: white; font-size: 60px; font-weight:bold')
    #             self.gestureMode.setText("Gesture Mode")              
    #         self.currentGestureMode = gesturePresence
    #         self.resetGestureDisplay()    
    
    def updateGroundSpeedPlot(self, velocity):
        pen = pg.mkPen(color='b', width=2, style=Qt.SolidLine)
        speedData = collections.deque(self.speedVals['speedVals'])
        speedData.appendleft(velocity) # doppler avg feature is at index 1
        if (len(speedData) > 40):
            speedData.pop()
        self.speedVals['speedVals'] = speedData
        self.speedPlots['avgSpeedPlot'].clear()
        self.speedPlots['avgSpeedPlot'].plot(self.speedVals['speedVals'], pen=pen)
    
    def updateGestureFeatures(self, features):
        pen = pg.mkPen(color='b', width=2, style=Qt.SolidLine)
        # Update doppler avg feature plot
        dopplerAvgData = collections.deque(self.gesture_featureVals['dopplerAvgVals'])
        dopplerAvgData.appendleft(features[1]) # doppler avg feature is at index 1
        if (len(dopplerAvgData) > 40):
            dopplerAvgData.pop()
        self.gesture_featureVals['dopplerAvgVals'] = dopplerAvgData
        self.gesture_featurePlots['dopplerAvgPlot'].clear()
        self.gesture_featurePlots['dopplerAvgPlot'].plot(self.gesture_featureVals['dopplerAvgVals'], pen=pen)

        # Update range avg feature plot
        rangeAvgData = collections.deque(self.gesture_featureVals['rangeAvgVals'])
        rangeAvgData.appendleft(features[0]) # range avg feature is at index 0
        if (len(rangeAvgData) > 40):
            rangeAvgData.pop()
        self.gesture_featureVals['rangeAvgVals'] = rangeAvgData
        self.gesture_featurePlots['rangeAvgPlot'].clear()
        self.gesture_featurePlots['rangeAvgPlot'].plot(self.gesture_featureVals['rangeAvgVals'], pen=pen)

        # Update num points feature plot
        numPointsData = collections.deque(self.gesture_featureVals['numPointsVals'])
        numPointsData.appendleft(features[4]) # num points feature is at index 4
        if (len(numPointsData) > 40):
            numPointsData.pop()
        self.gesture_featureVals['numPointsVals'] = numPointsData
        self.gesture_featurePlots['numPointsPlot'].clear()
        self.gesture_featurePlots['numPointsPlot'].plot(self.gesture_featureVals['numPointsVals'], pen=pen)

    def updateLevelSensingPeaks(self):
        comment1 = "Object 1 in meters : "
        label_text1 = f"{self.Peak1}"
        self.PeakListOutput1.setText(label_text1)        
        
        comment2 = "Object 2 in meters : "
        label_text2 = f"{self.Peak2}"
        self.PeakListOutput2.setText(label_text2)    
        
        comment3 = "Object 3 in meters : "
        label_text3 = f"{self.Peak3}"
        self.PeakListOutput3.setText(label_text3)
        
        comment1 = "Object 1 power in dB : "
        label_text1 = f"{self.Peak1Magnitude}"
        self.PeakMagnitudeOutput1.setText(label_text1)        
        
        comment2 = "Object 2 power in dB : "
        label_text2 = f"{self.Peak2Magnitude}"
        self.PeakMagnitudeOutput2.setText(label_text2)    
        
        comment3 = "Object 3 power in dB : "
        label_text3 = f"{self.Peak3Magnitude}"
        self.PeakMagnitudeOutput3.setText(label_text3)
        
    def updateLevelSensingPower(self, powerData):
        llPower = (powerData['power1v2'] \
            + powerData['power1v2RF'] + powerData['power1v8'] + powerData['power3v3']) * 0.1
        if( powerData['power1v2'] == 65535 ):
           llPower = 0
        else:
           llPower = round(llPower, 2)
  
        power_comment = "Power in mW: "
        power_label = f"{power_comment}{llPower}"
        self.PowerOutput.setText(power_label)

    def updatePowerNumbers(self, powerData):
        if powerData['power1v2'] == 65535:
            self.avgPower.setText('Average Power: N/A')
        else:
            powerStr = str((powerData['power1v2'] \
                + powerData['power1v2RF'] + powerData['power1v8'] + powerData['power3v3']) * 0.1)
            self.avgPower.setText('Average Power: ' + powerStr[:5] + ' mW')

    # Perform post processing on the raw probabilities output by the neural network.
    # Uses a probabilities threshold and count threshold to deterrmine if a gesture has occured.
    def gesturePostProc(self, ann_probs):
        numOutputProbs = len(self.gestureList)

        i = 0
        j = 0
        confSum = 0

        # Shift the existing values
        for i in range(GESTURE_FEATURE_LENGTH * numOutputProbs - numOutputProbs):
            self.sumProbs[i] = self.sumProbs[i + numOutputProbs]

        #  Add the values for the current frame
        for i in range(numOutputProbs):
            if ann_probs[i] >= self.probabilityThresholds[i]:
                self.sumProbs[GESTURE_FEATURE_LENGTH * numOutputProbs - numOutputProbs + i] = 1
            else:
                self.sumProbs[GESTURE_FEATURE_LENGTH * numOutputProbs - numOutputProbs + i] = 0

        self.currFramegesture = 0

        for i in range(numOutputProbs):
            confSum = 0

            for j in range(GESTURE_FEATURE_LENGTH):
                confSum += self.sumProbs[j*numOutputProbs + i]

            # Sum must be larger than count threshold to be considered a gesture
            if confSum > self.countThresholds[i]:
                self.currFramegesture = i

        if self.prevFramegesture != self.currFramegesture:
            if self.currFramegesture != GESTURE_NO_GESTURE_6843:
                self.gestureHandler(self.currFramegesture) 
        else:
            if self.frameNum % self.contGestureFramecount == 0 :
                if self.currFramegesture == GESTURE_CW_TWIRL_6843 or self.currFramegesture == GESTURE_CCW_TWIRL_6843 or self.currFramegesture == GESTURE_SHINE_6843:
                    self.gestureHandler(self.currFramegesture) 

        self.prevFramegesture = self.currFramegesture


    def updateGestureDisplay(self, text):
        self.gestureOutput.setStyleSheet(f'background-color: blue; color: white; font-size: {self.gestureFontSize}; font-weight: bold')
        self.gestureOutput.setText(text)
        self.gestureTimer.start()

    def updateGroundSpeedDisplay(self, text):
        self.speedOutput.setStyleSheet(f'background-color: blue; color: white; font-weight: bold')
        self.speedOutput.setText(text)
        
    def updateGroundSpeedMphDisplay(self, text):
        self.speedOutputMph.setStyleSheet(f'background-color: blue; color: white; font-weight: bold')
        self.speedOutputMph.setText(text)

    def initSurfacePhysicalSetupPane(self):
        self.surfaceSetupBox = QGroupBox('Physical Setup')

        self.gestureSetupGrid = QGridLayout()
        self.gestureSetupImg = QPixmap('images/surface_setup.png')
        self.gestureImgLabel = QLabel()
        self.gestureImgLabel.setPixmap(self.gestureSetupImg)
        self.gestureSetupGrid.addWidget(self.gestureImgLabel, 1, 1)

        self.surfaceSetupBox.setLayout(self.gestureSetupGrid)

        self.gridlay.addWidget(self.surfaceSetupBox,3,0,1,1) 

    def initSurfaceClassificationTab(self):

        self.initSurfacePhysicalSetupPane()

        self.surfaceTab = QWidget()
        vboxSurface = QVBoxLayout()

        vboxOutput = QGridLayout()
        self.surfaceOutput = QLabel("<b>Grass Classification</b><br>" + str(self.surfaceList[0]), self)
        self.surfaceOutput.setAlignment(Qt.AlignCenter)
        self.surfaceOutput.setStyleSheet('background-color: #46484f; color: white; font-size: 40px; font-weight: light')
        
        self.surfaceOutputRaw = QLabel("<b>Grass Probability</b><br>0.0%", self)
        self.surfaceOutputRaw.setAlignment(Qt.AlignCenter)
        self.surfaceOutputRaw.setStyleSheet('background-color: #46484f; color: white; font-size: 40px; font-weight: light')

        surfaceDescStr = """
        <p style="font-size: 30px"><b>Sensor Setup:</b></p><p style="font-size: 20px">18cm off the ground with 27 degree tilt off the vertical</p>
        <p style="font-size: 30px"><b>Model:       </b></p><p style="font-size: 20px">Sequential model trained on grass and large stone pavers</p>
        <p style="font-size: 30px"><b>More info:   </b></p><p style="font-size: 20px">See User Guide in the Radar Toolbox on dev.ti.com       </p>
        """
        self.surfaceDesc = QLabel(surfaceDescStr, self)
        self.surfaceDesc.setOpenExternalLinks(True)
        self.surfaceDesc.setAlignment(Qt.AlignLeft)
        self.surfaceDesc.setStyleSheet('background-color: white; color: black; font-size: 30px; font-weight: light')

        font = QFont()
        font.setPointSize(int(self.width() / 20))
        self.surfaceOutput.setFont(font)
        self.surfaceOutputRaw.setFont(font)
        self.surfaceDesc.setFont(font)
        self.surfaceOutputRange = pg.PlotWidget()
        self.surfaceOutputRange.setBackground((70,72,79))
        self.surfaceOutputRange.showGrid(x=True,y=True,alpha=1)
        self.surfaceOutputRange.getAxis('bottom').setPen('w') 
        self.surfaceOutputRange.getAxis('left').setPen('w') 
        self.surfaceOutputRange.getAxis('right').setStyle(showValues=False) 
        self.surfaceOutputRange.hideAxis('top') 
        self.surfaceOutputRange.hideAxis('right') 
        self.surfaceOutputRange.setXRange(0,100,padding=0.00)
        self.surfaceOutputRange.setYRange(0,1,padding=0.00)
        self.surfaceOutputRange.setMouseEnabled(False,False)
        self.surfaceOutputRangeData = pg.PlotCurveItem(pen=pg.mkPen(width=3, color='b'))
        self.surfaceOutputRange.addItem(self.surfaceOutputRangeData)

        self.surfaceOutputRange.getPlotItem().setLabel('bottom', '<p style="font-size: 20px;color: white">Relative Frame # (0 is most recent)</p>')
        self.surfaceOutputRange.getPlotItem().setLabel('left', '<p style="font-size: 20px;color: white">Grass Probability Value</p>')
        self.surfaceOutputRange.getPlotItem().setLabel('right', ' ')
        self.surfaceOutputRange.getPlotItem().setTitle('<p style="font-size: 30px;color: white">Probability Value over Time</p>')

        self.surfaceOutputRange.getAxis('top').setStyle(tickTextOffset=150)
        self.surfaceOutputRange.setMouseEnabled(False,False)
        self.surfaceOutputRange.mouseMoveEvent = lambda *args, **kwargs: None

        vboxOutput.addWidget(self.surfaceOutput, 0, 0, 1, 1)
        vboxOutput.addWidget(self.surfaceOutputRaw, 1, 0, 1, 1)
        vboxOutput.addWidget(self.surfaceDesc, 0, 1, 2, 1)
        vboxOutput.addWidget(self.surfaceOutputRange, 2, 0, 1, 2)
        #vboxOutput.setVerticalSpacing(0)
        vboxSurface.addLayout(vboxOutput)

        self.surfaceFontSize = '80px' 

        self.surfaceTab.setLayout(vboxSurface)

    def surfaceHandler(self, classification):
        self.surfaceLatestResults.appendleft(classification)
        # Simply take a weighted rolling average of the last 5 frames of data, customers should create a more robust algorithm
        currentClassification = np.average(list(self.surfaceLatestResults)[0:5], weights=[5, 4, 3, 2, 1])
        self.surfaceOutputRangeData.setData(np.arange(0, 100), list(self.surfaceLatestResults))

        # grass > 0.5, not grass <= 0.5
        if (currentClassification > 0.5) :
            self.surfaceOutput.setText("<b>Grass Classification</b><br>" + str(self.surfaceList[1]))
            self.surfaceOutput.setStyleSheet('background-color: green; color: white; font-size: 40px; font-weight: light')
            self.surfaceOutputRaw.setStyleSheet('background-color: green; color: white; font-size: 40px; font-weight: light')
        else :
            self.surfaceOutput.setText("<b>Grass Classification</b><br>" + str(self.surfaceList[0]))
            self.surfaceOutput.setStyleSheet('background-color: #46484f; color: white; font-size: 40px; font-weight: light')
            self.surfaceOutputRaw.setStyleSheet('background-color: #46484f; color: white; font-size: 40px; font-weight: light')
        
        self.surfaceOutputRaw.setText("<b>Grass Classification Value</b><br>" + "{:8.5f}".format(classification * 100) + "%")

    def init1dGraph(self):
        self.rangePlot = pg.PlotWidget()
        self.rangePlot.setBackground('w')
        self.rangePlot.showGrid(x=True,y=True)
        self.rangePlot.setXRange(0,self.chirpComnCfg['NumOfAdcSamples']/2,padding=0.01)
        self.rangePlot.setYRange(0,150,padding=0.01)
        self.rangePlot.setMouseEnabled(False,False)
        self.rangeData = pg.PlotCurveItem(pen=pg.mkPen(width=3, color='r'))
        self.rangePlot.addItem(self.rangeData)
        self.rangePlot.getPlotItem().setLabel('bottom', 'Range (meters)')
        self.rangePlot.getPlotItem().setLabel('left', 'Relative Power (dB)')
        
    def initLevelSensingGraph(self):
        self.levelsensingTab = QWidget()
        
        vboxLevelSense = QVBoxLayout()
        vboxTop = QHBoxLayout()
        vboxBottom = QHBoxLayout()
        
        vboxRangeProfile = QVBoxLayout()
        self.vboxPeakList = QVBoxLayout()
        self.vboxPeakMagnitude = QVBoxLayout()
        self.vboxObjectNo = QVBoxLayout()
        
        comment1 = "Peak No" 
        label_text1 = f"{comment1}"
        self.ObjectNo = QLabel(label_text1, self)
        self.ObjectNo.setAlignment(Qt.AlignCenter)
        self.ObjectNo.setStyleSheet('background-color: teal; color: white; font-size: 30px; font-weight: bold')
        font = QFont()
        font.setPointSize(int(self.width() / 15))
        self.ObjectNo.setFont(font)
        self.vboxObjectNo.addWidget(self.ObjectNo, 1)
        
        comment1 = "1" 
        label_text1 = f"{comment1}"
        self.ObjectNo1 = QLabel(label_text1, self)
        self.ObjectNo1.setAlignment(Qt.AlignCenter)
        self.ObjectNo1.setStyleSheet('background-color: teal; color: white; font-size: 30px; font-weight: bold')
        font = QFont()
        font.setPointSize(int(self.width() / 15))
        self.ObjectNo1.setFont(font)
        self.vboxObjectNo.addWidget(self.ObjectNo1, 1)

        comment2 = "2"
        label_text2 = f"{comment2}"
        self.ObjectNo2 = QLabel(label_text2, self)
        self.ObjectNo2.setAlignment(Qt.AlignCenter)
        self.ObjectNo2.setStyleSheet('background-color: teal; color: white; font-size: 30px; font-weight: bold')
        font = QFont()
        font.setPointSize(int(self.width() / 15))
        self.ObjectNo2.setFont(font)
        self.vboxObjectNo.addWidget(self.ObjectNo2, 1)
        
        comment3 = "3"
        label_text3 = f"{comment3}"
        self.ObjectNo3 = QLabel(label_text3, self)
        self.ObjectNo3.setAlignment(Qt.AlignCenter)
        self.ObjectNo3.setStyleSheet('background-color: teal; color: white; font-size: 30px; font-weight: bold')
        font = QFont()
        font.setPointSize(int(self.width() / 15))
        self.ObjectNo3.setFont(font)
        self.vboxObjectNo.addWidget(self.ObjectNo3, 1)
        
        comment1 = "Distance in meters" 
        label_text1 = f"{comment1}"
        self.PeakListOutput = QLabel(label_text1, self)
        self.PeakListOutput.setAlignment(Qt.AlignCenter)
        self.PeakListOutput.setStyleSheet('background-color: teal; color: white; font-size: 30px; font-weight: bold')
        font = QFont()
        font.setPointSize(int(self.width() / 15))
        self.PeakListOutput.setFont(font)
        self.vboxPeakList.addWidget(self.PeakListOutput, 1)
        
        comment1 = "Object 1 in meters : " 
        label_text1 = f"{self.Peak1}"
        self.PeakListOutput1 = QLabel(label_text1, self)
        self.PeakListOutput1.setAlignment(Qt.AlignCenter)
        self.PeakListOutput1.setStyleSheet('background-color: teal; color: white; font-size: 30px; font-weight: bold')
        font = QFont()
        font.setPointSize(int(self.width() / 15))
        self.PeakListOutput1.setFont(font)
        self.vboxPeakList.addWidget(self.PeakListOutput1, 1)

        comment2 = "Object 2 in meters : "
        label_text2 = f"{self.Peak2}"
        self.PeakListOutput2 = QLabel(label_text2, self)
        self.PeakListOutput2.setAlignment(Qt.AlignCenter)
        self.PeakListOutput2.setStyleSheet('background-color: teal; color: white; font-size: 30px; font-weight: bold')
        font = QFont()
        font.setPointSize(int(self.width() / 15))
        self.PeakListOutput2.setFont(font)
        self.vboxPeakList.addWidget(self.PeakListOutput2, 1)
        
        comment3 = "Object 3 in meters : "
        label_text3 = f"{self.Peak3}"
        self.PeakListOutput3 = QLabel(label_text3, self)
        self.PeakListOutput3.setAlignment(Qt.AlignCenter)
        self.PeakListOutput3.setStyleSheet('background-color: teal; color: white; font-size: 30px; font-weight: bold')
        font = QFont()
        font.setPointSize(int(self.width() / 15))
        self.PeakListOutput3.setFont(font)
        self.vboxPeakList.addWidget(self.PeakListOutput3, 1)
        
        comment1 = "Power in dB" 
        label_text = f"{comment1}"
        self.PeakMagnitudeOutput = QLabel(label_text, self)
        self.PeakMagnitudeOutput.setAlignment(Qt.AlignCenter)
        self.PeakMagnitudeOutput.setStyleSheet('background-color: teal; color: white; font-size: 30px; font-weight: bold')
        font = QFont()
        font.setPointSize(int(self.width() / 15))
        self.PeakMagnitudeOutput.setFont(font)
        self.vboxPeakMagnitude.addWidget(self.PeakMagnitudeOutput, 1)        
                
        comment1 = "Object 1 power in dB: " 
        label_text1 = f"{self.Peak1Magnitude}"
        self.PeakMagnitudeOutput1 = QLabel(label_text1, self)
        self.PeakMagnitudeOutput1.setAlignment(Qt.AlignCenter)
        self.PeakMagnitudeOutput1.setStyleSheet('background-color: teal; color: white; font-size: 30px; font-weight: bold')
        font = QFont()
        font.setPointSize(int(self.width() / 15))
        self.PeakMagnitudeOutput1.setFont(font)
        self.vboxPeakMagnitude.addWidget(self.PeakMagnitudeOutput1, 1)

        comment2 = "Object 2 power in dB: "
        label_text2 = f"{self.Peak2Magnitude}"
        self.PeakMagnitudeOutput2 = QLabel(label_text2, self)
        self.PeakMagnitudeOutput2.setAlignment(Qt.AlignCenter)
        self.PeakMagnitudeOutput2.setStyleSheet('background-color: teal; color: white; font-size: 30px; font-weight: bold')
        font = QFont()
        font.setPointSize(int(self.width() / 15))
        self.PeakMagnitudeOutput2.setFont(font)
        self.vboxPeakMagnitude.addWidget(self.PeakMagnitudeOutput2, 1)
        
        comment3 = "Object 3 power in dB: "
        label_text3 = f"{self.Peak3Magnitude}"
        self.PeakMagnitudeOutput3 = QLabel(label_text3, self)
        self.PeakMagnitudeOutput3.setAlignment(Qt.AlignCenter)
        self.PeakMagnitudeOutput3.setStyleSheet('background-color: teal; color: white; font-size: 30px; font-weight: bold')
        font = QFont()
        font.setPointSize(int(self.width() / 15))
        self.PeakMagnitudeOutput3.setFont(font)
        self.vboxPeakMagnitude.addWidget(self.PeakMagnitudeOutput3, 1)
          
        self.init1dGraph()
        
        self.HighlightPlotPeak1 = pg.ScatterPlotItem(pen=None, size=10, brush=pg.mkBrush('b'))
        self.HighlightPlotPeak2 = pg.ScatterPlotItem(pen=None, size=10, brush=pg.mkBrush('g'))
        self.HighlightPlotPeak3 = pg.ScatterPlotItem(pen=None, size=10, brush=pg.mkBrush('m'))
        self.rangePlot.addItem(self.HighlightPlotPeak1)
        self.rangePlot.addItem(self.HighlightPlotPeak2)
        self.rangePlot.addItem(self.HighlightPlotPeak3)
        
        self.peakLabel1 = pg.TextItem(f'1', anchor=(0.05, 1), color='b')
        self.peakLabel2 = pg.TextItem(f'2', anchor=(0.05, 1), color='g')
        self.peakLabel3 = pg.TextItem(f'3', anchor=(0.05, 1), color='m')
        self.rangePlot.addItem(self.peakLabel1)
        self.rangePlot.addItem(self.peakLabel2)
        self.rangePlot.addItem(self.peakLabel3)
        
        vboxRangeProfile.addWidget(self.rangePlot)
        
        vboxPower = QVBoxLayout()
        llPower = 0
        power_comment = "Power in mW: "
        power_label = f"{power_comment}{llPower}"
        self.PowerOutput = QLabel(power_label, self)
        self.PowerOutput.setAlignment(Qt.AlignCenter)
        self.PowerOutput.setStyleSheet('background-color: teal; color: white; font-size: 25px; font-weight: bold')
        font = QFont()
        font.setPointSize(int(self.width() / 15))
        self.PowerOutput.setFont(font)
        vboxPower.addWidget(self.PowerOutput, 1)
        
        vboxNote = QVBoxLayout()
        noteLable = QLabel("Note : Peaks are ordered based on their relative power. Peak with the highest relative power is designated as Peak 1")
        font = QFont("Arial", 8)
        noteLable.setFont(font)
        vboxNote.addWidget(noteLable)        
     
        vboxGraphics = QVBoxLayout()
        peak_barGraph = pg.BarGraphItem(x = [1, 2, 3], height = [0, 0, 0], width = 0.1, brush = 'g')
       
        self.peakScatterPlot = pg.PlotWidget()
        self.peakScatterPlot.setBackground('w')
        self.peakScatterPlot.showGrid(x=True,y=True)
        self.peakScatterPlot.setXRange(0,1000)
        self.peakScatterPlot.setYRange(0,20,padding=0.01)
        self.peakScatterPlot.setMouseEnabled(False,False)
        self.peakScatterPlot.getPlotItem().setLabel('bottom', 'Frame Number')
        self.peakScatterPlot.getPlotItem().setLabel('left', 'Distance in Meters')
        self.peakScatterPlot.getPlotItem().setLabel('top', 'Peak Movement over time')
        
        vboxGraphics.addWidget(self.peakScatterPlot)
                 
        #vboxTop.addLayout(vboxGraphics)
        vboxTop.addLayout(vboxRangeProfile)
        
        vboxBottom.addLayout(self.vboxObjectNo)
        vboxBottom.addLayout(self.vboxPeakList)
        vboxBottom.addLayout(self.vboxPeakMagnitude)
               
        vboxLevelSense.addLayout(vboxTop)
        vboxLevelSense.addLayout(vboxBottom)
        vboxLevelSense.addLayout(vboxPower)
        vboxLevelSense.addLayout(vboxNote)
        self.levelsensingTab.setLayout(vboxLevelSense)       
       
    def updateGraph(self, outputDict):
        pointCloud = None
        numPoints = 0
        classifierOutput = None
        tracks = None
        trackIndexs = None
        numTracks = 0
        self.frameNum = 0
        error = 0
        occupancyStates = None
        vitalsDict = None
        rangeProfile = None
        self.useFilter = 0
        heights = None
        enhancedPresenceDet = None
        gestureNeuralNetProb = None
        gesture = None
        gesturePresence = None
        gestureFeatures = None
        powerData = None
        velocity = None
        
        # Point Cloud
        if ('pointCloud' in outputDict):
            pointCloud = outputDict['pointCloud']
        # Number of Points
        if ('numDetectedPoints' in outputDict):
            numPoints = outputDict['numDetectedPoints']
        #Velocity
        if ('velocity' in outputDict):
            velocity = outputDict['velocity'][0][0]
        # Tracks
        if ('trackData' in outputDict):
            tracks = outputDict['trackData']

        # Heights
        if ('heightData' in outputDict):
            heights = outputDict['heightData']
    
        # Track index
        if ('trackIndexes' in outputDict):
            trackIndexs = outputDict['trackIndexes']
            
        # Number of Tracks
        if ('numDetectedTracks' in outputDict):
            numTracks = outputDict['numDetectedTracks']

        # Frame number
        if ('frameNum' in outputDict):
            self.frameNum = outputDict['frameNum'] 

        # Error
        if ('error' in outputDict):
            error = outputDict['error']
                    
        # Range Profile
        if ('rangeProfile' in outputDict):
            rangeProfile = outputDict['rangeProfile']
        
        # Range Profile Major
        if ('rangeProfileMajor' in outputDict):
            rangeProfileMajor = outputDict['rangeProfileMajor']
        
        # Range Profile
        if ('rangeProfileMinor' in outputDict):
            rangeProfileMinor = outputDict['rangeProfileMinor']

        # Occupancy State Machine
        if ('occupancy' in outputDict):
            occupancyStates = outputDict['occupancy']

        # Enhanced Presence Detection
        if ('enhancedPresenceDet' in outputDict):
            enhancedPresenceDet = outputDict['enhancedPresenceDet']

        # Vital Signs Info
        if ('vitals' in outputDict):
            vitalsDict = outputDict['vitals']

        # Classifier Info
        if ('classifierOutput' in outputDict):
            classifierOutput = outputDict['classifierOutput']
            
        # Gesture neural network output probabilities
        if ('gestureNeuralNetProb' in outputDict):
            gestureNeuralNetProb = outputDict['gestureNeuralNetProb']

        # Gesture extracted features
        if ('gestureFeatures' in outputDict):
            gestureFeatures = outputDict['gestureFeatures']
        
        # Gesture post-processed classifier output
        if ('gesture' in outputDict):
            gesture = outputDict['gesture']

        # Gesture/presence mode flag
        if ('gesturePresence' in outputDict):
            gesturePresence = outputDict['gesturePresence']

        if ('powerData' in outputDict):
            powerData = outputDict['powerData']

        # Surface classifier output
        if ('surfaceClassificationOutput' in outputDict):
            surfaceClassificationResult = outputDict['surfaceClassificationOutput']
        else:
            surfaceClassificationResult = None
        if (error != 0):
            print ("Parsing Error on frame: %d" % (self.frameNum))
            print ("\tError Number: %d" % (error))
        
        # Update text for display
        self.numPointsDisplay.setText('Points: '+str(numPoints))
        self.numTargetsDisplay.setText('Targets: '+str(numTracks))

        if(velocity is not None):
            self.updateGroundSpeedDisplay("{0:.2f} m/s".format(velocity))
            self.updateGroundSpeedPlot(velocity)
            self.updateGroundSpeedMphDisplay("{0:.2f} mph".format(velocity * 2.237))
        
        # Rotate point cloud and tracks to account for elevation and azimuth tilt
        if (self.profile['elev_tilt'] != 0 or self.profile['az_tilt'] != 0):
            if (pointCloud is not None):
                for i in range(numPoints):
                    rotX, rotY, rotZ = eulerRot (pointCloud[i,0], pointCloud[i,1], pointCloud[i,2], self.profile['elev_tilt'], self.profile['az_tilt'])
                    pointCloud[i,0] = rotX
                    pointCloud[i,1] = rotY
                    pointCloud[i,2] = rotZ
            if (tracks is not None):
                for i in range(numTracks):
                    rotX, rotY, rotZ = eulerRot (tracks[i,1], tracks[i,2], tracks[i,3], self.profile['elev_tilt'], self.profile['az_tilt'])
                    tracks[i,1] = rotX
                    tracks[i,2] = rotY
                    tracks[i,3] = rotZ

        # Shift points to account for sensor height
        if (self.profile['sensorHeight'] != 0):
            if (pointCloud is not None):
                pointCloud[:,2] = pointCloud[:,2] + self.profile['sensorHeight']
            if (tracks is not None):
                tracks[:,3] = tracks[:,3] + self.profile['sensorHeight']

        # Update boundary box colors based on results of Occupancy State Machine
        if (occupancyStates is not None):
            for box in self.boundaryBoxes:
                if ('occZone' in box['name']):
                    # Get index of the occupancy zone from the box name
                    occIdx = int(box['name'].lstrip(string.ascii_letters))
                    # Zone unnoccupied 
                    if (occIdx >= len(occupancyStates) or not occupancyStates[occIdx]):
                        box['color'].setCurrentText('Green')
                    # Zone occupied
                    else:
                        # Make first box turn red
                        if (occIdx == 0):
                            box['color'].setCurrentText('Red')
                        else:
                            box['color'].setCurrentText('Yellow')

        # Update boundary box colors based on results of Occupancy State Machine
        if (enhancedPresenceDet is not None):
            for box in self.boundaryBoxes:
                if ('mpdBox' in box['name']):
                    # Get index of the occupancy zone from the box name
                    boxIdx = int(box['name'].lstrip(string.ascii_letters))
                    # out of bounds
                    if (boxIdx >= len(enhancedPresenceDet)):
                        print("Warning : Occupancy results for box that does not exist")
                    elif (enhancedPresenceDet[boxIdx] == 0):
                        box['color'].setCurrentText('Blue') # Zone unoccupied
                    elif (enhancedPresenceDet[boxIdx] == 1):
                        box['color'].setCurrentText('Yellow') # Minor Motion Zone Occupancy 
                    elif (enhancedPresenceDet[boxIdx] == 2):
                        box['color'].setCurrentText('Red') # Major Motion Zone Occupancy
                    else:
                        print("Error : invalid result for Enhanced Presence Detection TLV")

        # Process gesture info
        if (gestureNeuralNetProb is not None and self.lastFrameProcd != self.frameNum):
            self.lastFrameProcd = self.frameNum
            gesture = self.gesturePostProc(gestureNeuralNetProb)
        elif (gesture is not None and gesture is not GESTURE_NO_GESTURE_6432):
            self.gestureHandler(gesture)
                
        # Process gesture/presence mode info
        # if(gesturePresence is not None):
        #     self.gesturePresenceHandler(gesturePresence)

        # Process gesture features
        if(gestureFeatures is not None):
            self.updateGestureFeatures(gestureFeatures)
            
        if (pointCloud is not None):
            for i in range(numPoints):
              if(i == 0):
                 self.Peak1 = round(pointCloud[i, 1], 3)
                 self.Peak1Magnitude = round(np.log10(round((pointCloud[i, 4]*64 + pointCloud[i, 5]), 4)+1)*20, 1)
                 #print ( f'Peak 1 Magnitude (${round(pointCloud[i, 4], 4)})')
              elif (i == 1):
                 self.Peak2 = round(pointCloud[i, 1], 3)
                 self.Peak2Magnitude = round(np.log10(round((pointCloud[i, 4]*64 + pointCloud[i, 5]), 4)+1)*20, 1)
                 #print ( f'Peak 2 Magnitude (${round(pointCloud[i, 4], 4)})')
              elif (i == 2):
                 self.Peak3 = round(pointCloud[i, 1], 3)
                 self.Peak3Magnitude = round(np.log10(round((pointCloud[i, 4]*64 + pointCloud[i, 5]), 4)+1)*20, 1)
                 #print ( f'Peak 3 Magnitude (${round(pointCloud[i, 4], 4)})')

        if(powerData is not None):
            self.updatePowerNumbers(powerData)

        # Process surface classification info
        if (surfaceClassificationResult is not None):
            self.surfaceHandler(surfaceClassificationResult)

        if(OFF_CHIP_PRESENCE_DETECTION_ENABLED == 1 and self.profile['enabled'] == 1):
            if pointCloud.shape[0] > 0:
                DBSCANObj = DBSCAN(eps=self.profile['maxDistance'], min_samples=self.profile['minPoints']).fit(pointCloud[:, 0:3])
                clusters = set(DBSCANObj.labels_) # Set of labels (no repeats)
                pointLabels = DBSCANObj.labels_ # List of labels by points (with repeated labels)
                centroids = []
                for clusterIdx, cluster in enumerate(clusters): # No repeats in a cluster
                    if(cluster >= 0): # Discard the -1 cluster which is the unassociated points
                        centroids.append({'x':0,'y':0,'z':0,'snr':0, 'numPoints':0, 'r':0, 'theta':0})
                        for labelIdx, label in enumerate(pointLabels):
                            if(label == cluster):
                                centroids[clusterIdx]['x'] = centroids[clusterIdx]['x'] + pointCloud[labelIdx][0] # X
                                centroids[clusterIdx]['y'] = centroids[clusterIdx]['y'] + pointCloud[labelIdx][1] # Y
                                centroids[clusterIdx]['z'] = centroids[clusterIdx]['z'] + pointCloud[labelIdx][2] # Z
                                centroids[clusterIdx]['snr'] = centroids[clusterIdx]['snr'] + pointCloud[labelIdx][4] # SNR
                                centroids[clusterIdx]['numPoints'] = centroids[clusterIdx]['numPoints'] + 1 #store the count to divide later
                        # Compute the centroid of the cluster, store the number of points and snr to pass to the state machine
                        centroids[clusterIdx]['x'] = centroids[clusterIdx]['x'] / centroids[clusterIdx]['numPoints'] 
                        centroids[clusterIdx]['y'] = centroids[clusterIdx]['y'] / centroids[clusterIdx]['numPoints'] 
                        centroids[clusterIdx]['z'] = centroids[clusterIdx]['z'] / centroids[clusterIdx]['numPoints']
                        centroids[clusterIdx]['r'] = math.sqrt(math.pow(centroids[clusterIdx]['x'], 2) + math.pow(centroids[clusterIdx]['y'], 2))
                        if(float(centroids[clusterIdx]['y']) > 0) :
                            centroids[clusterIdx]['theta'] = math.atan(float(centroids[clusterIdx]['x']) / float(centroids[clusterIdx]['y'])) * 180 / np.pi
                        else:
                            centroids[clusterIdx]['theta'] = 0

                    # Pass clusters into state machine
                    for machine in majorMotionStateMachines:
                        machine.step(centroids)
                        state = machine.getState()
                        bbIndex = machine.getBoundaryArcIndex() # Boundary Box index may not equal the state machine index
                        # Update boundary box colors based on results of Motion/Presence Detection
                        if(state == 1):
                            self.boundaryBoxes[bbIndex]['color'].setCurrentText('Red')
                        if(state == 0):
                            self.boundaryBoxes[bbIndex]['color'].setCurrentText('Blue')
        
        # Vital Signs info
        if (vitalsDict is not None):
            # Update info for each patient
            patientId = vitalsDict['id']
            # Check that patient id is valid
            if (patientId < self.profile['maxTracks']):
                self.vitalsPatientData[patientId]['rangeBin'] = vitalsDict['rangeBin']
                self.vitalsPatientData[patientId]['breathDeviation'] = vitalsDict['breathDeviation']
                self.vitalsPatientData[patientId]['breathRate'] = vitalsDict['breathRate']

                # Take the median of the last n heartrates to prevent it from being sporadic
                self.vitalsPatientData[patientId]['heartRate'].append(vitalsDict['heartRate'])
                while (len(self.vitalsPatientData[patientId]['heartRate']) > NUM_HEART_RATES_FOR_MEDIAN):
                    self.vitalsPatientData[patientId]['heartRate'].pop(0)
                medianHeartRate = statistics.median(self.vitalsPatientData[patientId]['heartRate'])
                
                # Check if the patient is holding their breath, and if there is a patient  detected at all
                # TODO ensure vitals output is 0 
                if(float(vitalsDict['breathDeviation']) == 0 or numTracks == 0):
                    patientStatus = 'No Patient Detected'
                    breathRateText = "N/A"
                    heartRateText = "N/A"
                    # Workaround to ensure waveform is flat when no track is present
                    for i in range(NUM_FRAMES_PER_VITALS_PACKET):
                        vitalsDict['heartWaveform'][i] = 0
                        vitalsDict['breathWaveform'][i] = 0
                else:
                    if (medianHeartRate == 0):
                        heartRateText = "Updating"
                    else:
                        heartRateText = str(round(medianHeartRate, 1))
                    # Patient breathing normally
                    if (float(vitalsDict['breathDeviation']) >= 0.02):
                        patientStatus = 'Presence'
                        if(self.vitalsPatientData[patientId]['breathRate'] == 0):
                            breathRateText = "Updating"
                        else:
                            # Round the floats to 1 decimal place and format them for display
                            breathRateText = str(round(self.vitalsPatientData[patientId]['breathRate'], 1))
                     # Patient holding breath
                    else:
                        patientStatus = 'Holding Breath'
                        breathRateText = "N/A"
                 
                if(self.vitalsIWRL6432 == 1):                
                    self.vitalsPatientData[patientId]['heartWaveform'].extend(vitalsDict['heartWaveform'])
                    while (len(self.vitalsPatientData[patientId]['heartWaveform']) > NUM_VITALS_FRAMES_IN_PLOT_IWRL6432):
                        self.vitalsPatientData[patientId]['heartWaveform'].pop(0)

                    # Add breathing rate waveform data for this packet to the graph
                    self.vitalsPatientData[patientId]['breathWaveform'].extend(vitalsDict['breathWaveform'])
                    while (len(self.vitalsPatientData[patientId]['breathWaveform']) > NUM_VITALS_FRAMES_IN_PLOT_IWRL6432):
                        self.vitalsPatientData[patientId]['breathWaveform'].pop(0)
                else: 
                    # Add heart rate waveform data for this packet to the graph
                    self.vitalsPatientData[patientId]['heartWaveform'].extend(vitalsDict['heartWaveform'])
                    while (len(self.vitalsPatientData[patientId]['heartWaveform']) > NUM_VITALS_FRAMES_IN_PLOT):
                        self.vitalsPatientData[patientId]['heartWaveform'].pop(0)

                    # Add breathing rate waveform data for this packet to the graph
                    self.vitalsPatientData[patientId]['breathWaveform'].extend(vitalsDict['breathWaveform'])
                    while (len(self.vitalsPatientData[patientId]['breathWaveform']) > NUM_VITALS_FRAMES_IN_PLOT):
                        self.vitalsPatientData[patientId]['breathWaveform'].pop(0)

                # Copy waveforms so that we can reverse their oritentation
                heartWaveform = self.vitalsPatientData[patientId]['heartWaveform'].copy()
                heartWaveform.reverse()

                # Copy waveforms so that we can reverse their oritentation
                breathWaveform = self.vitalsPatientData[patientId]['breathWaveform'].copy()
                breathWaveform.reverse()

                # Update relevant info in GUI
                self.vitals[patientId]['heartGraph'].setData(heartWaveform)
                self.vitals[patientId]['breathGraph'].setData( breathWaveform)
                self.vitals[patientId]['heartRate'].setText(heartRateText)
                self.vitals[patientId]['breathRate'].setText(breathRateText)
                self.vitals[patientId]['status'].setText(patientStatus)
                self.vitals[patientId]['rangeBin'].setText(str(self.vitalsPatientData[patientId]['rangeBin']))

        # Reset all heights each loop to delete heights from tracks that disappear.
        for cstr in self.coordStr:
            cstr.setVisible(False)

        ## Visualize Target Heights
        # If fall detection is enabled
        if(self.displayFallDet.checkState() == 2):
            # If there are heights to display
            if (heights is not None):
                if (len(heights) != len(tracks)):
                    print ("WARNING: number of heights does not match number of tracks")
                # Compute the fall detection results for each object
                fallDetectionDisplayResults = self.fallDetection.step(heights, tracks)
                ## Display fall detection results

                # For each height heights for current tracks
                for height in heights:
                    # Find track with correct TID
                    for track in tracks:
                        # Found correct track
                        if (int(track[0]) == int(height[0])):
                            tid = int(height[0])
                            height_str = 'tid : ' + str(height[0]) + ', height : ' + str(round(height[1], 2)) + ' m'
                            # If this track was computed to have fallen, display it on the screen
                            if(fallDetectionDisplayResults[tid] > 0): 
                                height_str = height_str + " FALL DETECTED"
                            self.coordStr[tid].setText(height_str)
                            self.coordStr[tid].setX(track[1])
                            self.coordStr[tid].setY(track[2])
                            self.coordStr[tid].setZ(track[3])
                            self.coordStr[tid].setVisible(True)
                            break

        # Point cloud Persistence
        numPersistentFrames = int(self.persistentFramesInput.currentText())
        if (self.configType.currentText() == DEMO_NAME_3DPC or self.configType.currentText() == DEMO_NAME_VITALS):
            numPersistentFrames = numPersistentFrames + 1

        # Add trackIndexs to the point cloud before adding it to the cumulative cloud
        if (trackIndexs is not None):
            # Small Obstacle Detection demo doesnt support track indexes
            if (self.configType.currentText() == DEMO_NAME_SOD or self.configType.currentText() == DEMO_NAME_x432_SOD):
                pass
            # For 3D People Tracking and vitals demos, the tracks and track indexes come one frame after the associated point cloud
            elif (self.configType.currentText() == DEMO_NAME_3DPC or self.configType.currentText() == DEMO_NAME_VITALS):
                if (self.previousClouds[len(self.previousClouds) - 1].shape[0] != trackIndexs.shape[0]):
                    # If there was no data from this frame then don't worry about this check
                    if (self.lastFrameErrorFlag == False):
                        print ("Warning in gui_main.py: number of points in last frame (" + str(self.previousClouds[len(self.previousClouds) - 1].shape[0]) + ") does not match number of track indexes (" + str(trackIndexs.shape[0])+ ")")
                else:
                    self.previousClouds[len(self.previousClouds) - 1][:, 6] = trackIndexs
            else:
                if (pointCloud.shape[0] != trackIndexs.shape[0]):
                    print ("Warning in gui_main.py: number of points does not match number of track indexes")
                else:
                    pointCloud[:, 6] = trackIndexs

        # Reset all heights each loop to delete heights from tracks that disappear.
        for cstr in self.classifierStr:
            cstr.setVisible(False)
        
        # Hold the track IDs detected in the current frame
        trackIDsInCurrFrame = []
        # Add classifier results with filtering to mimic MATLAB results
        if (classifierOutput is not None):
            # Loop through the tracks detected to label them as human/non-human
            for trackNum, trackName in enumerate(tracks):
                # Decode trackID from the trackName
                trackID = int(trackName[0])
                # Hold the track IDs detected in the current frame
                trackIDsInCurrFrame.append(trackID)
                # Track Velocity (radial) = (x * v_x + y*v_y + z*v_z)/ r
                trackVelocity = (trackName[1] * trackName[4] + trackName[2] * trackName[5] + trackName[3] * trackName[6]) \
                / math.sqrt(math.pow(trackName[1], 2) + math.pow(trackName[2], 2) + math.pow(trackName[3], 2))
                
                # Update the tags if ((classification probabilities have been generated by the radar for the current frame) AND 
                # (either the target has not already been detected as a human or the doppler is above the minimum velocity for classification)). 
                # This is designed to stop the tags from being assigned if target has already been detected as a human and becomes stationary.
                if(classifierOutput[trackNum][0] != 0.5 and not(self.wasTargetHuman[trackID] == 1 and abs(trackVelocity)<MIN_CLASSIFICATION_VELOCITY)):
                    # See if either label is above the minimum score needed for classification, it so, add the corresponding tag to the buffer
                    for label in range(NUM_CLASSES_IN_CLASSIFIER):
                        if(classifierOutput[trackNum][label] > CLASSIFIER_CONFIDENCE_SCORE):
                            self.classifierTags[trackID].appendleft(-1 if label == 0 else 1)
                
                
                ## Recompute sum of tags and number of unknown tags
                # Sum the Tags (composed of +1 for one label, -1 for the other label and 0 for unknown) to see which label is dominant
                sumOfTags = sum(self.classifierTags[trackID])
                # Count the number of times there is an unknown tag in the tag buffer
                numUnknownTags = sum(1 for i in self.classifierTags[trackID] if i == 0)

                ## Assign Labels
                # If we don't have enough tags for a decision or the number of tags for human/nonhuman are equal, make no decision 
                if(numUnknownTags > MAX_NUM_UNKNOWN_TAGS_FOR_HUMAN_DETECTION or sumOfTags == 0):
                    self.wasTargetHuman[trackID] = 0 # Target was NOT detected to be human in the current frame, save for next frame
                    self.classifierStr[trackID].setText("Unknown Label")
                # If we have enough tags and the majority of them are for nonhuman, then detect nonhuman
                elif(sumOfTags < 0):
                    self.wasTargetHuman[trackID] = 0 # Target was NOT detected to be human in the current frame, save for next frame
                    self.classifierStr[trackID].setText("Non-Human")
                # If we have enough tags and the majority of them are for human, then detect human
                elif(sumOfTags > 0):
                    self.wasTargetHuman[trackID] = 1 # Target WAS detected to be human in the current frame, save for next frame
                    self.classifierStr[trackID].setText("Human")
                # Populate string that will display a label      
                self.classifierStr[trackID].setX(trackName[1])
                self.classifierStr[trackID].setY(trackName[2])
                self.classifierStr[trackID].setZ(trackName[3] + 0.1) # Add 0.1 so it doesn't interfere with height text if enabled
                self.classifierStr[trackID].setVisible(True) 

            # Regardless of whether you get tracks in the current frame, if there were tracks in the previous frame, reset the
            # tag buffer and wasHumanTarget flag for tracks that aren't detected in the current frame but were detected in the previous frame
            tracksToShuffle = set(self.tracksIDsInPreviousFrame) - set(trackIDsInCurrFrame) 
            for track in tracksToShuffle:
                for frame in range(TAG_HISTORY_LEN):
                    self.classifierTags[track].appendleft(0) # fill the buffer with zeros to remove any history for the track
                self.wasTargetHuman[trackID] = 0 # Since target was not detected in current frame, reset the wasTargetHuman flag

            
            # Put the current tracks detected into the previous track list for the next frame
            self.tracksIDsInPreviousFrame = copy.deepcopy(trackIDsInCurrFrame)

        # Add current point cloud to the cumulative cloud if it's not empty
        if pointCloud is not None:
            self.previousClouds.append(pointCloud)
            self.lastFrameErrorFlag == False
        else:
            self.lastFrameErrorFlag = True

        # If we have more point clouds than needed, stated by numPersistentFrames, delete the oldest ones 
        while(len(self.previousClouds) > numPersistentFrames):
            self.previousClouds.pop(0)
            
        # Since track indexes are delayed a frame on the IWR6843 demo, delay showing the current points by 1 frame
        if ((self.parser.parserType == "DoubleCOMPort") and (self.frameNum > 1 and \
        (self.configType.currentText() == DEMO_NAME_3DPC or self.configType.currentText() == DEMO_NAME_VITALS))):
            cumulativeCloud = np.concatenate(self.previousClouds[:-1])
        elif (len(self.previousClouds) > 0):
            cumulativeCloud = np.concatenate(self.previousClouds)

        # Update 3D Plot
        if (self.graphTabs.currentWidget() == self.pcplot):
            # Update graph, but first ensure the last update completed
            if (self.graphFin):
                self.plotstart = int(round(time.time()*1000))
                self.graphFin = 0
                self.get_thread = updateQTTargetThread3D(cumulativeCloud, tracks, self.scatter, self.pcplot, numTracks, self.ellipsoids, self.coordStr, classifierOutput, self.zRange, self.colorGradient, self.pointColorMode.currentText(), self.plotTracks.isChecked(), self.trackColorMap, self.pointBounds)
                self.get_thread.done.connect(self.graphDone)
                self.get_thread.start(priority=QThread.HighestPriority-1)

        elif (self.graphTabs.currentWidget() == self.rangePlot):
            
            # TODO add logic here to plot major or minor depending on gui monitor input
            if(self.parser.parserType == "DoubleCOMPort"): # Range plot not supported on 6843
                self.rangePlot.getPlotItem().setLabel('top','range profile disabled')
            elif (self.guiMonitor['rangeProfile'] == 0):
                self.rangePlot.getPlotItem().setLabel('top','range profile disabled')
            elif (self.guiMonitor['rangeProfile'] == 1):
                if (rangeProfileMajor is not None):
                    self.plotstart = int(round(time.time()*1000))
                    numRangeBinsParsed = len(rangeProfileMajor)
                    # Check size of rangeData matches expected size
                    if (numRangeBinsParsed == next_power_of_2(round(self.chirpComnCfg['NumOfAdcSamples']/2))):             
                   
                        rangeProfileMajor = np.log10(rangeProfileMajor)*20
                        
                        # Update graph data
                        self.rangeData.setData(self.rangeAxisVals, rangeProfileMajor)
                    else:
                        print (f'Error: Size of rangeProfileMajor (${numRangeBinsParsed}) did not match the expected size (${next_power_of_2(round(self.chirpComnCfg["NumOfAdcSamples"]/2))})')
            elif (self.guiMonitor['rangeProfile'] == 2):
                if (rangeProfileMinor is not None):
                    self.plotstart = int(round(time.time()*1000))
                    numRangeBinsParsed = len(rangeProfileMinor)
                    # Check size of rangeData matches expected size
                    if (numRangeBinsParsed == next_power_of_2(round(self.chirpComnCfg['NumOfAdcSamples']/2))):
                        rangeProfileMinor = np.log10(rangeProfileMinor)*20
                        
                        # Update graph data
                        self.rangeData.setData(self.rangeAxisVals, rangeProfileMinor)
                    else:
                        print (f'Error: Size of rangeProfileMinor (${numRangeBinsParsed}) did not match the expected size (${next_power_of_2(round(self.chirpComnCfg["NumOfAdcSamples"]/2))})')
            elif (self.guiMonitor['rangeProfile'] == 3):
                self.rangePlot.getPlotItem().setLabel('middle','Major & Minor Range Profile Mode Not Supported')
            else:
                self.rangePlot.getPlotItem().setLabel('middle','INVALID gui monitor range profile input')
                            
            self.graphDone()
            
        elif (hasattr(self, 'levelsensingTab') and self.graphTabs.currentWidget() == self.levelsensingTab):
            
            # TODO add logic here to plot major or minor depending on gui monitor input
            if(self.parser.parserType == "DoubleCOMPort"): # Range plot not supported on 6843
                self.rangePlot.getPlotItem().setLabel('top','range profile disabled')
            elif (self.guiMonitor['rangeProfile'] == 0):
                self.rangePlot.getPlotItem().setLabel('top','range profile disabled')
                self.plotstart = int(round(time.time()*1000))
            elif (self.guiMonitor['rangeProfile'] == 1):
                if (rangeProfileMajor is not None):
                    self.plotstart = int(round(time.time()*1000))
                    numRangeBinsParsed = len(rangeProfileMajor)
                    # Check size of rangeData matches expected size
                    if (numRangeBinsParsed == next_power_of_2(round(self.chirpComnCfg['NumOfAdcSamples']/2))):
                    
                        for i in range(len(rangeProfileMajor)):
                            rangeProfileMajor[i] += 1
                        
                        rangeProfileMajor = np.log10(rangeProfileMajor)*20
                        
                        # Update graph data
                        self.rangeData.setData(self.rangeAxisVals, rangeProfileMajor)
                        
                                    # Highlighting specific points
                        for i in range(len(self.rangeAxisVals)):
                            if (self.Peak1 >= self.rangeAxisVals[i] and self.Peak1 < self.rangeAxisVals[i+1]):
                                highlight_peak1 = i  
                            if (self.Peak2 >= self.rangeAxisVals[i] and self.Peak2 < self.rangeAxisVals[i+1]):
                                highlight_peak2 = i   
                            if (self.Peak3 >= self.rangeAxisVals[i] and self.Peak3 < self.rangeAxisVals[i+1]):
                                highlight_peak3 = i                      
            
                        #self.HighlightPlot.clear()
                        highlight_indices = [highlight_peak1]
                        highlight_x = [self.rangeAxisVals[i] for i in highlight_indices]
                        highlight_y = [rangeProfileMajor[i] for i in highlight_indices]
                        data = [{'pos': (x_val, y_val)} for x_val, y_val in zip(highlight_x, highlight_y)]
                        self.HighlightPlotPeak1.setData(data)
            
                        # Adding labels to highlighted points
                        for i in range(len(highlight_indices)):
                             self.peakLabel1.setPos(highlight_x[i], highlight_y[i])
            
         
                        highlight_indices = [highlight_peak2]
                        highlight_x = [self.rangeAxisVals[i] for i in highlight_indices]
                        highlight_y = [rangeProfileMajor[i] for i in highlight_indices]
                        data = [{'pos': (x_val, y_val)} for x_val, y_val in zip(highlight_x, highlight_y)]
                        self.HighlightPlotPeak2.setData(data)
            
                        for i in range(len(highlight_indices)):
                             self.peakLabel2.setPos(highlight_x[i], highlight_y[i])
            
                        highlight_indices = [highlight_peak3]
                        highlight_x = [self.rangeAxisVals[i] for i in highlight_indices]
                        highlight_y = [rangeProfileMajor[i] for i in highlight_indices]
                        data = [{'pos': (x_val, y_val)} for x_val, y_val in zip(highlight_x, highlight_y)]
                        self.HighlightPlotPeak3.setData(data)
            
                        for i in range(len(highlight_indices)):
                             self.peakLabel3.setPos(highlight_x[i], highlight_y[i])
                   
                       
                    else:
                        print (f'Error: Size of rangeProfileMajor (${numRangeBinsParsed}) did not match the expected size (${next_power_of_2(round(self.chirpComnCfg["NumOfAdcSamples"]/2))})')
            else:
                self.rangePlot.getPlotItem().setLabel('middle','INVALID gui monitor range profile input')
            
            self.updateLevelSensingPeaks()
        
            if(powerData is not None):
               self.updateLevelSensingPower(powerData)
            
            self.graphDone()
        elif (hasattr(self, 'gestureTab') and self.graphTabs.currentWidget() == self.gestureTab):
            self.plotstart = int(round(time.time()*1000))
            self.graphDone()
        elif (hasattr(self, 'surfaceTab') and self.graphTabs.currentWidget() == self.surfaceTab):
            self.plotstart = int(round(time.time()*1000))
            self.graphDone()
        else: 
            print (f'Warning: Invalid Widget Selected: ${self.graphTabs.currentWidget()}')

    def graphDone(self):
        plotend = int(round(time.time()*1000))
        plotime = plotend - self.plotstart
        try:
            if (self.frameNum > 1):
                self.averagePlot = (plotime*1/self.frameNum) + (self.averagePlot*(self.frameNum-1)/(self.frameNum))
            else:
                self.averagePlot = plotime
        except:
            self.averagePlot = plotime
        self.graphFin = 1
        pltstr = 'Average Plot time: '+str(plotime)[:5] + ' ms'
        fnstr = 'Frame: '+str(self.frameNum)
        self.frameNumDisplay.setText(fnstr)
        self.plotTimeDisplay.setText(pltstr)


    def resetFallText(self):
        self.fallAlert.setText('Standing')
        self.fallPic.setPixmap(self.standingPicture)
        self.fallResetTimerOn = 0


    def updateFallThresh(self):
        try:
            newThresh = float(self.fallThreshInput.text())
            self.fallThresh = newThresh
            self.fallThreshMarker.setPos(self.fallThresh)
        except:
            print('No numberical threshold')


    def connectCom(self):
        self.parser.frameTime = self.frameTime
        print('Parser type: ',self.configType.currentText())
        # init threads and timers
        self.uart_thread = parseUartThread(self.parser)
        if (self.configType.currentText() != 'Replay'):
            self.uart_thread.fin.connect(self.parseData)
        self.uart_thread.fin.connect(self.updateGraph)
        self.parseTimer = QTimer()
        self.parseTimer.setSingleShot(False)
        self.parseTimer.timeout.connect(self.parseData)        
        try:
            if (os.name == 'nt'):
                uart = "COM"+ self.cliCom.text()
                data = "COM"+ self.dataCom.text()
            else:
            	uart = self.cliCom.text()
            	data = self.dataCom.text()
            if(self.deviceType.currentText() in DEVICE_LIST[0:2]): # If using x843 device
                self.parser.connectComPorts(uart, data)
            else: # If not x843 device then defer to x432 device
                if (self.configType.currentText() == DEMO_NAME_x432_GESTURE):
                    self.parser.connectComPort(uart, 1250000)
                else:
                    self.parser.connectComPort(uart)
            self.connectStatus.setText('Connected')
        #TODO: create the disconnect button action
        except Exception as e:
            print (e)
            self.connectStatus.setText('Unable to Connect')
        if (self.configType.currentText() == "Replay"):
            self.connectStatus.setText('Replay')
        if (self.configType.currentText() == DEMO_NAME_LRPD):
            self.frameTime = 400

    def updateNumTracksBuffer(self):
        # Classifier Data
        # Use a deque here because the append operation adds items to the back and pops the front
        self.classifierTags = [deque([0] * TAG_HISTORY_LEN, maxlen = TAG_HISTORY_LEN) for i in range(self.profile['maxTracks'])]
        self.tracksIDsInPreviousFrame = []
        self.wasTargetHuman = [0 for i in range(self.profile['maxTracks'])]
        if(self.configType.currentText() == DEMO_NAME_3DPC):
            self.fallDetection = FallDetection(self.profile['maxTracks'])
            
    # Select and parse the configuration file
    # Use the most recently used cfg file path as the default option
    def selectCfg(self):
        try:
            file = self.selectFile()
            cachedData.setCachedCfgPath(file) # cache the file and demo used
            self.parseCfg(file)
            if 'maxTracks' in self.profile:
                self.updateNumTracksBuffer() # Update the max number of tracks based off the config file
        except Exception as e:
            print(e)
            print('No cfg file selected!')
    
    def selectFile(self):
        try:
            current_dir = os.getcwd()
            configDirectory = current_dir
            path = cachedData.getCachedCfgPath()
            if (path != ""):
                configDirectory = path
        except:
            configDirectory = ''
        
        fd = QFileDialog()
        filt = "cfg(*.cfg)"
        filename = fd.getOpenFileName(directory=configDirectory,filter=filt)
        return filename[0]


    # Add a boundary box to the boundary boxes tab
    def addBoundBox(self, name, minX=0, maxX=0, minY=0, maxY=0, minZ=0, maxZ=0):
        newBox = self.setBoxControlLayout(name)
        self.boundaryBoxes.append(newBox)
        self.boundaryBoxViz.append(gl.GLLinePlotItem())
        boxIndex = len(self.boundaryBoxes) - 1
        self.boxTab.addTab(newBox['boxCon'], name)
        self.boundaryBoxes[boxIndex]['boundList'][0].setText(str(minX))
        self.boundaryBoxes[boxIndex]['boundList'][1].setText(str(maxX))
        self.boundaryBoxes[boxIndex]['boundList'][2].setText(str(minY))
        self.boundaryBoxes[boxIndex]['boundList'][3].setText(str(maxY))
        self.boundaryBoxes[boxIndex]['boundList'][4].setText(str(minZ))
        self.boundaryBoxes[boxIndex]['boundList'][5].setText(str(maxZ))            
        
        # Specific functionality for various types of boxes
        # Point boundary box
        if ('pointBounds' in name):
            desc = 'Remove points outside of this zone\nDefaults to last boundaryBox in .cfg'
            self.boundaryBoxes[boxIndex]['description'].setText(desc)
            self.boundaryBoxes[boxIndex]['checkEnable'].setDisabled(False)
        # Zone occupancy box
        elif ('occZone' in name):
            desc = 'Checks occupancy status on these zones'
            self.boundaryBoxes[boxIndex]['description'].setText(desc)
            self.boundaryBoxes[boxIndex]['checkEnable'].setChecked(True)
            self.boundaryBoxes[boxIndex]['color'].setCurrentText('Green')
            # Lock each text field
            for textBox in self.boundaryBoxes[boxIndex]['boundList']:
                textBox.setDisabled(True)
            # Lock enable box
            self.boundaryBoxes[boxIndex]['checkEnable'].setDisabled(True)
            self.boundaryBoxes[boxIndex]['color'].setDisabled(True)
        elif ('trackerBounds' in name):
            desc = 'Checks for tracks in this zone'
            self.boundaryBoxes[boxIndex]['description'].setText(desc)
            self.boundaryBoxes[boxIndex]['checkEnable'].setChecked(True)
            # Lock each text field
            for textBox in self.boundaryBoxes[boxIndex]['boundList']:
                textBox.setDisabled(True)
            # Lock enable box
            self.boundaryBoxes[boxIndex]['checkEnable'].setDisabled(True)
        elif ('mpdBox' in name):
            desc = 'checks for motion or presence in the box'
            self.boundaryBoxes[boxIndex]['description'].setText(desc)
            self.boundaryBoxes[boxIndex]['checkEnable'].setChecked(True)
            self.boundaryBoxes[boxIndex]['color'].setCurrentText('Blue')
            # Lock each text field
            for textBox in self.boundaryBoxes[boxIndex]['boundList']:
                textBox.setDisabled(True)
            # Lock enable box
            self.boundaryBoxes[boxIndex]['checkEnable'].setDisabled(True)
            self.boundaryBoxes[boxIndex]['color'].setDisabled(True)        
        # Set visible if enabled
        if (self.boundaryBoxes[boxIndex]['checkEnable'].isChecked()):
            self.boundaryBoxViz[boxIndex].setVisible(True)
        else:
            self.boundaryBoxViz[boxIndex].setVisible(False)
        self.pcplot.addItem(self.boundaryBoxViz[boxIndex])
        self.onChangeBoundaryBox()
        

    def parseCfg(self, fname):
        with open(fname, 'r') as cfg_file:
            self.cfg = cfg_file.readlines()
        counter = 0
        chirpCount = 0
        for line in self.cfg:
            args = line.split()
            if (len(args) > 0):
                # cfarCfg
                if (args[0] == 'cfarCfg'):
                    pass
                    #self.cfarConfig = {args[10], args[11], '1'}
                # trackingCfg
                elif (args[0] == 'trackingCfg'):
                    if (len(args) < 5):
                        print ("Error: trackingCfg had fewer arguments than expected")
                        continue
                    self.profile['maxTracks'] = int(args[4])
                    # Update the maximum number of tracks based off the cfg file
                    self.trackColorMap = get_trackColors(self.profile['maxTracks'])
                    for m in range(self.profile['maxTracks']):
                        # Add track gui object
                        mesh = gl.GLLinePlotItem()
                        mesh.setVisible(False)
                        self.pcplot.addItem(mesh)
                        self.ellipsoids.append(mesh)
                        # Add track coordinate string
                        text = GLTextItem()
                        text.setGLViewWidget(self.pcplot)
                        text.setVisible(False)
                        self.pcplot.addItem(text)
                        self.coordStr.append(text)
                        # Add track classifier label string
                        classifierText = GLTextItem()
                        classifierText.setGLViewWidget(self.pcplot)
                        classifierText.setVisible(False)
                        self.pcplot.addItem(classifierText)
                        self.classifierStr.append(classifierText)
                    # If we only support 1 patient, hide the other patient window
                    if (self.profile['maxTracks'] == 1):
                        self.vitals[1]['pane'].setVisible(False)
                    # Initialize Vitals output dictionaries for each potential patient
                    for i in range (min(self.profile['maxTracks'], MAX_VITALS_PATIENTS)):
                        # Initialize 
                        patientDict = {}
                        patientDict ['id'] = i
                        patientDict ['rangeBin'] = 0
                        patientDict ['breathDeviation'] = 0
                        patientDict ['heartRate'] = []
                        patientDict ['breathRate'] = 0
                        patientDict ['heartWaveform'] = []
                        patientDict ['breathWaveform'] = []
                        self.vitalsPatientData.append(patientDict)

                        # Make each patient's pane visible
                        self.vitals[i]['pane'].setVisible(True)

                elif (args[0] == 'AllocationParam'):
                    pass
                    #self.allocConfig = tuple(args[1:6])
                elif (args[0] == 'GatingParam'):
                    pass
                    #self.gatingConfig = tuple(args[1:4])
                elif (args[0] == 'SceneryParam' or args[0] == 'boundaryBox'):
                    if (len(args) < 7):
                        print ("Error: SceneryParam/boundaryBox had fewer arguments than expected")
                        continue
                    self.boundaryLine = counter
                    leftX = float(args[1])
                    rightX = float(args[2])
                    nearY = float(args[3])
                    farY = float(args[4])
                    bottomZ = float(args[5])
                    topZ = float(args[6])
                    self.addBoundBox('trackerBounds', leftX, rightX, nearY, farY, bottomZ, topZ)
                    # Default pointBounds box to have the same values as the last boundaryBox in the config
                    # These can be changed by the user
                    self.boundaryBoxes[0]['boundList'][0].setText(str(leftX))
                    self.boundaryBoxes[0]['boundList'][1].setText(str(rightX))
                    self.boundaryBoxes[0]['boundList'][2].setText(str(nearY))
                    self.boundaryBoxes[0]['boundList'][3].setText(str(farY))
                    self.boundaryBoxes[0]['boundList'][4].setText(str(bottomZ))
                    self.boundaryBoxes[0]['boundList'][5].setText(str(topZ))
                elif (args[0] == 'staticBoundaryBox'):
                    self.staticLine = counter
                elif (args[0] == 'profileCfg'):
                    if (len(args) < 12):
                        print ("Error: profileCfg had fewer arguments than expected")
                        continue
                    self.profile['startFreq'] = float(args[2])
                    self.profile['idle'] = float(args[3])
                    self.profile['adcStart'] = float(args[4])
                    self.profile['rampEnd'] = float(args[5])
                    self.profile['slope'] = float(args[8])
                    self.profile['samples'] = float(args[10])
                    self.profile['sampleRate'] = float(args[11])
                    print(self.profile)
                elif (args[0] == 'frameCfg'):
                    if (len(args) < 4):
                        print ("Error: frameCfg had fewer arguments than expected")
                        continue
                    self.frameTime = float(args[5])
                    self.profile['numLoops'] = float(args[3])
                    self.profile['numTx'] = float(args[2])+1
                elif (args[0] == 'chirpCfg'):
                    chirpCount += 1
                elif (args[0] == 'sensorPosition'):
                    # sensorPosition for x843 family has 3 args
                    if(self.deviceType.currentText() in DEVICE_LIST[0:2]):
                        if (len(args) < 4):
                            print ("Error: sensorPosition had fewer arguments than expected")
                            continue
                        self.profile['sensorHeight'] = float(args[1])
                        self.profile['az_tilt'] = float(args[2])
                        self.profile['elev_tilt'] = float(args[3])

                    # sensorPosition for x432 family has 5 args
                    if (self.deviceType.currentText() in DEVICE_LIST[2] or self.deviceType.currentText() in DEVICE_LIST[3]):
                        if (len(args) < 6):
                            print ("Error: sensorPosition had fewer arguments than expected")
                            continue
                        #xOffset and yOffset are not implemented in the python code yet.
                        self.profile['xOffset'] = float(args[1])
                        self.profile['yOffset'] = float(args[2])
                        self.profile['sensorHeight'] = float(args[3])
                        self.profile['az_tilt'] = float(args[4])
                        self.profile['elev_tilt'] = float(args[5])
                # Only used for Small Obstacle Detection
                elif (args[0] == 'occStateMach'):
                    numZones = int(args[1])
                # Only used for Small Obstacle Detection
                elif (args[0] == 'zoneDef'):
                    if (len(args) < 8):
                        print ("Error: zoneDef had fewer arguments than expected")
                        continue
                    zoneIdx = int(args[1])
                    minX = float(args[2])
                    maxX = float(args[3])
                    minY = float(args[4])
                    maxY = float(args[5])
                    # Offset by 3 so it is in center of screen
                    minZ = float(args[6]) + self.profile['sensorHeight']
                    maxZ = float(args[7]) + self.profile['sensorHeight']

                    name = 'occZone' + str(zoneIdx)

                    self.addBoundBox(name, minX, maxX, minY, maxY, minZ, maxZ)
                elif (args[0] == 'mpdBoundaryBox'):
                    if (len(args) < 8):
                        print ("Error: mpdBoundaryBox had fewer arguments than expected")
                        continue

                    if self.mpdZoneType is None:
                        self.mpdZoneType = 'mpdBoundaryBox'
                    elif (self.mpdZoneType != 'mpdBoundaryBox'):
                        print("Error: Cannot add mpdBoundaryBox when one or more %s is already defined." % self.mpdZoneType)
                        continue
                    else:
                        print("mpdZoneType: %s" % self.mpdZoneType)

                    zoneIdx = int(args[1])
                    minX = float(args[2])
                    maxX = float(args[3])
                    minY = float(args[4])
                    maxY = float(args[5])
                    minZ = float(args[6])
                    maxZ = float(args[7])
                    name = 'mpdBox' + str(zoneIdx)
                    self.addBoundBox(name, minX, maxX, minY, maxY, minZ, maxZ)

                elif(args[0] == 'mpdBoundaryArc'):
                    if (len(args) < 8):
                        print ("Error: mpdBoundaryArc had fewer arguments than expected")
                        continue

                    if self.mpdZoneType is None:
                        self.mpdZoneType = 'mpdBoundaryArc'
                    elif (self.mpdZoneType != 'mpdBoundaryArc'):
                        print("Error: Cannot add mpdBoundaryArc when one or more %s is already defined." % self.mpdZoneType)
                        continue
                    else:
                        print("mpdZoneType: %s" % self.mpdZoneType)

                    zoneIdx = int(args[1])
                    minR = float(args[2])
                    maxR = float(args[3])
                    minTheta = float(args[4])
                    maxTheta = float(args[5])
                    minZ = float(args[6])
                    maxZ = float(args[7])
                    name = 'mpdArc' + str(zoneIdx)
                    if(OFF_CHIP_PRESENCE_DETECTION_ENABLED == 1):
                        majorMotionStateMachines.append(majorBoundaryArcStateMachineType())
                    self.addBoundBox(name, minR, maxR, minTheta, maxTheta, minZ, maxZ)
   
                elif (args[0] == 'chirpComnCfg'):
                    if (len(args) < 8):
                        print ("Error: chirpComnCfg had fewer arguments than expected")
                        continue
                    try:
                        self.chirpComnCfg['DigOutputSampRate'] = int(args[1])
                        self.chirpComnCfg['DigOutputBitsSel'] = int(args[2])
                        self.chirpComnCfg['DfeFirSel'] = int(args[3])
                        self.chirpComnCfg['NumOfAdcSamples'] = int(args[4])
                        self.chirpComnCfg['ChirpTxMimoPatSel'] = int(args[5])
                        self.chirpComnCfg['ChirpRampEndTime'] = 10 * float(args[6])
                        self.chirpComnCfg['ChirpRxHpfSel'] = int(args[7])
                    except Exception as e:
                        print (e)

                elif (args[0] == 'chirpTimingCfg'):
                    if (len(args) < 6):
                        print ("Error: chirpTimingCfg had fewer arguments than expected")
                        continue
                        
                    self.chirpTimingCfg['ChirpIdleTime'] = 10.0 * float(args[1])
                    self.chirpTimingCfg['ChirpAdcSkipSamples'] = int(args[2]) << 10
                    self.chirpTimingCfg['ChirpTxStartTime'] = 10.0 * float(args[3])
                    self.chirpTimingCfg['ChirpRfFreqSlope'] = float(args[4])
                    self.chirpTimingCfg['ChirpRfFreqStart'] = float(args[5])
                elif (args[0] == 'clusterCfg'):
                    if (len(args) < 4):
                        print ("Error: clusterCfg had fewer arguments than expected")
                        continue
                    self.profile['enabled'] = float(args[1])
                    self.profile['maxDistance'] = float(args[2])
                    self.profile['minPoints'] = float(args[3])
                                    
                # NOTE - Only major mode is supported
                elif (args[0] == 'majorStateCfg'):
                    if (len(args) < 9):
                        print ("Error: majorStateCfg had fewer arguments than expected")
                        continue
                    pointThre1 = int(args[1])
                    pointThre2 = int(args[2])
                    snrThre2 = int(args[3])
                    pointHistThre1 = int(args[4])
                    pointHistThre2 = int(args[5])
                    snrHistThre2 = int(args[6])
                    histBufferSize = int(args[7])
                    minor2emptyThre = int(args[8])

                    stateMachineIdx = 0
                    boundaryBoxIdx = 0
                    if(OFF_CHIP_PRESENCE_DETECTION_ENABLED == 1):
                        for box in self.boundaryBoxes:
                            if('mpdArc' in box['name']):
                                majorMotionStateMachines[stateMachineIdx].configure(pointThre1, pointThre2, snrThre2, \
                                pointHistThre1, pointHistThre2, snrHistThre2,  histBufferSize, minor2emptyThre,\
                                float(box['boundList'][0].text()),float(box['boundList'][1].text()),\
                                float(box['boundList'][2].text()),float(box['boundList'][3].text()),\
                                float(box['boundList'][4].text()),float(box['boundList'][5].text()), boundaryBoxIdx)
                                stateMachineIdx=stateMachineIdx+1
                            boundaryBoxIdx = boundaryBoxIdx + 1
    
                # This is specifically guiMonitor for 60Lo, this parsing will break the gui when an SDK 3 config is sent
                elif (args[0] == 'guiMonitor'):
                    if(self.deviceType.currentText() in DEVICE_LIST[2] or self.deviceType.currentText() in DEVICE_LIST[3]):
                        if (len(args) < 12):
                            print ("Error: guiMonitor had fewer arguments than expected")
                            continue
                    self.guiMonitor['pointCloud'] = int(args[1])
                    self.guiMonitor['rangeProfile'] = int(args[2])
                    self.guiMonitor['NoiseProfile'] = int(args[3])
                    self.guiMonitor['rangeAzimuthHeatMap'] = int(args[4])
                    self.guiMonitor['rangeDopplerHeatMap'] = int(args[5])
                    self.guiMonitor['statsInfo'] = int(args[6])

                elif(args[0] == 'measureRangeBiasAndRxChanPhase'):
                    self.measureRangeBiasAndRxChanPhase['enabled'] = 1
                    self.measureRangeBiasAndRxChanPhase['centerDist'] = float(args[2])
                    self.measureRangeBiasAndRxChanPhase['searchRange'] = float(args[3])

                elif(args[0] == 'clutterRemoval'):
                    self.clutterRemoval = int(args[1])

                elif(args[0] == 'sigProcChainCfg'):
                    # Major Motion is if the motion mode is equal to 1 or 3
                    self.sigProcChain['majorMotionEnabled'] = int(args[3]) % 2
                    # Minor Motion is if the motion mode is 2 or 3
                    self.sigProcChain['minorMotionEnabled'] = 1 if int(args[3]) > 1 else 0
                    
                elif(args[0] == 'channelCfg'):
                    self.channelCfg['RX'] = int(args[1])
                    self.channelCfg['TX'] = int(args[2])
            counter += 1

        # If measureRangeBiasAndRxChanPhase is set, make sure that the constraints are met
        if (self.measureRangeBiasAndRxChanPhase['enabled'] == 1):
            # Only run the measurement if BPM is disabled, clutter removal is off, major motion is on and the number of TX/RX antennas is 2/3
            if (self.chirpComnCfg['ChirpTxMimoPatSel'] == 4):
                print("Error : measureRangeBiasAndRxChanPhase requires TDM mode not BPM Mode. \n Change the 5th argument of chirpComnCfg to 1")
            if (self.clutterRemoval == 1):
                print("Error : measureRangeBiasAndRxChanPhase requires Clutter Removal Off. \n Change the first argument clutterRemoval to 0")
            if (self.sigProcChain['majorMotionEnabled'] == 0):
                print("Error : measureRangeBiasAndRxChanPhase requires Major Motion Enabled. \n Change the 3th argument of sigProcChainCfg to 1 or 3")
            if (self.channelCfg['TX'] != 3):
                print("Error : measureRangeBiasAndRxChanPhase requires 2 TX enabled. \n Change the 2nd argument of channelCfg to 3")
            if (self.channelCfg['RX'] != 7):
                print("Error : measureRangeBiasAndRxChanPhase requires 3 RX enabled. \n Change the 1st argument of channelCfg to 7")
            
            # Create range bin zone in the gui
            rangeMin = self.measureRangeBiasAndRxChanPhase['centerDist'] - self.measureRangeBiasAndRxChanPhase['searchRange']/2
            rangeMax = self.measureRangeBiasAndRxChanPhase['centerDist'] + self.measureRangeBiasAndRxChanPhase['searchRange']/2
            self.compRangeBiasZone = pg.LinearRegionItem((rangeMin, rangeMax), movable=False,span=(0, 0.94))
            self.rangePlot.addItem(self.compRangeBiasZone)
            text = pg.TextItem(text='Place Peak in Calibration Zone', color=(200,0,0), anchor=(0.5, 0))
            text.setPos(self.measureRangeBiasAndRxChanPhase['centerDist'],145)
            self.rangePlot.addItem(text)
            text.setVisible(True)

        # self.rangeRes = (3e8*(100/self.chirpComnCfg['DigOutputSampRate']))/(2*self.chirpTimingCfg['ChirpRfFreqSlope']*self.chirpComnCfg['NumOfAdcSamples'])
        self.rangeRes = (3e8*(100/self.chirpComnCfg['DigOutputSampRate'])*1e6)/(2*self.chirpTimingCfg['ChirpRfFreqSlope']*1e12*self.chirpComnCfg['NumOfAdcSamples'])
        self.rangePlot.setXRange(0,(self.chirpComnCfg['NumOfAdcSamples']/2)*self.rangeRes,padding=0.01)
        
        self.rangeAxisVals = np.arange(0, self.chirpComnCfg['NumOfAdcSamples']/2*self.rangeRes, self.rangeRes)
        print(self.guiMonitor['rangeProfile'])

        if (self.guiMonitor['rangeProfile'] == 0):
            self.rangePlot.getPlotItem().setLabel('top','range profile disabled')
        elif (self.guiMonitor['rangeProfile'] == 1):
            self.rangePlot.getPlotItem().setLabel('top','Major Range Profile')
        elif (self.guiMonitor['rangeProfile'] == 2):
            self.rangePlot.getPlotItem().setLabel('top','Minor Range Profile')
        elif (self.guiMonitor['rangeProfile'] == 3):
            self.rangePlot.getPlotItem().setLabel('top','Major & Minor Range Profile Mode Not Supported')
        else:
            self.rangePlot.getPlotItem().setLabel('top','INVALID gui monitor range profile input')
        

        # self.profile['maxRange'] = self.profile['sampleRate']*1e3*0.9*3e8/(2*self.profile['slope']*1e12)
        # bw = self.profile['samples']/(self.profile['sampleRate']*1e3)*self.profile['slope']*1e12
        # rangeRes = 3e8/(2*bw)
        # Tc = (self.profile['idle']*1e-6 + self.profile['rampEnd']*1e-6)*chirpCount
        # lda = 3e8/(self.profile['startFreq']*1e9)
        # maxVelocity = lda/(4*Tc)
        # velocityRes = lda/(2*Tc*self.profile['numLoops']*self.profile['numTx'])
        # self.configTable.setItem(1,1,QTableWidgetItem(str(self.profile['maxRange'])[:5]))
        # self.configTable.setItem(2,1,QTableWidgetItem(str(rangeRes)[:5]))
        # self.configTable.setItem(3,1,QTableWidgetItem(str(maxVelocity)[:5]))
        # self.configTable.setItem(4,1,QTableWidgetItem(str(velocityRes)[:5]))
   
        # Update sensor position
        self.az_tilt.setText(str(self.profile['az_tilt']))
        self.elev_tilt.setText(str(self.profile['elev_tilt']))
        self.s_height.setText(str(self.profile['sensorHeight']))
        self.onChangeSensorPosition()

    def sendCfg(self):
        try:
            self.saveBinaryBox.setDisabled(True)
            if (self.configType.currentText() != "Replay"):
                self.parser.sendCfg(self.cfg)
                self.configSent = 1
                self.parseTimer.start(int(self.frameTime)) # need this line 
                
        except Exception as e:
            print(e)
            print ('No cfg file selected!')

    def startApp(self):
        self.configSent = 1
        self.parseTimer.start(int(self.frameTime)) # need this line 

    def parseData(self):
        self.uart_thread.start(priority=QThread.HighestPriority)

    def whoVisible(self):
        if (self.threeD):
            self.threeD = 0
        else:
            self.threeD = 1

if __name__ == '__main__':
    if (compileGui):
        appctxt = ApplicationContext()
        app = QApplication(sys.argv)
        screen = app.primaryScreen()
        size = screen.size()
        main = Window(size=size)
        main.show()
        exit_code = appctxt.app.exec_()
        sys.exit(exit_code)
    else:
        QApplication.setAttribute(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
        app = QApplication(sys.argv)
        screen = app.primaryScreen()
        size = screen.size()
        main = Window(size=size)
        main.show()
        sys.exit(app.exec_())
