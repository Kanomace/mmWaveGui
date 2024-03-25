import struct
import sys
import serial
import binascii
import time
import numpy as np
import math

import os
import datetime

# Local File Imports
from parseTLVs import *
from gui_common import *

def parseStandardFrame(frameData):
    # Constants for parsing frame header
    headerStruct = 'Q8I'
    frameHeaderLen = struct.calcsize(headerStruct)
    tlvHeaderLength = 8

    # Define the function's output structure and initialize error field to no error
    outputDict = {}
    outputDict['error'] = 0

    # A sum to track the frame packet length for verification for transmission integrity 
    totalLenCheck = 0   

    # Read in frame Header
    try:
        magic, version, totalPacketLen, platform, frameNum, timeCPUCycles, numDetectedObj, numTLVs, subFrameNum = struct.unpack(headerStruct, frameData[:frameHeaderLen])
    except:
        print('Error: Could not read frame header')
        outputDict['error'] = 1

    # Move frameData ptr to start of 1st TLV   
    frameData = frameData[frameHeaderLen:]
    totalLenCheck += frameHeaderLen

    # Save frame number to output
    outputDict['frameNum'] = frameNum

    # Initialize the point cloud struct since it is modified by multiple TLV's
    # Each point has the following: X, Y, Z, Doppler, SNR, Noise, Track index
    outputDict['pointCloud'] = np.zeros((numDetectedObj, 7), np.float64)
    # Initialize the track indexes to a value which indicates no track
    outputDict['pointCloud'][:, 6] = 255
    # Find and parse all TLV's
    for i in range(numTLVs):
        try:
            tlvType, tlvLength = tlvHeaderDecode(frameData[:tlvHeaderLength])
            frameData = frameData[tlvHeaderLength:]
            totalLenCheck += tlvHeaderLength
        except:
            print('TLV Header Parsing Failure: Ignored frame due to parsing error')
            outputDict['error'] = 2
            return {}

        # Detected Points
        if (tlvType == MMWDEMO_OUTPUT_MSG_DETECTED_POINTS):
            outputDict['numDetectedPoints'], outputDict['pointCloud'] = parsePointCloudTLV(frameData[:tlvLength], tlvLength, outputDict['pointCloud'])
        # Range Profile
        elif (tlvType == MMWDEMO_OUTPUT_MSG_RANGE_PROFILE):
            outputDict['rangeProfile'] = parseRangeProfileTLV(frameData[:tlvLength])
        # Range Profile
        elif (tlvType == MMWDEMO_OUTPUT_EXT_MSG_RANGE_PROFILE_MAJOR):
            outputDict['rangeProfileMajor'] = parseRangeProfileTLV(frameData[:tlvLength])
        # Range Profile
        elif (tlvType == MMWDEMO_OUTPUT_EXT_MSG_RANGE_PROFILE_MINOR):
            outputDict['rangeProfileMinor'] = parseRangeProfileTLV(frameData[:tlvLength])
        # Sensor Velocity
        elif (tlvType == MMWDEMO_OUTPUT_EXT_MSG_VELOCITY):
            outputDict['velocity'] = parseVelocityTLV(frameData[:tlvLength])
        # Noise Profile
        elif (tlvType == MMWDEMO_OUTPUT_MSG_NOISE_PROFILE):
            pass
        # Static Azimuth Heatmap
        elif (tlvType == MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP):
            pass
        # Range Doppler Heatmap
        elif (tlvType == MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP):
            pass
        # Performance Statistics
        elif (tlvType == MMWDEMO_OUTPUT_MSG_STATS):
            pass
        # Side Info
        elif (tlvType == MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO):
            outputDict['pointCloud'] = parseSideInfoTLV(frameData[:tlvLength], tlvLength, outputDict['pointCloud'])
         # Azimuth Elevation Static Heatmap
        elif (tlvType == MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP):
            pass
        # Temperature Statistics
        elif (tlvType == MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS):
            pass
        # Spherical Points
        elif (tlvType == MMWDEMO_OUTPUT_MSG_SPHERICAL_POINTS):
            outputDict['numDetectedPoints'], outputDict['pointCloud'] = parseSphericalPointCloudTLV(frameData[:tlvLength], tlvLength, outputDict['pointCloud'])
        # Target 3D
        elif (tlvType == MMWDEMO_OUTPUT_MSG_TRACKERPROC_3D_TARGET_LIST or tlvType == MMWDEMO_OUTPUT_EXT_MSG_TARGET_LIST):
            outputDict['numDetectedTracks'], outputDict['trackData'] = parseTrackTLV(frameData[:tlvLength], tlvLength)
        elif (tlvType == MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_HEIGHT):
            outputDict['numDetectedHeights'], outputDict['heightData'] = parseTrackHeightTLV(frameData[:tlvLength], tlvLength)
         # Target index
        elif (tlvType == MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_INDEX or tlvType ==  MMWDEMO_OUTPUT_EXT_MSG_TARGET_INDEX):
            outputDict['trackIndexes'] = parseTargetIndexTLV(frameData[:tlvLength], tlvLength)
         # Capon Compressed Spherical Coordinates
        elif (tlvType == MMWDEMO_OUTPUT_MSG_COMPRESSED_POINTS):
            outputDict['numDetectedPoints'], outputDict['pointCloud'] = parseCompressedSphericalPointCloudTLV(frameData[:tlvLength], tlvLength, outputDict['pointCloud'])
        # Presence Indication
        elif (tlvType == MMWDEMO_OUTPUT_MSG_PRESCENCE_INDICATION):
            pass
        # Occupancy State Machine
        elif (tlvType == MMWDEMO_OUTPUT_MSG_OCCUPANCY_STATE_MACHINE):
            outputDict['occupancy'] = parseOccStateMachTLV(frameData[:tlvLength])
        elif (tlvType == MMWDEMO_OUTPUT_MSG_VITALSIGNS):
            outputDict['vitals'] = parseVitalSignsTLV(frameData[:tlvLength], tlvLength)
        elif(tlvType == MMWDEMO_OUTPUT_EXT_MSG_DETECTED_POINTS):
            outputDict['numDetectedPoints'], outputDict['pointCloud'] = parsePointCloudExtTLV(frameData[:tlvLength], tlvLength, outputDict['pointCloud'])
        elif (tlvType == MMWDEMO_OUTPUT_MSG_GESTURE_FEATURES_6843):
            outputDict['features'] = parseGestureFeaturesTLV(frameData[:tlvLength])
        elif (tlvType == MMWDEMO_OUTPUT_MSG_GESTURE_OUTPUT_PROB_6843):
            outputDict['gestureNeuralNetProb'] = parseGestureProbTLV6843(frameData[:tlvLength])
        elif (tlvType == MMWDEMO_OUTPUT_MSG_GESTURE_FEATURES_6432): # 6432 features output 350
            outputDict['gestureFeatures'] = parseGestureFeaturesTLV6432(frameData[:tlvLength])
        elif (tlvType == MMWDEMO_OUTPUT_MSG_GESTURE_CLASSIFIER_6432):
            outputDict['gesture'] = parseGestureClassifierTLV6432(frameData[:tlvLength])
        elif (tlvType == MMWDEMO_OUTPUT_MSG_GESTURE_PRESENCE_x432):
            # outputDict['gesturePresence'] = parseGesturePresenceTLV6432(frameData[:tlvLength])
            pass
        elif (tlvType == MMWDEMO_OUTPUT_MSG_GESTURE_PRESENCE_THRESH_x432):
            pass
        # Performance Statistics
        elif (tlvType == MMWDEMO_OUTPUT_MSG_EXT_STATS):
            outputDict['procTimeData'], outputDict['powerData'], outputDict['tempData'] \
            = parseExtStatsTLV(frameData[:tlvLength], tlvLength)
        # Presence Detection in each zone
        elif(tlvType == MMWDEMO_OUTPUT_EXT_MSG_ENHANCED_PRESENCE_INDICATION):
            outputDict['enhancedPresenceDet'] = parseEnhancedPresenceInfoTLV(frameData[:tlvLength], tlvLength)
        # Probabilities output by the classifier
        elif(tlvType == MMWDEMO_OUTPUT_EXT_MSG_CLASSIFIER_INFO):
            outputDict['classifierOutput'] = parseClassifierTLV(frameData[:tlvLength], tlvLength)
        # Raw data from uDoppler extraction around targets
        elif(tlvType == MMWDEMO_OUTPUT_EXT_MSG_MICRO_DOPPLER_RAW_DATA):
            pass
        # uDoppler features from each target
        elif(tlvType == MMWDEMO_OUTPUT_EXT_MSG_MICRO_DOPPLER_FEATURES):
            pass
        # Surface classification value
        elif(tlvType == MMWDEMO_OUTPUT_MSG_SURFACE_CLASSIFICATION):
            outputDict['surfaceClassificationOutput'] = parseSurfaceClassificationTLV(frameData[:tlvLength])
        elif(tlvType == MMWDEMO_OUTPUT_EXT_MSG_RX_CHAN_COMPENSATION_INFO):
            outputDict['rx_chan_comp'] = parseRXChanCompTLV(frameData[:tlvLength], tlvLength)
        else:
            print ("Warning: invalid TLV type: %d" % (tlvType))

        # Move to next TLV
        frameData = frameData[tlvLength:]
        totalLenCheck += tlvLength
    
    # Pad totalLenCheck to the next largest multiple of 32
    # since the device does this to the totalPacketLen for transmission uniformity
    totalLenCheck = 32 * math.ceil(totalLenCheck / 32)

    # Verify the total packet length to detect transmission error that will cause subsequent frames to dropped
    if (totalLenCheck != totalPacketLen):
        print('Warning: Frame packet length read is not equal to totalPacketLen in frame header. Subsequent frames may be dropped.')
        outputDict['error'] = 3

    return outputDict

# Decode TLV Header
def tlvHeaderDecode(data):
    tlvType, tlvLength = struct.unpack('2I', data)
    return tlvType, tlvLength

