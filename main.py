#!/usr/bin/env python3
#----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
#----------------------------------------------------------------------------

import json
import time
import sys

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTablesInstance
import ntcore
import numpy as np
import cv2

#   JSON format:
#   {
#       "team": <team number>,
#       "ntmode": <"client" or "server", "client" if unspecified>
#       "cameras": [
#           {
#               "name": <camera name>
#               "path": <path, e.g. "/dev/video0">
#               "pixel format": <"MJPEG", "YUYV", etc>   // optional
#               "width": <video mode width>              // optional
#               "height": <video mode height>            // optional
#               "fps": <video mode fps>                  // optional
#               "brightness": <percentage brightness>    // optional
#               "white balance": <"auto", "hold", value> // optional
#               "exposure": <"auto", "hold", value>      // optional
#               "properties": [                          // optional
#                   {
#                       "name": <property name>
#                       "value": <property value>
#                   }
#               ],
#               "stream": {                              // optional
#                   "properties": [
#                       {
#                           "name": <stream property name>
#                           "value": <stream property value>
#                       }
#                   ]
#               }
#           }
#       ]
#       "switched cameras": [
#           {
#               "name": <virtual camera name>
#               "key": <network table key used for selection>
#               // if NT value is a string, it's treated as a name
#               // if NT value is a double, it's treated as an integer index
#           }
#       ]
#   }

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []

KNOWN_DISTANCE = 118 # distância real do objeto para a câmera (só deve ser usada para calibrar a largura focal da câmera)
KNOWN_WIDTH = 55 # largura real do objeto (usada para calcular a distância da câmera para o objeto)
F = 333.82 # focal length
STANDART_ERROR = 105

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

def readSwitchedCameraConfig(config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False

    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    inst = CameraServer.getInstance()
    camera = UsbCamera(config.name, config.path)
    server = inst.startAutomaticCapture(camera=camera, return_server=True)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera

def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.getInstance().addSwitchedCamera(config.name)

    def listener(fromobj, key, value, isNew):
        if isinstance(value, float):
            i = int(value)
            if i >= 0 and i < len(cameras):
              server.setSource(cameras[i])
        elif isinstance(value, str):
            for i in range(len(cameraConfigs)):
                if value == cameraConfigs[i].name:
                    server.setSource(cameras[i])
                    break

    NetworkTablesInstance.getDefault().getEntry(config.key).addListener(
        listener,
        ntcore.constants.NT_NOTIFY_IMMEDIATE |
        ntcore.constants.NT_NOTIFY_NEW |
        ntcore.constants.NT_NOTIFY_UPDATE)

    return server

# INÍCIO DAS FUNÇÕES DESENVOLVIDAS POR NÓS

def process(frame, vt): # método pra processar as imagens
    # Menores valores possiveis pra Threshold HSV (peguei do GRIP)
    low_H = 50
    low_S = 50
    low_V = 100

    # Maiores valores possiveis para Threshold HSV (peguei do GRIP)
    high_H = 200
    high_S = 200
    high_V = 255
    
    # filtro
    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # converte o frame pra HSV
    frame_threshold = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V)) # troca os frames que nao batem com os valores pra preto

    return frame_threshold

def distance_to_object(known_width, focal_length, percieved_width):
    return (known_width * focal_length) / percieved_width
    
def find_object(frame, bbox=[0, 0, 640, 480]):
    x1, y1, w, h = bbox
    x2 = x1 + w
    y2 = y1 + h
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
    roi = frame[y1:y2, x1:x2]
    height, width = frame.shape
    _, contours, img = cv2.findContours(roi, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        contourArea = cv2.contourArea(contour)
        x,y,w,h = cv2.boundingRect(contour) # pega um retangulo baseado no contorno
        ratio = w/h
        # print([x, y, w, h])
        density = contourArea/(w*h)
        print("Ratio: {0} Density: {1}".format(ratio, density))
        if evaluate(ratio, density):
            rectangle = [x, y, w, h]
            return rectangle

def evaluate(ratio, density):
    if ratio > 2 and ratio < 2.5:
        if density > 0.05 and density < 0.15:
            return True
    return False

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)

    # start cameras
    for config in cameraConfigs:
        cameras.append(startCamera(config))

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)

    shuffle = ntinst.getTable("Shuffleboard")
    vision_table = shuffle.getSubTable("Vision")
    cs = CameraServer.getInstance()
    # CvSink objects allows you to apply OpenCV magic to CameraServer frames 
    cv_sink = cs.getVideo()
    
    # CvSource objects allows you to pass OpenCV frames to your CameraServer
    filterStream = cs.putVideo("HSV Filter", 160,120)

    # numpy buffer to store image data
    # if you don't do this, the CvSink glitches out and gives you something that's not a np array
    # I haven't really played with this, so feel free to play around and save processing power
    img = np.zeros(shape=(160,120,3), dtype=np.uint8)
    # loop forever
    while True:
        frame_time, img = cv_sink.grabFrame(img)
        if frame_time == 0:
            filterStream.notifyError(cv_sink.getError())
            continue
        filtered = process(img, vision_table)

        obj = find_object(filtered)

        if obj:
            center = obj[0] + (obj[2] / 2)
            diff = (640/2) - center + STANDART_ERROR
            dist = distance_to_object(KNOWN_WIDTH, F, obj[3])
            # print("Dist: {0} Diff: {1}".format(dist, diff))
            vision_table.putNumber('Difference', diff)
            vision_table.putNumber('Distance', dist)
    
        filterStream.putFrame(filtered)
