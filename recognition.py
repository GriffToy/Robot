from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import imutils
import cv2

# Origin is upper left of screen
FRAMEWIDTH = 512
FRAMEHEIGHT = 384
TARGETWIDTH = 32    # Width of target box. Must be between 0 and FRAMEWIDTH.
TARGETHEIGHT = 64   # Height of target box. Must be between 0 and FRAMEWIDTH.
LOWERX = (FRAMEWIDTH - TARGETWIDTH)/2
UPPERX = (FRAMEWIDTH + TARGETWIDTH)/2
LOWERY = (FRAMEHEIGHT - TARGETHEIGHT)/2
UPPERY = (FRAMEHEIGHT + TARGETHEIGHT)/2

def setup():
    # sets picamera up and returns camera object
    camera = PiCamera()
    camera.resolution = (FRAMEWIDTH, FRAMEHEIGHT)
    camera.vflip = True
    camera.hflip = True
    rawCapture = PiRGBArray(camera)
    time.sleep(0.1)
    return camera


def detect(camera, XMLFile, scale = 1.1, neighbors = 3, mSize = (0, 0)):
    # Takes a picture and returns cones cascade objects
    cone_cascade = cv2.CascadeClassifier(XMLFile)
    rawCapture = PiRGBArray(camera)
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cones = cone_cascade.detectMultiScale(gray, scaleFactor = scale, minNeighbors = neighbors, minSize = mSize)
    #cones = cone_cascade.detectMultiScale(gray, scaleFactor = 1.3, minNeighbors = 6, minSize = (60, 60))
    rawCapture.close()
    return cones

def biggestObject(objectTupleArray):
    # Returns the index of the object with biggest area, -1 if there is no object detected
    if len(objectTupleArray) > 0:
        index = 0
        area = 0
        i = 0
        for (x, y, w, h) in objectTupleArray:
            if h * w > area:
                index = i
                area = h * w
            i += 1
        return index
    return -1

def printObjects(objectTupleArray):
    # Iterates through the objectTupleArray and prints the tupleArray and number of objects
    print "Number of objects detected: " + `len(objectTupleArray)`
    print "Index of largest object: " + `biggestObject(objectTupleArray)`
    print "Object tuple array: " + `objectTupleArray`

def isCentered(objectTuple):
    # Returns true if object is within target area
    x = objectTuple[0]
    y = objectTuple[1]
    w = objectTuple[2]
    h = objectTuple[3]
    xmid = x + w/2
    ymid = y + h/2
    if xmid > LOWERX and xmid < UPPERX and ymid > LOWERY and ymid < UPPERY:
        return True
    return False
