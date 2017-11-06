import time
import RPi.GPIO as GPIO

import navigate
import cannon
import recognition
import random
from multiprocessing import *
import multiprocessing
from picamera.array import PiRGBArray
from picamera import PiCamera

# Constants
MIN_DISTANCE = 15
US_ANGLE = [-90, -45, 0, 45, 90]
DART_COUNT = 4
SMALL_DART_COUNT = 6
CONEFILE = 'CascadeFiles/cascadecone9.xml'
INVERTEDCONEFILE = 'CascadeFiles/cascadereload8.xml'
SPHEREFILE = 'CascadeFiles/cascadeball10.xml'
TIMEOUT = 8


def moveRandom(xmin = 0, xmax = 1, ymin = 0, ymax = 1):
    # Moves cannon to a random location.
    xCoord = random.uniform(xmin, xmax)
    yCoord = random.uniform(ymin, ymax)
    print "Moving to xCoord: " + `xCoord` + " yCoord: " + `yCoord`
    cannon.setH(xCoord)
    cannon.setV(yCoord)

def sentryMode():
    camera = recognition.setup()
    try:
        # Function that will look for a cone and fire if found. Used after a magnet is detected.
        global DART_COUNT
        if DART_COUNT > 0:
            cannon.init()
            lastSeenTime = time.time()
        while DART_COUNT > 0:
            print "detect, check, fire or track"
            targetObjects = recognition.detect(camera, SPHEREFILE)
            index = recognition.biggestObject(targetObjects)
            if index != -1:
                lastSeenTime = time.time()
                # Check if biggest object is in the middle of the screen and fire if it is
                if recognition.isCentered(targetObjects[index]):
                    print "Firing"
                    cannon.moveV(0.1)
                    navigate.blink()
                    cannon.firex(1)
                    cannon.moveV(-0.1)
                    DART_COUNT = DART_COUNT - 1
                else:
                        # Move cannon
                        biggestObject =targetObjects[index]
                        xTarg = biggestObject[0] + biggestObject[2]/2
                        yTarg = biggestObject[1] + biggestObject[3]/2
                        print "Found an object, moving to coordinates, x: " + `xTarg` + " y: " + `yTarg`
                        cannon.moveTo(xTarg, yTarg)
            elif time.time() - lastSeenTime > TIMEOUT:
                cannon.sweepH(0.08)
                time.sleep(0.2)
        print "Out of darts"
    finally:
        #GPIO.cleanup()
        camera.close()
        print "Ran the sentry close"

def smallSentry(mag_q):
    #search for inverted cones
    #track them and return their angle compared to center of robot (-180 to 180)
    #possibly implement reverse queue to turn cannon when hitting navigaton error.
    camera = recognition.setup()
    cannon.init()
    lastSeenTime = time.time()
    try:
        # Function that will look for a cone and fire if found. Used after a magnet is detected.
        foundMagnet = None
        while foundMagnet != 'Stop':
            try:
                foundMagnet = mag_q.get(timeout = 0.001)
                print foundMagnet
            except:
                pass
            
            print "In small sentry."
            targetObjects = recognition.detect(camera, SPHEREFILE)
            index = recognition.biggestObject(targetObjects)
            if index != -1:
                lastSeenTime = time.time()
                # Move cannon
                if recognition.isCentered(targetObjects[index]):
                    print "Firing"
                    cannon.moveV(0.1)
                    cannon.moveH(-0.03)
                    navigate.blink()
                    cannon.firex_small(1)
                    cannon.moveH(0.03)
                    cannon.moveV(-0.1)
                else:
                        # Move cannon
                        biggestObject =targetObjects[index]
                        xTarg = biggestObject[0] + biggestObject[2]/2
                        yTarg = biggestObject[1] + biggestObject[3]/2
                        print "Found an object, moving to coordinates, x: " + `xTarg` + " y: " + `yTarg`
                        cannon.moveTo(xTarg * 1.10, yTarg)
            elif time.time() - lastSeenTime > TIMEOUT:
                cannon.sweepH(0.08)
                time.sleep(0.2)
                print "Sweeping"
    finally:
        camera.close()
        print "Ran the smallSentry close"

def navigateRobot():
    navigate.init()
    mag_q = multiprocessing.Queue()
    scanProcess = Process(target = smallSentry, args = (mag_q,))
    scanProcess.daemon = True
    scanProcess.start()
    try:
        while True:
            distance = navigate.readDistance()
            angle = navigate.calcAngle(distance)
            navigate.turnTowards(angle)
            if navigate.isMagnet():
                #print "Should go into sentry mode. If you see this, it is commented out"
                print "Found magnet. Going into sentry mode."
                mag_q.put('Stop')
                scanProcess.join()
                return -1
            
            if navigate.isLeftWhite():
                initial = distance[3]
                while(navigate.isLeftWhite()): navigate.turnInPlace(-1)
                navigate.turnAngle(20)
                while (distance[3] > initial/2):
                    navigate.turnTowards(0)
                    distance = navigate.readDistance()
                    if min(distance) < MIN_DISTANCE: break
                    if navigate.isLeftWhite() or navigate.isRightWhite(): break
                    
            if navigate.isRightWhite():
                initial = distance[1]
                while(navigate.isRightWhite()): navigate.turnInPlace(1)
                navigate.turnAngle(-20)
                while (distance[1] > initial/2):
                    navigate.turnTowards(0)
                    distance = navigate.readDistance()
                    if min(distance) < MIN_DISTANCE: break
                    if navigate.isLeftWhite() or navigate.isRightWhite(): break
                    
            if distance[2] < MIN_DISTANCE:
                navigate.turnAngle(US_ANGLE[distance.index(max(distance))])
    finally:
        #GPIO.cleanup()
        scanProcess.join()
        print "Ran the navigate close"

def navigateBack():
    print "Navigating back."
    navigate.init()
    nav_q = multiprocessing.Queue()
    scan_q = multiprocessing.Queue()
    scanProcess = Process(target = scan, args = (nav_q,scan_q,))
    scanProcess.daemon = True
    scanProcess.start()
    
    targetAngle = None
    currentAngle = 0
    currentTime = time.time()
    lastTime = time.time()
    Error = False
    
    while targetAngle != 'STOP':
	    
        distance = navigate.readDistance()
        if abs(currentAngle) > 360:
            #there was a problem, just wait for a new bearing
            targetAngle = None
            currentAngle = 0
            
        if targetAngle != None:
            # Read the current angle
            currentTime = time.time()
            dTime = currentTime - lastTime
            lastTime = currentTime
            if not Error:
                currentAngle = currentAngle - navigate.readGyro() * dTime
            
            
            # calculate delta angle between current and target
            dAngle = targetAngle - currentAngle
            if dAngle < -180:
                currentAngle += 360
                dAngle = targetAngle - currentAngle
            if dAngle > 180:
                currentAngle -= 360
                dAngle = targetAngle - currentAngle
            #navigate based on delta angle

            print currentAngle, targetAngle, dAngle

            if abs(dAngle) < 180:
                scale = (180 - abs(dAngle))/180.0
                if dAngle < 0:
                    distance[3] *= scale
                    distance[4] *= scale
                    if distance[3] < 10: distance[3] = 10
                    if distance[4] < 10: distance[4] = 10
                elif dAngle > 0:
                    distance[0] *= scale
                    distance[1] *= scale
                    if distance[0] < 10: distance[0] = 10
                    if distance[1] < 10: distance[1] = 10
                angle = navigate.calcAngle(distance)
                navigate.turnTowards(angle)
                Error = False
            '''
            elif abs(dAngle) > 90:
                scan_q.put(dAngle)
                navigate.turnAngle(dAngle)
                currentAngle += dAngle
                Error = True
            '''    
            if min(distance) < 6:
                #find if left or right too close
                print "Emergency turn"
                if distance.index(min(distance)) > 2:
                    navigate.turnAngle(-20)
                    currentAngle += -20
                    scan_q.put(-20)
                if distance.index(min(distance)) < 2:
                    navigate.turnAngle(20)
                    currentAngle += 20
                    scan_q.put(20)
                time.sleep(0.5)
                Error = True

        try:
            targetAngle = nav_q.get(timeout = 0.001)
            print "Target angle:", targetAngle
            currentAngle = 0
        except:
            pass
        
        if targetAngle == None:
            angle = navigate.calcAngle(distance)
            navigate.turnTowards(angle)  

            
        if navigate.isLeftWhite():
            initial = distance[3]
            while(navigate.isLeftWhite()): navigate.turnInPlace(-1)
            scan_q.put(20)
            navigate.turnAngle(20)
            currentAngle += 20
            
            Error = True
            while (distance[3] > initial/2):
                navigate.turnTowards(0)
                distance = navigate.readDistance()
                if min(distance) < MIN_DISTANCE: break
                if navigate.isLeftWhite() or navigate.isRightWhite(): break
                    
        if navigate.isRightWhite():
            initial = distance[1]
            while(navigate.isRightWhite()): navigate.turnInPlace(1)
            scan_q.put(-20)
            navigate.turnAngle(-20)
            currentAngle -= 20
            
            Error = True
            while (distance[1] > initial/2):
                navigate.turnTowards(0)
                distance = navigate.readDistance()
                if min(distance) < MIN_DISTANCE: break
                if navigate.isLeftWhite() or navigate.isRightWhite(): break
                    
        if distance[2] < MIN_DISTANCE:
            currentAngle += US_ANGLE[distance.index(max(distance))]
            scan_q.put(US_ANGLE[distance.index(max(distance))])
            navigate.turnAngle(US_ANGLE[distance.index(max(distance))])
            Error = True
            
    scanProcess.join()

def scan(nav_q, scan_q):
    #search for inverted cones
    #track them and return their angle compared to center of robot (-180 to 180)
    #possibly implement reverse queue to turn cannon when hitting navigaton error.
    camera = recognition.setup()
    cannon.init()
    lastSeenTime = time.time()
    try:
        # Function that will look for a cone and fire if found. Used after a magnet is detected.
        while True:
            print "10"
            try:
                targetAngle = scan_q.get(timeout = 0.001)
                print "Cannon Target angle:", targetAngle
                cannon.moveAngle(-targetAngle)
            except:
                pass
            
            print "detect, check, fire or track"
            targetObjects = recognition.detect(camera, INVERTEDCONEFILE)
            index = recognition.biggestObject(targetObjects)
            if index != -1:
                lastSeenTime = time.time()
                # Move cannon
                biggestObject =targetObjects[index]
                xTarg = biggestObject[0] + biggestObject[2]/2
                yTarg = biggestObject[1] + biggestObject[3]/2
                print "Found an object, moving to coordinates, x: " + `xTarg` + " y: " + `yTarg`
                cannon.moveTo(xTarg * 1.05, yTarg) # Might consider multiplying xTarg by a constant, so it overshoots, since robot is moving. 
                nav_q.put(cannon.getAngle())
                print "putting value: " + `cannon.getAngle()`
            elif time.time() - lastSeenTime > TIMEOUT:
                cannon.sweepH(0.08)
                time.sleep(0.2)
                print "Sweeping"
    finally:
        nav_q.put("STOP")
        camera.close()
        GPIO.cleanup()
        print "Ran the scan close"

def scanTest():
    q = multiprocessing.Queue()
    scanProcess = Process(target = scan, args = (q,))
    scanProcess.daemon = True
    scanProcess.start()
    targetAngle = 0
    try:
        while targetAngle != "STOP":
            try:
                targetAngle = q.get(timeout = 0.001)
                print "Target angle:", targetAngle
                currentAngle = 0
            except:
                pass
    finally:
        scanProcess.join()
    

if __name__ == "__main__":
    try:
        navigateRobot()
        sentryProcess = Process(target = sentryMode,)
        sentryProcess.start()
        sentryProcess.join()
        navigateBack()
        '''cannon.init()
        cannon.moveAngle(-45)
        print cannon.getAngle()
        time.sleep(1)
        
        cannon.moveAngle(90)
        print cannon.getAngle()
        time.sleep(1)

        cannon.moveAngle(-180)
        print cannon.getAngle()
        time.sleep(1)'''
                               
    finally:
        GPIO.cleanup()
        print "Exiting"

