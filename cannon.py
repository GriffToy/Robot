import time
import RPi.GPIO as GPIO

FRAMEWIDTH = 512
FRAMEHEIGHT = 384

UPPIN = 16	# out
DOWNPIN = 18	# out
LEFTPIN = 19    # out
RIGHTPIN = 21   # out
FIREPIN = 22	# out
SHOTFIRED = 33	# in
UPPERLIMIT = 29	# in
LOWERLIMIT = 31	# in
LEFTLIMIT = 35	# in
RIGHTLIMIT = 37	# in
SMALLFIREPIN = 32 # out


L_TIME_CONSTANT = 2.488
R_TIME_CONSTANT = 2.275
U_TIME_CONSTANT = 0.411
D_TIME_CONSTANT = 0.399
H_POSITION = 0.5
V_POSITION = 0.5
NUMXFRAMES = 10	# 6, 5.5Number of horizontal frames in a complete sweep
NUMYFRAMES = 1.8	# Number of vertical frames in a complete sweep
H_DEGREES = 270
DIRECTION = 1

GPIO.setmode(GPIO.BOARD)
GPIO.setup(UPPIN, GPIO.OUT)
GPIO.setup(DOWNPIN, GPIO.OUT)
GPIO.setup(LEFTPIN, GPIO.OUT)
GPIO.setup(RIGHTPIN, GPIO.OUT)
GPIO.setup(FIREPIN, GPIO.OUT)
GPIO.setup(SMALLFIREPIN, GPIO.OUT)
GPIO.setup(SHOTFIRED, GPIO.IN)
GPIO.setup(UPPERLIMIT, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(LOWERLIMIT, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(LEFTLIMIT, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(RIGHTLIMIT, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)


# positions are 0-1, left-right, down-up

def init():
    global L_TIME_CONSTANT
    global R_TIME_CONSTANT
    global U_TIME_CONSTANT
    global D_TIME_CONSTANT
    pointRight()
    L_TIME_CONSTANT = pointLeft()
    print "L Time constant: ", L_TIME_CONSTANT
    R_TIME_CONSTANT = pointRight()
    print "R Time constant: ", R_TIME_CONSTANT
    setH(0.5)
    pointDown()
    U_TIME_CONSTANT = pointUp()
    print "U Time constant: ", U_TIME_CONSTANT
    D_TIME_CONSTANT = pointDown()
    print "D Time constant: ", D_TIME_CONSTANT
    setV(0.3)
    print "(X,Y)", H_POSITION, V_POSITION


def pointDown():
    # Points cannon up until it reaches limit
    startTime = time.time()
    while not GPIO.input(LOWERLIMIT):
        GPIO.output(DOWNPIN, GPIO.HIGH)
    GPIO.output(DOWNPIN, GPIO.LOW)
    V_POSITION = 1
    return time.time() - startTime

def pointUp():
    # Points cannon down until it reaches limit
    startTime = time.time()
    while not GPIO.input(UPPERLIMIT):
        GPIO.output(UPPIN, GPIO.HIGH)
    GPIO.output(UPPIN, GPIO.LOW)
    V_POSITION = 0
    return time.time() - startTime

def pointLeft():
    global H_POSITION
    # Points cannon left until it reaches limit
    startTime = time.time()
    while not GPIO.input(LEFTLIMIT):
        GPIO.output(LEFTPIN, GPIO.HIGH)
    GPIO.output(LEFTPIN, GPIO.LOW)
    H_POSITION = 0
    return time.time() - startTime

def pointRight():
    global H_POSITION
    # Points cannon right until it reaches limit
    startTime = time.time()
    while not GPIO.input(RIGHTLIMIT):
        GPIO.output(RIGHTPIN, GPIO.HIGH)
    GPIO.output(RIGHTPIN, GPIO.LOW)
    H_POSITION = 1
    return time.time() - startTime

def setH(p):
    global H_POSITION
    if p > 1:
        p = 1
    startTime = time.time()
    if p > H_POSITION:
        while time.time() - startTime < R_TIME_CONSTANT * (p - H_POSITION):
            GPIO.output(RIGHTPIN, 1)
            if GPIO.input(RIGHTLIMIT):
                p = 1
                break
        GPIO.output(RIGHTPIN, 0)
    else:
        while time.time() - startTime < L_TIME_CONSTANT * (H_POSITION - p):
            GPIO.output(LEFTPIN, 1)
            if GPIO.input(LEFTLIMIT):
                p = 0
                break
        GPIO.output(LEFTPIN, 0)
    H_POSITION = p
    
def setV(p):
    global V_POSITION
    if p > 1:
        p = 1
    startTime = time.time()
    if p < V_POSITION:
        while time.time() - startTime < U_TIME_CONSTANT * (V_POSITION - p):
            GPIO.output(UPPIN, 1)
            if GPIO.input(UPPERLIMIT):
                p = 0
                break
        GPIO.output(UPPIN, 0)
    else:
        while time.time() - startTime < D_TIME_CONSTANT * (p - V_POSITION):
            GPIO.output(DOWNPIN, 1)
            if GPIO.input(LOWERLIMIT):
                p = 1
                break
        GPIO.output(DOWNPIN, 0)
    V_POSITION = p

# Changes: added moveH, moveV functions. changed the logic of setV/setH if p > 1 to set p equal to 1.
# changed the sign of if p < V_POSITION so that vertical origin is on top.	
# changed the V_position set in point up/down so that from up = 1 and down = 0 to up = 0 and down = 1
def moveH(amount):
	# Moves cannon amount units horizontally, where 1 is a full right turn and -1 is full left turn
	setH(H_POSITION + amount)
	
def moveV(amount):
	# Moves cannon amount units vertically, where 1 is a full up and -1 is full down
	setV(V_POSITION + amount)
	
def moveTo(xCoord, yCoord):
	xCoordShifted = xCoord - 256		# Converts x coordinates between -256 and 256
	yCoordShifted = yCoord - 192		# Converts y coordinates between -192 and 192
	xNormalized = xCoordShifted/256.0	# Converts x coordinates between -1 and 1
	yNormalized = yCoordShifted/192.0	# Convertx y coordinates between -1 and 1
	xAmount = xNormalized/NUMXFRAMES
	yAmount = yNormalized/NUMYFRAMES
	moveH(xAmount)
	moveV(yAmount)
	#time.sleep(1)

def getAngle():
    return(H_POSITION*H_DEGREES - H_DEGREES/2)

def moveAngle(angle):
    moveH(float(angle)/float(H_DEGREES))

def sweepH(amount):
    global DIRECTION
    if GPIO.input(LEFTLIMIT) or GPIO.input(RIGHTLIMIT) or H_POSITION >= 1 or H_POSITION <= 0:
        DIRECTION = DIRECTION * -1
        setV(0.3)
    moveH(DIRECTION * amount)

def firex(shots):
    for i in range(shots):
        GPIO.output(FIREPIN, GPIO.HIGH)
        while not GPIO.input(SHOTFIRED):
            time.sleep(0.1)
            #print GPIO.input(SHOTFIRED)
        while GPIO.input(SHOTFIRED):
            time.sleep(0.1)
            #print GPIO.input(SHOTFIRED)
        time.sleep(0.6)
    GPIO.output(FIREPIN, GPIO.LOW)

def firex_small(shots):
    GPIO.output(SMALLFIREPIN, 1)
    time.sleep(0.04*shots)
    GPIO.output(SMALLFIREPIN, 0)
