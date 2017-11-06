from BrickPi import *
import FaBo9Axis_MPU9250
import time
import RPi.GPIO as GPIO
import signal


# Constants
STOPPING_DISTANCE = 25  # Used for center ultrasonic sensor
SIDE_THRESHOLD = 25
TURN_SPEED = 75#100
FORWARD_SPEED = 75#100
MAG_OFFSET_Y = 90 # Run "magnetTesting.py" and adjust y and z offsets to the abs. avg. y and z values.
MAG_OFFSET_Z = 130
MAGNET_SENSITIVITY = 50#100
LIGHT_THRESHOLD = 500 # White is less than this value, black is greater
ROTATE_SPEED = 50 # Speed the ultrasonic sensor rotates
US_PIN = [7, 11, 12, 13, 15]
US_ANGLE = [-90, -45, 0, 45, 90]
mpu9250 = FaBo9Axis_MPU9250.MPU9250()

# Globals
dist = [0]*5

def signal_handler(signum, frame):
    raise Exception("Ultrasonic Timeout")

def blink():
    GPIO.setup(13, GPIO.OUT)
    for x in range(5):
        try:
            GPIO.output(13, False)
            time.sleep(0.05)
            GPIO.output(13, True)
            time.sleep(0.05)  
        except KeyboardInterrupt:
            GPIO.output(13, False)
            break
    GPIO.output(13, False)

def init():

    BrickPiSetup()  # setup the serial port for communication

    BrickPi.SensorType[PORT_4] = TYPE_SENSOR_LIGHT_ON  # Light sensor
    BrickPi.SensorType[PORT_1] = TYPE_SENSOR_LIGHT_ON  # Light sensor
    BrickPiSetupSensors()   #Send the properties of sensors to BrickPi

    # Motors A and B run on same CPU. Use these ports if you want robot to go constant speed.
    BrickPi.MotorEnable[PORT_A] = 1 #Enable the Motor A
    BrickPi.MotorEnable[PORT_B] = 1 #Enable the Motor B
    signal.signal(signal.SIGALRM, signal_handler)

def calcAngle(d):
    weight = 0
    for i in range(5): weight += d[i] * US_ANGLE[i]
    return weight / (sum(d) - d[2])
    

def readDistance():
    global dist
    signal.alarm(1)
    try:
        GPIO.setmode(GPIO.BOARD)
        for i in range(5):
            GPIO.setup(US_PIN[i], GPIO.OUT)

            time.sleep(0.000002)

            GPIO.output(US_PIN[i], 0)
            time.sleep(0.000002)
    
            GPIO.output(US_PIN[i], 1)
            time.sleep(0.000005)
    
            GPIO.output(US_PIN[i], 0)
            time.sleep(0.000002)

            GPIO.setup(US_PIN[i], GPIO.IN)

            while GPIO.input(US_PIN[i])==0:
                starttime = time.time()

            while GPIO.input(US_PIN[i])==1:
                endtime = time.time()

            duration = endtime-starttime
            dist[i] = duration*34000/2
        signal.alarm(0)
        return dist
    except Exception, msg:
        print "Timeout"
        return [30]*5
def isLeftWhite():
        # Returns True if color sensor detects white, False otherwise.
        BrickPiUpdateValues()
        color = BrickPi.Sensor[PORT_4]
        if color < LIGHT_THRESHOLD:
                return True
        return False

def isRightWhite():
        # Returns True if color sensor detects white, False otherwise.
        BrickPiUpdateValues()
        color = BrickPi.Sensor[PORT_1]
        if color < LIGHT_THRESHOLD:
                return True
        return False

def stopRobot():
        # Sets both motors to 0
	BrickPi.MotorSpeed[PORT_A] = 0 
	BrickPi.MotorSpeed[PORT_B] = 0
	BrickPiUpdateValues()

def isMagnet():
        # Returns True if found a magnet, False otherwise.
        mag = mpu9250.readMagnet()
        if abs(mag['z']) - MAG_OFFSET_Z > MAGNET_SENSITIVITY or abs(mag['y']) - MAG_OFFSET_Y > MAGNET_SENSITIVITY:
                # Only need to check y and z directions, as robot will be moving forward.
                # Basically creates a detection line running from the inside of both wheels.
                return True
        return False

def turnAngle(desiredAngle):
	# Turns desireAngle degrees. IMU must be laying flat.
	current_time = time.time()
	last_time = time.time()
	#current_time = int(round(time.time() * 1000))
	#last_time = int(round(time.time() * 1000))
	dt = 0
	angle = 0
	# Set each motor's speed to turn accordingly
	if desiredAngle < 0:	# Turn counter-clockwise (left)
                print("Turning left")
		BrickPi.MotorSpeed[PORT_A] = TURN_SPEED
		BrickPi.MotorSpeed[PORT_B] = -TURN_SPEED
	elif desiredAngle > 0:	# Turn clockwise (right)
                print("Turning right")
		BrickPi.MotorSpeed[PORT_A] = -TURN_SPEED
		BrickPi.MotorSpeed[PORT_B] = TURN_SPEED
	else:		        # Don't turn at all
                print("Not turning at all")
		BrickPi.MotorSpeed[PORT_A] = 0
		BrickPi.MotorSpeed[PORT_B] = 0
		return
	#print("Current angle: " + `angle` + " Desired angle: " + `desiredAngle`)
    # Turn until the desired angle is reached
	while abs(angle) < abs(desiredAngle):
		BrickPiUpdateValues()
		gyro = mpu9250.readGyro()
		current_time = time.time()
		dt = current_time - last_time
		#current_time = int(round(time.time() * 1000))	# Should look into why it is multiplied by 1000, then
		#dt = (current_time - last_time)/1000.0			# divided by 1000 in this line
		last_time = current_time
		angle = angle + gyro['z'] * dt
		#print( "Angle: " + `angle`)
		#print( " gz = " + `( gyro['z'])`)
	BrickPi.MotorSpeed[PORT_A] = 0
	BrickPi.MotorSpeed[PORT_B] = 0
	BrickPiUpdateValues()

def readGyro():
    return mpu9250.readGyro()['z']

def turnTowards(angle):
    rightspeed = 0
    leftspeed = 0
    if angle < 0:
        rightspeed = FORWARD_SPEED
        leftspeed = FORWARD_SPEED + angle*1.42

    else:
        rightspeed = FORWARD_SPEED - angle*1.42
        leftspeed = FORWARD_SPEED    
        
    BrickPi.MotorSpeed[PORT_A] = int(rightspeed)    #Set the speed of MotorA (-255 to 255)
    BrickPi.MotorSpeed[PORT_B] = int(leftspeed)     #Set the speed of MotorB (-255 to 255)
    BrickPiUpdateValues()

def turnInPlace(direction):
    # -1 for left 1 for right
    BrickPi.MotorSpeed[PORT_A] = FORWARD_SPEED * direction
    BrickPi.MotorSpeed[PORT_B] = - FORWARD_SPEED * direction
    BrickPiUpdateValues()
    
