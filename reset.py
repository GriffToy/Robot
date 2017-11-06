import RPi.GPIO as GPIO
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

GPIO.cleanup()
