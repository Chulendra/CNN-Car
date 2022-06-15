import os
import numpy as np
import cv2
import RPi.GPIO as IO
from time import sleep
import socket, pickle

# setup socket connection
HOST = 'localhost'
PORT = 50007
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

# GPIO setup
IO.setwarnings(False)
IO.setmode(IO.BOARD)

IO.setup(11, IO.OUT)
IO.setup(13, IO.OUT)
IO.setup(15, IO.OUT)
IO.setup(40, IO.OUT)

IO.output(11, 0)  # light
IO.output(13, 0)  # rightMotor
IO.output(15, 0)  # leftMotor

steer = IO.PWM(40, 50)
leftMotor = IO.PWM(15, 50)
rightMotor = IO.PWM(13, 50)

steer.start(7.5)
leftMotor.start(0)
rightMotor.start(0)


# steering control
def setAngle(angle):
    duty = angle / 18.0 + 2
    steer.ChangeDutyCycle(duty)


# PID control
Kp = 0.4
Kd = 0.4
minSpeed = 0
baseSpeed = 55
maxSpeed = 100
last_error = 0


def PID(cx):
    global last_error
    error = cx - 80
    controlSpeed = Kp * error + Kd * (error - last_error)
    last_error = error

    leftMotorSpeed = baseSpeed + controlSpeed
    rightMotorSpeed = baseSpeed - controlSpeed

    if leftMotorSpeed < minSpeed:
        leftMotorSpeed = minSpeed

    if leftMotorSpeed > maxSpeed:
        leftMotorSpeed = maxSpeed

    if rightMotorSpeed < minSpeed:
        rightMotorSpeed = minSpeed

    if rightMotorSpeed > maxSpeed:
        rightMotorSpeed = maxSpeed

    steerAngle = 90 + 50 * (leftMotorSpeed - rightMotorSpeed) / 100
    setAngle(steerAngle)

    leftMotor.ChangeDutyCycle(leftMotorSpeed)
    rightMotor.ChangeDutyCycle(rightMotorSpeed)


def position(pos):
    pos = pickle.dumps(pos)
    s.send(pos)
    pos = s.recv(4096)
    pos = pickle.loads(pos)
    return pos
    # print('Received 10', repr(frame))


# Video Capture
video_capture = cv2.VideoCapture(-1)
video_capture.set(3, 160)
video_capture.set(4, 120)

last_cx = 0

while True:

    # Capture the frames
    ret, frame = video_capture.read()

    # Crop the image
    crop_img = frame[0:60, 0:160]

    # Convert to grayscale
    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

    # Gaussian blur
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # reshaping
    blur = blur.reshape([-1, 15, 40, 1])

    # Position from prediction
    cx = position(blur)

    if cx[0]:
        PID(last_cx)

    else:

        # [0, (15-30), (30-45), (45-60), (60-75), (75-85), (85-100), (100-115), (115-130), (130-145), 160]
        # [0, 45/2, 75/2, 105/2, 135/2, 80, 185/2, 215/2, 245/2, 275/2, 160]

        if cx[1]:
            cx = 0

        elif cx[2]:
            cx = 22

        elif cx[3]:
            cx = 37

        elif cx[4]:
            cx = 52

        elif cx[5]:
            cx = 67

        elif cx[6]:
            cx = 80

        elif cx[7]:
            cx = 92

        elif cx[8]:
            cx = 107

        elif cx[9]:
            cx = 122

        elif cx[10]:
            cx = 137

        else:
            cx = 160

        PID(cx)
        last_cx = cx

    cv2.imread('frame', blur)
    if cv2.waitKey(5) & 0XFF == 27:
        break

video_capture.release()
cv2.destroyAllWindows()

# Reset hardware
leftMotor.stop()
rightMotor.stop()
setAngle(90)
IO.output(11, 0)
sleep(0.1)
steer.stop()
