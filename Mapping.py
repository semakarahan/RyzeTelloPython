from djitellopy import tello
import KeyPressModule as kp
import numpy as np
import cv2
import math
from time import sleep
#######  PARAMETERS ################
fSpeed = 117/10 # forward speed cm/s
aSpeed = 360/10  # Angular speed Degrees/s
interval = 0.25

dinterval = fSpeed*interval
aInterval = aSpeed*interval
####################################


x,y = 500,500
a = 0
yaw = 0

kp.init()
me = tello.Tello()
me.connect()
print(me.get_battery())
points = []

def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 15
    aspeed = 50
    global yaw,x,y,a
    d=0
    if kp.getKey("LEFT"):
        lr = -speed
        d = dinterval
        a = -180

    elif kp.getKey("RIGHT"):
        lr = speed
        d = -dinterval
        a = 180

    if kp.getKey("UP"):
        fb = speed
        d = dinterval
        a = 270

    elif kp.getKey("DOWN"):
        fb = -speed
        d = -dinterval
        a = -90

    if kp.getKey("w"): ud = speed
    elif kp.getKey("s"): ud = -speed

    if kp.getKey("a"):
        yv = -aspeed
        yaw -=aInterval

    elif kp.getKey("d"):
        yv = aspeed
        yaw +=aInterval

    if kp.getKey("q"): me.land()
    if kp.getKey("e"): me.takeoff()

    sleep(interval)
    a += yaw
    x += int(d*math.cos(math.radians(a)))
    y += int(d*math.sin(math.radians(a)))

    return [lr, fb, ud, yv, x, y]

def drawPoints(img,points):
    for point in points:
        cv2.circle(img,(point[0],point[1]),5,(0,0,255),cv2.FILLED)
    cv2.circle(img,(points[-1]),8,(0,255,0),cv2.FILLED)

    cv2.putText(img,f'({(points[-1][0]-500)/100},{(points[-1][1]-500)/100})m',
                (points[-1][0]+10,(points[-1][1]+30)),cv2.FONT_HERSHEY_SIMPLEX,1,
                (255,0,255),1)

me.takeoff()

while True:
    vals = getKeyboardInput()
    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])

    img = np.zeros((1000,1000,3), np.uint8)
    points.append((vals[4],vals[5]))
    drawPoints(img,points)
    cv2.imshow("Output",img)
    cv2.waitKey(1)