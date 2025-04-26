from controller import Robot, Camera, Motor
import cv2
import numpy as np


robot = Robot()
timestep = int(robot.getBasicTimeStep())


camera = robot.getDevice('camera')
camera.enable(timestep)


translation_field = robot.getSelf().getField("translation")


def move_left():
    pos = translation_field.getSFVec3f()
    if pos[0] > -0.9:
        translation_field.setSFVec3f([pos[0] - 0.02, pos[1], pos[2]])

def move_right():
    pos = translation_field.getSFVec3f()
    if pos[0] < 0.9:
        translation_field.setSFVec3f([pos[0] + 0.02, pos[1], pos[2]])


def detect_ball(image):
    img = np.frombuffer(image, np.uint8).reshape((480, 640, 4))
    bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)


    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Find ball contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    ball_x = None
    if contours and len(contours) > 0:
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            ball_x = cx
    return ball_x


while robot.step(timestep) != -1:
    image = camera.getImage()
    ball_x = detect_ball(image)

    if ball_x is not None:
        center_x = 320  # Middle of the image
        tolerance = 40  # Pixels
        
        if ball_x < center_x - tolerance:
            print("Ball left, moving left")
            move_left()
        elif ball_x > center_x + tolerance:
            print("Ball right, moving right")
            move_right()
        else:
            print("Ball centered, staying put")
    else:
        print("No ball detected, staying put")
