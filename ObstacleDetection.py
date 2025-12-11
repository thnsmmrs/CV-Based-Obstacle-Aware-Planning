import cv2
import numpy as np
import matplotlib
from RRT import rrt
from robot_kinematics import Robot3R
from robot_kinematics import improve_obstacle_mask
import time


capture = False #creating capture variable to be used for running single frame into RRT
dispFPS = False #display FPS debug option for tuning sampling frequency using fps
#to compare runtimes
robot = Robot3R()
robot.initialize(link_lengths=[200,200,200], base_position=(10,10))
#initializing robot params *can be changed
goal = None
path = None
RRT_samples = 10 #initializing num of samples output from RRT used to pick best option
step_size = 10 #step size for distance between nodes in RRT
#tuning notes: FPS average remained same between 25-200 and only saw slight
#drop off at 10. At sample size 5 avg fps = 13 and 10 fps = 15. 25 fps ~= 16
#global vars needed for scope

def mouse_callback(event, x, y, flags, param): 
    global goal
    if event == cv2.EVENT_LBUTTONDOWN:
        goal = (x, y)
#function and RRT courtesy of https://github.com/nimRobotics/RRT/blob/master/rrt.py
cv2.namedWindow("Overlay")
cv2.setMouseCallback("Overlay", mouse_callback)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  720)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
#Locks resolution at 720p (from logi c270 - stays same for all cameras)

start = (50,50)
#define home position (can be in terms of h,w for ratios)

Threshold = 30  #Threshold identifies what is an object and what is not, 20-30 suggested range from doc but will be testable
overlayStrength = 0.3 #Strength of overlay color on detected obstacles, 0-1 range

if dispFPS:
    prev_time = time.time()
    fps = 0.0
    fpsarr = []
while True:
    ret, frame = cap.read()
    
    #Reads next frames continously
    if not ret:
        break
    blur = cv2.GaussianBlur(frame, (5,5), 0)
    color_hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(color_hsv, (0,0,50), (180,40,255))
    fg_mask = cv2.bitwise_not(white)

    #implementing improve obstacle mask function to clean up mask using morphology
    obstacle_mask = fg_mask.copy()
    obstacle_mask = improve_obstacle_mask(obstacle_mask)
    obstacle_mask = cv2.bitwise_not(fg_mask)
    overlay = frame.copy()
    overlay[obstacle_mask == 255] = (255, 255, 255)  # Highlight obstacles in red (can change color)
    vis = cv2.addWeighted(frame, overlayStrength, overlay, 1 - overlayStrength, 0)
    cv2.circle(vis, start, 3, (0,0,255), -1) #params as defined in circle function
    
    if cv2.waitKey(1) & 0xFF == 67:
        capture = True
    
    if goal != None:
        cv2.circle(vis, goal, 3, (0,0,255), -1)
    #drawing start point and goal point
    #Overlay outputs the detected obstacles on the original input frame
        if capture:
            capturedFrame = frame.copy()
            capturedOM = obstacle_mask.copy()
            for i in range(RRT_samples):
                compDist = 0
                path, dist = rrt(capturedOM, start, goal, step_size, robot)
                if dist > compDist:
                    compDist = dist
                    bestPath = path
            if bestPath is not None:
                for (x, y) in bestPath:
                    cv2.circle(vis, (x, y), 2, (255, 0, 0), -1)
                    break
    if dispFPS:
        curr_time = time.time()
        diff = curr_time - prev_time
        prev_time = curr_time

        if diff > 0:
            fps = 1.0/diff
        cv2.putText(vis,f"FPS: {fps:.1f}",(10, 30),cv2.FONT_HERSHEY_SIMPLEX,1.0,(0, 255, 0),2,cv2.LINE_AA)
        fpsarr.append(fps)
        if len(fpsarr) > 1000:
            fpsarr.pop(0) #pops first fps value if array gets too big (long runtime)

    cv2.imshow("Overlay", vis)
    #cv2.imshow("Obstacle Mask", obstacle_mask)

    
    if cv2.waitKey(1) & 0xFF == 27: #ESC to end program
        if dispFPS:
            avgfps = sum(fpsarr) / len(fpsarr)
            print(f"Average FPS: ", avgfps)
        break

cap.release()
cv2.destroyAllWindows()
