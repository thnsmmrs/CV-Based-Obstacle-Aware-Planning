import cv2
import numpy as np
from RRT import rrt
from robot_kinematics import improve_obstacle_mask

goal = None
path = None
#global vars needed for scope

def mouse_callback(event, x, y, flags, param): 
    global goal
    if event == cv2.EVENT_LBUTTONDOWN:
        goal = (x, y)
#function and RRT courtesy of https://github.com/nimRobotics/RRT/blob/master/rrt.py
cv2.namedWindow("Overlay")
cv2.setMouseCallback("Overlay", mouse_callback)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
#Locks resolution at 720p (from logi c270 - stays same for all cameras)
ret,background = cap.read()
#grabs first frame and initilizes the background
background = cv2.cvtColor(background,cv2.COLOR_BGR2GRAY)
background = cv2.GaussianBlur(background,(5,5),0)
#will be used to filter background
h,w = background.shape[:2]
start = (50,50)
#define home position (can be in terms of h,w for ratios)

Threshold = 30  #Threshold identifies what is an object and what is not, 20-30 suggested range from doc but will be testable
overlayStrength = 0.3 #Strength of overlay color on detected obstacles, 0-1 range

while True:
    ret, frame = cap.read()
    
    #Reads next frames continously
    if not ret:
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    #Converts frame to grayscale and blurs it to reduce noise and improve detection accuracy
    diff = cv2.absdiff(background, gray)
    #Finds the difference between background and current frame

    _, fg_mask = cv2.threshold(diff,Threshold,255,cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    fg_mask = improve_obstacle_mask(fg_mask, 100)
    #implementing improve obstacle mask function to clean up mask using morphology
    obstacle_mask = np.zeros_like(fg_mask)
    for valid_contours in contours:
        cv2.drawContours(obstacle_mask, [valid_contours], -1, (255), thickness=-1)
    overlay = frame.copy()
    overlay[obstacle_mask == 255] = (255, 255, 255)  # Highlight obstacles in red (can change color)
    vis = cv2.addWeighted(frame, overlayStrength, overlay, 1 - overlayStrength, 0)
    cv2.circle(vis, start, 3, (0,0,255), -1) #params as defined in circle function
    if goal != None:
        cv2.circle(vis, goal, 3, (40,0,255), -1)
    #drawing start point and goal point
    #Overlay outputs the detected obstacles on the original input frame
        path = rrt(obstacle_mask, start, goal, 100)
        print(path)
    cv2.imshow("Overlay", vis)
    #print(path) #debug
    #cv2.imshow("Obstacle Mask", obstacle_mask) #improved qual

    
    if cv2.waitKey(1) & 0xFF == 27: #ESC to end program
        break

cap.release()
cv2.destroyAllWindows()
