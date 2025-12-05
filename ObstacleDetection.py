import cv2
import numpy as np

cap = cv2.VideoCapture(0)
ret,background = cap.read()
#grabs first frame and initilizes the background
background = cv2.cvtColor(background,cv2.COLOR_BGR2GRAY)
background = cv2.GaussianBlur(background,(5,5),0)
#will be used to filter background

Threshold = 25  #Threshold identifies what is an object and what is not, 20-30 suggested range but will be testable
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
    
    obstacle_mask = np.zeros_like(fg_mask)
    for valid_contours in contours:
        cv2.drawContours(obstacle_mask, [valid_contours], -1, (255), thickness=-1)
    overlay = frame.copy()
    overlay[obstacle_mask == 255] = (255, 255, 255)  # Highlight obstacles in red (can change color)
    vis = cv2.addWeighted(frame, overlayStrength, overlay, 1 - overlayStrength, 0)

    cv2.imshow("Raw", frame)
    cv2.imshow("Overlay", vis)
    #Overlay outputs the detected obstacles on the original input frame

    if cv2.waitKey(1) & 0xFF == 27: #Press ESC to quit
        break

cap.release()
cv2.destroyAllWindows()
