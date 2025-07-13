import cv2
import numpy as np
from masks import rOrange, rBlack, rBlue, rMagenta, rGreen, rRed


def display_roi(img, ROIs, color):
    for ROI in ROIs: 
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[0], ROI[3]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[0], ROI[3]), color, 4)
    
    return img

def find_contours(img_lab, lab_range, ROI):
    
    #segment image to only be the ROI
    img_segmented = img_lab[ROI[1]:ROI[3], ROI[0]:ROI[2]]
    
    lower_mask = np.array(lab_range[0])
    upper_mask = np.array(lab_range[1])

    #threshold image
    mask = cv2.inRange(img_segmented, lower_mask, upper_mask)
    
    kernel = np.ones((5, 5), np.uint8)
    
    #perform erosion and dilation
    eMask = cv2.erode(mask, kernel, iterations=1)
    dMask = cv2.dilate(eMask, kernel, iterations=1)
    
    #find contours
    contours = cv2.findContours(dMask, cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    return contours

def max_contour(contours, ROI): 
    maxArea = 0
    maxY = 0
    maxX = 0
    mCnt = 0
    
    for cnt in contours:
        
        area = cv2.contourArea(cnt)
        
        if area > 100: 
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)

            x += ROI[0] + w // 2
            y += ROI[1] + h
            
            if area > maxArea:
                maxArea = area
                maxY = y
                maxX = x
                mCnt = cnt

    return [maxArea, maxX, maxY, mCnt]

def display_variables(variables): 

    names = list(variables.keys())

    for i in range(len(names)):
        name = names[i]
        value = variables[name]
        # Print each item on a new line
        print(f"{name}: {value}", end="\r\n")
    
    # Move the cursor up to overwrite the previous lines
    print("\033[F" * len(names), end="")

def pOverlap(img_lab, ROI, add=False):
    
    lower_mask = np.array(rBlack[0])
    upper_mask = np.array(rBlack[1])
        
    mask = cv2.inRange(img_lab[ROI[1]:ROI[3], ROI[0]:ROI[2]], lower_mask, upper_mask)

    lower_mask2 = np.array(rMagenta[0])
    upper_mask2 = np.array(rMagenta[1])
        
    mask2 = cv2.inRange(img_lab[ROI[1]:ROI[3], ROI[0]:ROI[2]], lower_mask2, upper_mask2)
        
    if not add: 
        mask = cv2.subtract(mask, cv2.bitwise_and(mask, mask2))
    else:
        mask = cv2.add(mask, mask2)

    kernel = np.ones((5, 5), np.uint8)

    eMask = cv2.erode(mask, kernel, iterations=1)

    contours = cv2.findContours(eMask, cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]

    return contours
