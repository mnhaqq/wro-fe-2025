#!./env/bin/python3
import cv2
from picamera2 import Picamera2
from ArduinoComms import ArduinoComms
from pid import PIDController
import numpy as np
from masks import rOrange, rBlack, rBlue
import time
from time import sleep

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

def main():
    time.sleep(3)
    
    picam2 = Picamera2()

    sensor_modes = picam2.sensor_modes

    selected_mode = None
    for mode in sensor_modes:
        if mode['size'] == (2304, 1296) or mode['size'] == (1920, 1080) or mode['size'] == (1280, 720):
            selected_mode = mode
            break
    if selected_mode is None:
        selected_mode = sensor_modes[0] 

    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (1280, 720)}, 
        sensor={"output_size": selected_mode['size'], "bit_depth": selected_mode['bit_depth']}
    )
    
    picam2.configure(config)
    picam2.start()
    time.sleep(1)

    #lists storing coordinates for the regions of interest to find contours of the lanes and the orange line 
    # order: x1, y1, x2, y2
    ROI1 = [10, 420, 200, 500]
    ROI2 = [1080, 420, 1270, 500] # 380, 600 | 400, 620
    ROI3 = [540, 420, 740, 500]

    arduino = ArduinoComms()
    arduino.connect("/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0", 9600, 1000)

    #booleans for tracking whether car is in a left or right turn
    lTurn = False
    rTurn = False
  
    t = 0 #number of turns car has completed
    
    kp = 0.008 #value of proportional for proportional steering
    kd = 0.001 #value of derivative for proportional and derivative sterring
    
    pid = PIDController(kp, 0, kd)
    
    straightConst = 90 #angle in which car goes straight

    turnThresh = 150 #if area of a lane is under this threshold car goes into a turn
    exitThresh = 1500 #if area of both lanes is over this threshold car exits a turn
  
    angle = 90 #variable for the current angle of the car
    prevAngle = angle #variable tracking the angle of the previous iteration
    tDeviation = 45 #value used to calculate the how far left and right the car turns during a turn
    sharpRight = straightConst - tDeviation #the default angle sent to the car during a right turn
    sharpLeft = straightConst + tDeviation #the default angle sent to the car during a left turn
    
    #maximum limit for angles
    maxRight = straightConst - 60
    maxLeft = straightConst + 60
    
    speed = 210 #variable for the speed of the car, 1665
    
    aDiff = 0 #value storing the difference of area between contours
    prevDiff = 0 #value storing the previous difference of contours for derivative steering
    
    arduino.set_drive_motor_value(0)

    sleep(2) #delay 8 seconds for the servo to be ready

    #boolean tracking whether the orange line on the mat is detected
    lDetected = False
    
    #boolean tracking whether we are in debug mode or not
    debug = False
     
    start = False
    turnDir = "none"

    while True:
        rightArea, leftArea = 0, 0

        #get an image from pi camera
        img = picam2.capture_array()
        
        # convert from BGR to LAB
        img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
        
        #blur image
        img_lab = cv2.GaussianBlur(img_lab, (7, 7), 0)
        
        #find contours of walls and orange and blue lines
        cListLeft = find_contours(img_lab, rBlack, ROI1)
        cListRight = find_contours(img_lab, rBlack, ROI2)
        cListOrange = find_contours(img_lab, rOrange, ROI3)
        cListBlue = find_contours(img_lab, rBlue, ROI3)
        
        #find areas of walls
        leftArea = max_contour(cListLeft, ROI1)[0]
        rightArea = max_contour(cListRight, ROI2)[0]
        
        #indicate if orange line or blue line is detected and set turn direction accordingly
        if max_contour(cListOrange, ROI3)[0] > 100: 
            lDetected = True
            if turnDir == "none":
                turnDir = "right"
        elif max_contour(cListBlue, ROI3)[0] > 100:
            lDeteted = True
            if turnDir == "none":
                turnDir = "left"
        
        #draw contours
        cv2.drawContours(img[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], cListOrange, -1, (0, 255, 0), 2)
        cv2.drawContours(img[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], cListLeft, -1, (0, 255, 0), 2)
        cv2.drawContours(img[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]], cListRight, -1, (0, 255, 0), 2)
        
        #calculate difference of areas between the areas of the lanes
        aDiff = rightArea - leftArea

        #calculate angle using PD steering
        #angle = int(max(straightConst + aDiff * kp + (aDiff - prevDiff) * kd, 0))
        angle = int(max(straightConst + pid.compute(aDiff), 0))

        #if the area of either lane is less than or equal to turnThresh and the car is not in a turn going the other direction, set the boolean of the respective direction turn to true
        if leftArea <= turnThresh and not rTurn:

            lTurn = True

        elif rightArea <= turnThresh and not lTurn:

            rTurn = True
            
        if not start:
            #write intial values to car
            arduino.set_steering_motor_value(90)
            arduino.set_drive_motor_value(speed)
            start = True
            
        arduino.set_drive_motor_value(speed)
        
        #if angle is different from previous angle
        if angle != prevAngle:
            #if car is in a left or right turn
            if lTurn or rTurn: 

              #if the area of the lane the car is turning towards is greater than or equal to exitThresh, the turn is completed and the booleans are set to false and the number of turns is increased by 1
              if (rightArea > exitThresh and rTurn) or (leftArea > exitThresh and lTurn): 
                  #set turn variables to false as turn is over
                  lTurn = False 
                  rTurn = False
                  
                  #reset prevDiff
                  #prevDiff = 0 
                  pid.prev_error = 0         
                  #increase number of turns by 1 only if the orange line has been detected 
                  if lDetected: 
                      t += 1
                        
                      lDetected = False

              #set the angle to the default angle for turns if in a turn, 
              #if the calculated angle is sharper than the default angle and is within the limits of maxLeft and maxRight, use that angle instead
              elif lTurn:
                  angle = min(max(angle, sharpLeft), maxLeft)
              elif rTurn: 
                  angle = max(min(angle, sharpRight), maxRight)

              #write angle to servo motor
              arduino.set_steering_motor_value(90)
              time.sleep(0.05)
              arduino.set_steering_motor_value(angle)
              time.sleep(0.01)
            #if not in a turn write the angle and if the angle is over sharpLeft or sharpRight values it will be rounded down to those values
            else:
                arduino.set_steering_motor_value(max(min(angle, sharpLeft), sharpRight))
                time.sleep(0.01)
          
        #update previous area difference
        #prevDiff = aDiff
        pid.prev_error = aDiff
        
        prevAngle = angle #update previous angle
        
        #stop car once car is straight and 12 turns have been performed
        if t == 12 and abs(angle - straightConst) <= 10:
            
            #change delay based on turn direction
            if turnDir == "left": 
                sleep(1)
            else:
                sleep(1.5)
                
            arduino.set_drive_motor_value(0) 
            break

        #debug mode
        if debug: 
            
            #stop the car and end the program if either q is pressed or the car has done 3 laps (12 turns) and is mostly straight (within 15 degrees)
            if cv2.waitKey(1)==ord('q'):
                arduino.set_drive_motor_value(0) 
                break
          
            #display regions of interest
            img = display_roi(img, [ROI1, ROI2, ROI3], (255, 204, 0))

            #show image
            cv2.imshow("finalColor", img)
            
            #cv2.imshow("walls", imgThresh)
            
            #display variables for debugging
            variables = {
                "left wall area": leftArea,
                "right wall area": rightArea,
                "left turn": lTurn,
                "right turn": rTurn,
                "# turns": t,
                "lDetected": lDetected

            }

            display_variables(variables)
              
    if debug: 
        #close all image windows
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
