from GUI import GUI
from HAL import HAL
import cv2
import numpy as np
import time

# Enter sequential code!
# kp = 0.036
# ki = 0
# kd = 0.0633

kp = 0.03
ki = 0
kd = 0.057

prev_err = 0
area = 0
speed = 3

def calculateAngle(err):
    global prev_err, area

    # Add the error to the area to simulate integration
    area += err
    # Get the difference to obtain the rate of change of error w.r.t time
    rate_of_change = err - prev_err

    p_term = kp * err
    d_term = kd * rate_of_change
    i_term = ki * area
    
    angle = p_term + d_term + i_term

    prev_err = err

    # Rudimentary bounding function for the angle values
    if angle > 80:
        angle = 80
    elif angle < -80:
        angle = -80
        
    return angle 

def maskImage(img, result):
    # Because the Red colour range in HSV is a circle
    # We have to define two ranges and add them for the most accurate colour detection

    # HSV RED Range Lower (0 - 10)
    lower1 = np.array([0, 100, 20])
    upper1 = np.array([10, 255, 255])
    
    # HSV RED Range Upper (160 - 179)
    lower2 = np.array([160,100,20])
    upper2 = np.array([179,255,255])
    
    lower_mask = cv2.inRange(img, lower1, upper1)
    upper_mask = cv2.inRange(img, lower2, upper2)
    
    # Add the two to make the complete mask
    full_mask = lower_mask + upper_mask
    
    # Actual 'and' part to get the image with just the red line
    masked = cv2.bitwise_and(result, result, mask=full_mask)

    return (masked, full_mask)

while True:
    img = HAL.getImage()
    result = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    (masked, mask) = maskImage(hsv, result)
    M = cv2.moments(mask)
    # M['m00'] is sometimes 0 and can cause a divide by 0 error
    if M['m00'] > 0:
        c_x = int(M["m10"]/M["m00"])
        c_y = int(M["m01"]/M["m00"])
        cv2.circle(img, (c_x, c_y), 5, (0, 200, 0), -1)
        
        GUI.showImage(img)
        # Difference of X position with respect to the center of the image
        err = masked.shape[1]/2 - c_x  
        # +ve implies turn left
        # -ve implies turn right
        
        #PID Function, returns the angle required
        angle = calculateAngle(err) / 10

        HAL.setV(speed)
        # Don't give the car any correcting angle if the car is straight
        if (not (err < 5 and err > -5)):
            HAL.setW(angle)

    time.sleep(10 / 1000) # Delay of 10ms
