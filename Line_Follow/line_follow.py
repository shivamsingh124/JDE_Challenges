from GUI import GUI
from HAL import HAL
import numpy as np
import cv2
# Enter sequential code!

def maskImage(img):
    mask = np.zeros(img.shape[:2], dtype="uint8")
    cv2.rectangle(mask, (10,30), (100, 150), 255, -1)
    masked = cv2.bitwise_and(img, img,mask=mask)
    return masked

while True:
    # Enter iterative code!
    img = HAL.getImage()
    masked = maskImage(img)
    GUI.showImage(masked)
    
    
