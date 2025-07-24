import cv2
import numpy as np
import os


os.makedirs("input", exist_ok=True)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)


marker_size = 200   
full_size = 300    

for id in range(3):  
  
    marker = cv2.aruco.generateImageMarker(aruco_dict, id, marker_size)

    
    image = 255 * np.ones((full_size, full_size), dtype=np.uint8)

   
    margin = (full_size - marker_size) // 2
    image[margin:margin + marker_size, margin:margin + marker_size] = marker

    
    cv2.imwrite(f"input/marker_{id}.png", image)

print(" Generated markers saved in 'input/' folder.")
