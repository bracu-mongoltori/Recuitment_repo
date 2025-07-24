import cv2
import numpy as np
import os

scene = np.ones((400, 600, 3), dtype=np.uint8) * 255  


marker_paths = ["input/marker_0.png", "input/marker_1.png", "input/marker_2.png"]
markers = []

for path in marker_paths:
    marker = cv2.imread(path)
    if marker is None:
        print(f" Marker not found: {path}")
        exit()
    marker = cv2.resize(marker, (100, 100))  
    markers.append(marker)


scene[50:150, 50:150] = markers[0]    
scene[200:300, 100:200] = markers[1]  
scene[100:200, 400:500] = markers[2]  


os.makedirs("input", exist_ok=True)
cv2.imwrite("input/scene_with_multiple_markers.png", scene)
print(" Composed scene saved as 'input/scene_with_multiple_markers.png'")
