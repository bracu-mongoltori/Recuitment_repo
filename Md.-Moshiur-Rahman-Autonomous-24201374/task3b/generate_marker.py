import cv2
import numpy as np

def generate_aruco_marker_numpy(dict_type=cv2.aruco.DICT_4X4_50, marker_id=0, marker_size=200, save_path="aruco_marker.png"):
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)

    
    bits = aruco_dict.bytesList[marker_id]
    marker_img = np.zeros((marker_size, marker_size), dtype=np.uint8)
    cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size, marker_img, 1)


    cv2.imwrite(save_path, marker_img)
    print(f"Marker saved as {save_path}")

generate_aruco_marker_numpy()
