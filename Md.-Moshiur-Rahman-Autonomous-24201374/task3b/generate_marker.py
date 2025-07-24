import cv2
import numpy as np

def generate_aruco_marker_numpy(dict_type=cv2.aruco.DICT_4X4_50, marker_id=0, marker_size=200, white_border_size=20, save_path="aruco_marker.png"):
    """
    Generates an ArUco marker image with a white border.

    Args:
        dict_type (int): The predefined ArUco dictionary type.
        marker_id (int): The ID of the marker to generate.
        marker_size (int): The size (in pixels) of the square marker itself (excluding the white border).
        white_border_size (int): The thickness (in pixels) of the white border around the marker.
        save_path (str): The file path to save the generated marker image.
    """
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)

    
    marker_core = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size, borderBits=1)


    total_image_size = marker_size + (2 * white_border_size)
    
    final_marker_img = np.ones((total_image_size, total_image_size), dtype=np.uint8) * 255


    start_x = white_border_size
    end_x = start_x + marker_size
    start_y = white_border_size
    end_y = start_y + marker_size

    
    final_marker_img[start_y:end_y, start_x:end_x] = marker_core

    
    cv2.imwrite(save_path, final_marker_img)
    print(f"Marker with white border saved as {save_path}")


generate_aruco_marker_numpy(marker_id=0) 