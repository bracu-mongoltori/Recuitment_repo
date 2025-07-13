import cv2
import sys

def detect_aruco_markers(image_path):

    image = cv2.imread(image_path)
    if image is None:
        print(f"Image not found: {image_path}")
        return

    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        print(f"Detected marker IDs: {ids.flatten()}")
        
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
    else:
        print("No markers detected")


    cv2.imshow("Detected ArUco Markers", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_aruco_markers(sys.argv[1])
