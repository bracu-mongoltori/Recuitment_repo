import cv2
import sys
import numpy as np 

def detect_aruco_markers_webcam():
   
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    
   
    parameters.adaptiveThreshConstant = 7 
    
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    
    cap = cv2.VideoCapture(0) 

    if not cap.isOpened():
        print("Error: Could not open webcam. Make sure it's connected and not in use by another application.")
        return

    print("Press 'q' to quit.")

    while True:
        
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read frame from webcam. Exiting...")
            break

       
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        
        corners, ids, _ = detector.detectMarkers(gray)

        
        if ids is not None:
            
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
       
        cv2.imshow("Live ArUco Marker Detection", frame)

    
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

   
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    
    detect_aruco_markers_webcam()