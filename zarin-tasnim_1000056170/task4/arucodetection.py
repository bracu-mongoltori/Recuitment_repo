import cv2
import os

def detect_markers(image_path, output_path):
    image = cv2.imread(image_path)
    if image is None:
        print(f" Failed to load: {image_path}")
        return


    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)


    parameters = cv2.aruco.DetectorParameters()


    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)


    corners, ids, _ = detector.detectMarkers(gray)


    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        print(f" Detected {len(ids)} marker(s) in '{os.path.basename(image_path)}': {ids.flatten().tolist()}")
    else:
        print(f" No markers found in '{os.path.basename(image_path)}'.")

    cv2.imwrite(output_path, image)


if __name__ == "__main__":
    input_dir = "input"
    output_dir = "output"
    os.makedirs(output_dir, exist_ok=True)


    for filename in os.listdir(input_dir):
        if filename.endswith(".jpg") or filename.endswith(".png"):
            input_path = os.path.join(input_dir, filename)
            output_path = os.path.join(output_dir, f"detected_{filename}")
            detect_markers(input_path, output_path)

    print(" Detection complete. Results saved in 'output/' folder.")
