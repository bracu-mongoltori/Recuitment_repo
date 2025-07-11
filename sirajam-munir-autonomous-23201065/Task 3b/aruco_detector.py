# Import OpenCV library
import cv2

# Start the webcam (device index 0 = default camera)
cap = cv2.VideoCapture(0)

# Load a predefined ArUco dictionary
# DICT_6X6_250 means 6x6 grid markers, with 250 unique IDs
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Create detector parameters to fine-tune how the detection behaves
params = cv2.aruco.DetectorParameters()

# Below are parameter tweaks for better real-world detection performance:

# Minimum window size for adaptive thresholding (helps detect edges)
params.adaptiveThreshWinSizeMin = 3

# Maximum window size for adaptive thresholding
params.adaptiveThreshWinSizeMax = 23

# Step size used to increase window size from min to max
params.adaptiveThreshWinSizeStep = 10

# Min perimeter of the marker as a percentage of the image width
params.minMarkerPerimeterRate = 0.02

# Max perimeter of the marker to consider
params.maxMarkerPerimeterRate = 4.0

# Enable subpixel corner refinement (more precise marker corners)
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

# Create an ArUcoDetector object using the dictionary and tuned parameters
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

# Notify the user
print("Starting webcam... Press 'q' to exit.")

# Infinite loop to keep reading frames from webcam
while True:
    # Read a frame from the webcam
    ret, frame = cap.read()

    # If frame was not grabbed successfully, exit the loop
    if not ret:
        print("❌ Failed to grab frame.")
        break

    # Convert the frame to grayscale (ArUco detection doesn't need color)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers in the grayscale image
    corners, ids, _ = detector.detectMarkers(gray)

    # If any markers were detected, draw boxes around them and print their IDs
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        print("✅ Detected marker IDs:", ids.flatten())
    else:
        print("🔍 No markers found.")

    # Display the frame (with or without detection)
    cv2.imshow("Webcam ArUco Detection", frame)

    # Wait for a short time (1ms), check if 'q' is pressed OR window is closed
    key = cv2.waitKey(1)
    if key == ord('q') or cv2.getWindowProperty("Webcam ArUco Detection", cv2.WND_PROP_VISIBLE) < 1:
        print(" Exiting...")
        break

# Release the webcam resource and close the OpenCV window properly
cap.release()
cv2.destroyAllWindows()
