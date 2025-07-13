import cv2  # Import OpenCV for marker generation
import os   # Import OS for folder and file operations

# Create and save a set of ArUco markers into the 'markers/' directory
def create_aruco_marker_set(marker_count=10, marker_size=200):
    os.makedirs("markers", exist_ok=True)  # Create the output folder if it doesn't exist

    # Use the 6x6 ArUco dictionary with 250 unique IDs
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    # Get the maximum number of marker IDs available in the dictionary
    max_ids = aruco_dict.bytesList.shape[0]

    # Ensure the requested marker count doesn't exceed the dictionary's capacity
    if marker_count > max_ids:
        print(f"⚠️ Requested {marker_count} markers, but only {max_ids} available.")
        marker_count = max_ids  # Limit to max available

    # Generate markers with IDs from 0 to (marker_count - 1)
    for marker_id in range(marker_count):
        img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)  # Create marker
        cv2.imwrite(f"markers/marker_{marker_id}.png", img)  # Save image

    print(f"✅ {marker_count} marker(s) saved in 'markers/' folder")

# Run the function only when the script is executed directly
if __name__ == "__main__":
    create_aruco_marker_set(marker_count=25)  # You can change this number freely
