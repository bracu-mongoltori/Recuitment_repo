# Import OpenCV and os for file handling
import cv2
import os

# Define the folder to save the generated marker images
output_folder = "test_images"

# Create the folder if it doesn't exist already
os.makedirs(output_folder, exist_ok=True)

# Load the predefined dictionary (6x6 grid, 250 unique marker IDs)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Generate markers with IDs 0 through 4
for marker_id in range(5):
    # Generate the raw marker image (black squares on white background)
    # 200 = size in pixels (without border)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, 200)

    # Add a white border around the marker
    # This helps detection by preventing the marker from blending into the background
    marker_img = cv2.copyMakeBorder(
        marker_img,            # source image
        top=50, bottom=50,     # add 50 pixels to top and bottom
        left=50, right=50,     # add 50 pixels to left and right
        borderType=cv2.BORDER_CONSTANT,  # plain border
        value=255              # white color (255 in grayscale)
    )

    # Create a filename like marker_0.png, marker_1.png, ...
    filename = os.path.join(output_folder, f"marker_{marker_id}.png")

    # Save the final marker image with border
    cv2.imwrite(filename, marker_img)

    # Print confirmation message
    print(f"✅ Saved {filename}")
