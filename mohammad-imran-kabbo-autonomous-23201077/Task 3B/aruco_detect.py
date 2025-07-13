import cv2
import os

# Load ArUco dictionary and detector (once only)
dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dict, params)

# Detect markers in a given image path
def detect_on_image(img_path):
    img = cv2.imread(img_path)
    if img is None:
        print(f"⚠️ Could not read {img_path}")
        return

    img = cv2.resize(img, None, fx=2.0, fy=2.0)  # Upscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)  # Improve contrast

    corners, ids, _ = detector.detectMarkers(gray)
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
        print(f"✅ {len(ids)} marker(s) detected in {img_path}")
        cv2.imwrite(f"detected_{os.path.basename(img_path)}", img)
    else:
        print(f"❌ No markers found in {img_path}")
        
# Detect markers using webcam
def detect_on_webcam():
    cap = cv2.VideoCapture(0)  # Open webcam
    print("🎥 Press 'q' to quit")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imshow("Webcam ArUco Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

# Run detection on 4 images, then webcam (optional)
if __name__ == "__main__":
    # If you want to automatically detect all .jpg files safely:
    image_dir = "images"
    image_files = [f for f in os.listdir(image_dir) if f.endswith(".jpg")]
    image_files = sorted(image_files)[:5]  # Only take the first 5
    # Detect from images in 'images/' folder
    for fname in image_files:
        detect_on_image(os.path.join(image_dir, fname))
        cv2.destroyAllWindows()

    # Ask user if webcam should be used
    run_webcam = input("Run webcam detection? (y/n): ").strip().lower()
    if run_webcam == 'y':
        detect_on_webcam()
