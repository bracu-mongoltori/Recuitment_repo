how it works: 
1. we generate markers in generate_markers.py  using OpenCV's cv2.aruco.generateImageMarker() function. Each marker has it's unique id and stored as an image in input 

2.  in compose_scene.py we make a white canvas which will help simulate an environment. Previously genereated images are also here to mimic real life scenes with multiple markers

3. we detect the markers by importing the images from the input directory and convert it to grayscale and then detect the markers using aruco detection. then it is stored in the output folder

why efficient:
-- we dont need data training or heavy GPU processing 
-- therefore it works quickly and accurately on CPUS
