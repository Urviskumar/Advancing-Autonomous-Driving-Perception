# Autonomous-Driving-Perception

## Basic Overview

 <p align="justify">This project demonstrated the effective use of computer
 vision techniques, and control algorithms
in developing an autonomous navigation system. Despite the
 challenges faced, the project was successful in achieving
 its objectives and contributed to the field of autonomous
 navigation.</p>





## A. Introduction
<p align="justify">In autonomous driving, perception systems are pivotal as they
 interpret sensory data to understand the envi
 ronment, which is essential for decision-making and planning.
 Ensuring the safety of these perception systems is fundamental
 for achieving high-level autonomy, allowing us to confidently
 delegate driving and monitoring tasks to machines.  This project
 focuses on enhancing the understanding and navigation capabil
 ities of self-driving robots through sensor fusion and computervision techniques. 
 Specifically, it explores the depth based perception using ZED2 camera to improve
 autonomous driving perception.</p>

## B. Background
<p align="justify">In this project, we utilized depth-based perception to en
 able autonomous navigation of the robot in an unfamiliar
 environment. The fusion of 2D LiDAR and depth camera
 sensors demanded substantial computational resources, leading
 to system throttle errors during the object detection task
 alone. In addition to object detection, we also maneuvered the
 Rosmaster R2 bot autonomously, detecting traffic signs such
 as 'Move', 'Turn', and 'Stop'. Depth cameras and traditional
 cameras play critical roles in mobile robot perception, providing 3D environmental information and facilitating vision
 guided navigation, respectively. Below figure shows such example of
 the camera that we have used in this project. </p>

  
 ##   C. Hardware and Software Setup

<p align="center"><img src="ZED2_Camera.jpeg" alt="txt_to_img" width="700" height="500"/></a></p>
<p align="center"><img src="RosmasterR2.jpeg" alt="txt_to_img" width="700" height="500"/></a></p>


<p align="justify">The project utilized a combination of advanced hardware
 and software to process and analyze sensor data:

* Jetson Xavier Processor: Served as the computational
 backbone, handling data processing and model execution.
* ZED2 RGBD Camera:Provided high-resolution images
 and depth data, crucial for object detection and distance
 estimation. Fig 2 shows such example of the camera that
 we have used in this project.
* ROS (Robot Operating System): Enabled efficient sys
tem integration, data handling, and algorithm implemen
tation.
* ZED SDK: Offered tools and APIs for extracting and
 processing data from the ZED 2 camera.

The integration of these hardware components through ROS
 facilitated a modular approach, allowing for the independent
 development and testing of subsystems. Below figure shows the implemented environment we have used through this project.</p>

 <p align="center"><img src="our_environmnet.jpeg" alt="txt_to_img" width="700" height="500"/></a></p>


##  D. Challenges and Solution
<p align="justify">The project faced significant challenges in terms of  computational power.
 The computational capacity of the Jetson
 Xavier was limited, which posed a significant hurdle. The
 team was unable to implement 2D LiDAR fusion and the
 ZED 2 camera as initially planned due to these constraints.
 Additionally, the project demanded significant computational
 power for processing depth information from the ZED2 RGBD
 camera and running the YOLOv5 object detection model
 simultaneously to perform Autonomous Navigation. These
 computation demands added to the complexity and challenges
 of the project.<br><br>
Despite the challenges, the team managed to devise effective
 solutions and achieve significant milestones. For depth-based
 perception, the ZED 2 camera was utilized for depth and RGB
 data. The team estimated distances to objects using the depth
 map, which proved to be a valuable asset for the project. For
 autonomous navigation, the team incorporated computer vision
 techniques to detect lanes and control the robot accordingly.
 Werelied on depth data to get the distance of the object (traffic
 sign), which was crucial for the navigation system. To address
 the resource constraints, the team optimized algorithms for
 computational efficiency. We achieved real-time performance
 on Jetson Xavier by prioritizing system efficiency, which
 significantly improved the overall performance of the project.</p>


## E. Working
 <p align="justify">1) Lane Detection: In this project, we embarked on a
 journey to develop an autonomous navigation system, starting with 
 the fundamental task of detecting lanes. </p>
 <p align="justify">
 2) Object Detection: We utilized the YOLOv5 pre-trained
 model for object detection. This model has been widely used
 in various applications, including lane detection, missing road
 lane markings detection, and pedestrian detection. The use of
 YOLOv5 allowed us to effectively detect objects in real-time,
 contributing significantly to the success of the project. Here we
 have detected successfully the 'Stop', 'Move', and 'Turn' signs
 using which robot will perform the task according to the signs.</p>
 <p align="justify">
 3) Depth Estimation: The ZED 2 camera is a powerful
 stereo camera that plays a crucial role in depth estimation for
 autonomous navigation. It combines advances in AI, sensor
 hardware, and stereo vision to build an unmatched solution
 in spatial perception and understanding. The camera features
 ultra-wide depth perception with a 110-degree horizontal and
 70-degree vertical field of view, including optical distortion
 compensation. It also has enhanced low-light vision with an
 f/1.8 aperture and improved ISP, capturing 40 percent more
 light in dark environments. The ZED 2 camera uses stereo
 vision and neural networks to replicate human-like vision,
 enabling depth perception from 0.2 to 20m².  In autonomous navigation,
  this depth information is used to
 identify obstacles, plan paths, and make decisions about the
 robot's movements. </p>


 <p align="justify">
  4) PIDController:  The system utilized a PID(Proportional-Integral-Derivative)
 controller for dynamic steering adjustment.The PID controller
 adjusted the robot's steering based on the deviation from the
 desired lane position.This allowed the robot to make smooth
 and precise adjustments to its course, ensuring it stayed on
 track and navigated the lanes effectively. Throughout this
 project,we tried to improve our control as much as possible
 resulting the smooth navigation with in the environment.For lane following, a PID
 controller adjusts v and w to minimize the error between
 the robot's current position and the desired path.</p>



## F. ROS Integration
 <p align="justify">The Robot Operating System (ROS) played a pivotal role
 in the implementation of this project. ROS is a flexible
 framework for writing robot software and provides services
 designed for a heterogeneous computer cluster such as hardware abstraction,
 low-level device control, implementation of
 commonly-used functionality, message-passing between processes, and package management.</p>

 <p align="justify">For object detection, the YOLOv5 model was 
 integrated into the ROS framework. The images from the camera were passed
 to the YOLOv5 node, which detected objects of interest and
 published the detections as ROS messages. These messages
 were used to trigger specific actions by the robot.
 The ZED 2 camera was used to obtain depth information,
 which was crucial for object distance estimation. </p>


https://github.com/Urviskumar/-Autonomous-Driving-Perception/assets/98739768/7e57924c-33b4-4871-89c8-27f249627703

## Lane Following:

 <p align="justify">The lane following functionality of the robot involves a sophisticated combination of image processing techniques and control algorithms to ensure precise navigation along road lanes. Through the integration of edge detection and color filtering, the robot adeptly identifies and tracks lane markings, enabling it to maintain a stable and centered position within the lanes.</p>

 <p align="justify">Image Processing Pipeline:

## Gaussian Blur:
    The first step in the image processing pipeline involves applying a Gaussian blur to the camera input. This helps in reducing noise and smoothing out the image, preparing it for subsequent processing steps.

## Yellow Color Filtering:
    Utilizing the HSV color space, the robot isolates the yellow lane markings from the rest of the scene. By defining a specific range of yellow hues, the robot accurately filters out pixels corresponding to the lanes, enhancing the lane detection process.

## Yellow color mask:
    By identifying abrupt intensity changes in the image, edges corresponding to lane markings are extracted, providing crucial information for subsequent analysis.

## Region of Interest Masking:
    To focus solely on the relevant portion of the image, a region of interest is defined, typically covering the area of the road ahead where lane markings are expected. This helps in reducing computational overhead and improves the efficiency of lane detection.

## Lane Tracking and Control:
    Once lane markings are detected, the robot employs robust algorithms for lane tracking and control, ensuring smooth and precise navigation along the road.

## Hough Line Transform:
    Detected edges are processed using the Hough line transform to identify line segments corresponding to lane markings. These line segments are then extrapolated to form complete lane boundaries, providing a comprehensive representation of the road geometry.
\
## Lane Centering:
    By calculating the midpoint between the detected lane boundaries, the robot determines its position within the lane. Adjustments to the steering are made to ensure that the robot remains centered within the lanes, maintaining a consistent trajectory along the road.

## PID Control:
    The robot utilizes a PID controller to dynamically adjust its steering angle based on the deviation from the desired lane position. By continuously monitoring the error between the current and desired positions, precise steering corrections are applied, resulting in smooth and stable lane following behavior.

## Execution:

 <p align="justify">Through the successful execution of the lane following functionality, the robot demonstrates remarkable proficiency in autonomously navigating roadways. With its ability to accurately detect and track lane markings while maintaining a centered position within the lanes, the robot showcases a high level of autonomy and adaptability in real-world driving scenarios. This robust and reliable lane following capability ensures safe and efficient navigation, contributing to the overall effectiveness of the robotic system in diverse environments.</p>





##  Object Detection:
 <p align="justify"> This model was built by Amir. It gives us the detections and we use it further according to our needs. 
The object detection module equips the robot with the capability to identify and localize various objects of interest within its environment. Leveraging an ONNX model trained on the YOLOv5 architecture, the robot processes incoming camera images to detect objects such as "Car", "Walk", "Speed", "Stop", "Go", and "Turn" signs.</p>

## Implementation Details:

### ROS Publishers:
<p align="justify">   Two ROS publishers are initialized to broadcast information about detected objects:
            /detected_class: Publishes the name of the detected class (e.g., "Car", "Walk", "Stop").
            /detected_class_distance: Publishes the coordinates of the detected object's centroid.

### Object Detection Logic:
<p align="justify"> Upon detecting objects in the image, the system extracts relevant information, including bounding box coordinates, confidence scores, and class labels.
        The bounding box's centroid coordinates (cx, cy) are calculated to represent the object's position.
        The class label is determined based on the assigned class number (cls), and if the confidence score exceeds a predefined threshold (e.g., 0.8), the class name is published.
        The centroid coordinates and class label are packaged into ROS messages and published via the designated topics.

### Visualization:
<p align="justify">    Detected objects are visualized on the camera image for debugging and visualization purposes.
        Bounding boxes are drawn around detected objects, and labels indicating the class name and confidence score are overlayed.
        Additionally, small dots are drawn at the centroid of each detected object for reference.

###    Image Processing and Publishing:
 <p align="justify"> After processing, the annotated image with detected objects is converted back to a NumPy array and published for visualization in the ROS ecosystem.

        
## G. Results and Analysis
 <p align="justify">The robot demonstrated a high degree of proficiency in
 following lane markings detected in the camera images. It
 employed techniques such as color filtering and line fitting to
 identify lanes. The robot was able to adjust its steering to stay
 within the lanes, demonstrating the effectiveness of the control
 algorithms implemented. The success in lane following is a
 testament to the robustness of the computer vision techniques
 and control algorithms used in this project.<br><br>
The robot was capable of detecting objects of interest within
 its environment, such as ”Turn”, ”Go”, or ”Stop” signs. This
 was achieved using advanced computer vision techniques.
 The detection was used to trigger specific actions by the
 robot, adding a layer of interactivity and responsiveness to the
 system. The successful detection of objects of interest and the
 subsequent triggering of actions demonstrate the effectiveness
 of the object detection algorithms implemented.<br><br>
 The robot was able to estimate the distance to detected
 objects using depth information obtained from the camera.
 This information was crucial for making decisions about when
 to execute certain actions, such as stopping or turning. The
 ability to accurately estimate distances to objects is a critical
 component of autonomous navigation systems, as it allows the
 system to interact safely and effectively with its environment.
 The success in object distance estimation indicates the effec
tiveness of the depth perception capabilities of the system.</p>

 

## H. Conclusion
<p align="justify">In conclusion, the results obtained from this project demonstrates the effectiveness of the techniques and algorithms implemented. 
 The robot was able to follow lanes, detect objects,
 and estimate distances to objects accurately, demonstrating its
 potential for real-world applications in autonomous navigation.</p>

