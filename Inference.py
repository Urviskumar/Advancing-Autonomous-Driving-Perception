#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Point
import time


rospy.init_node('camera_processor')

#center_offset = 0

# initialize the CvBridge object
bridge = CvBridge()

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
twist = Twist()

detected_class = None
cx = 0
cy = 0
s = 0
depth = 0

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Setpoint value

        self.error = 0  # Current error
        self.last_error = 0  # Previous error
        self.integral = 0  # Accumulated error

    def update(self, measured_value):
        # Calculate the error
        self.error = self.setpoint - measured_value

        # Calculate the proportional term
        proportional_term = self.Kp * self.error

        # Calculate the integral term
        self.integral += self.error
        integral_term = self.Ki * self.integral

        # Calculate the derivative term
        derivative_term = self.Kd * (self.error - self.last_error)
        self.last_error = self.error

        # Calculate the control output
        control_output = proportional_term + integral_term + derivative_term

        return control_output

def birdseye_view(image):
    # Define the source and destination points for the perspective transformation
    height, width = image.shape[:2]
    #src_points = np.float32([(0, height), (width, height), (width // 2 + 50, height // 2 + 100), (width // 2 - 50, height // 2 + 100)])
    #dst_points = np.float32([(0, height), (width, height), (width, 0), (0, 0)])

    src_points = np.float32([(185, 200), (width-185, 200), (width, height-60), (0, height-60)])
    dst_points = np.float32([(100, 100), (width-100, 100), (width-100, height), (100, height)])

    # Compute the perspective transformation matrix
    M = cv2.getPerspectiveTransform(src_points, dst_points)

    # Apply the perspective transformation to the image
    warped_image = cv2.warpPerspective(image, M, (width, height), flags=cv2.INTER_LINEAR)

    return warped_image

def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1/2)
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return np.array([x1, y1, x2, y2])

def average_slope_intercept(image, lines):
    left_fit = []
    right_fit = []
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1,x2), (y1, y2), 1)
        slope = parameters[0]
        intercept = parameters[1]
        if slope < 0:
            left_fit.append((slope, intercept))
        else:
            right_fit.append((slope, intercept))
    left_fit_average = np.average(left_fit, axis=0)
    right_fit_average = np.average(right_fit, axis=0)
    left_line = make_coordinates(image, left_fit_average)
    right_line = make_coordinates(image, right_fit_average)
    return np.array([left_line, right_line])

def canny(image):
    #gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(image, (5,5), 0)
    #lower_yellow = np.array([2, 200, 10])  # Lower HSV values for yellow
    #upper_yellow = np.array([100, 255, 255])
    lower_yellow = np.array([2, 180, 100])  # Adjusted lower HSV values for yellow
    upper_yellow = np.array([40, 255, 255])
    canny = cv2.inRange(blur, lower_yellow, upper_yellow)
    #gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    #blur = cv2.GaussianBlur(gray, (5,5), 0)
    #canny = cv2.Canny(blur, 50, 150)
    return canny

def display_lines(image, lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1,y1, x2, y2 = line.reshape(4)
            cv2.line(line_image, (x1,y1), (x2,y2), (255,0,0), 10)
            #cv2.circle(line_image, (int(float((x2-x1)/2)),int(float(y1/1.2))), 5, (0,0,255),-1)
    return line_image

def region_of_interest(image):
    height = image.shape[0]
    width = image.shape[1]
    polygons = np.array([
        #[(230,height-230), (width-250, height-230),(width,height-148), (width,height), (0,height),(0,height)]
        [(250,300), (450, 300),(640,380),(640,640), (0,640)]
        ])
    #polygons = np.array([
    #    [(0,height-20), (width, height-20), (width,(height/2)+20), (0,(height/2)+20)]
    #    ])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

def detected_class_callback(msg):
    global detected_class
    detected_class = msg.data


def detected_class_distance_callback(msg):
    global cx,cy
    cx = int(msg.x)
    cy = int(msg.y)

def depth_callback(msg):
    global cx, cy, dist
    
        # Convert ROS Image message to OpenCV image
    depths = np.frombuffer(msg.data, dtype=np.float32)

        # Image coordinates of the center pixel
    u = cx
    v = cy

        # Linear index of the center pixel
    center_idx = u + 640 * v
    dist = depths[center_idx]

        # Output the measure
        

# define a function to process the camera data
def process_image(image):
    global detected_class,cx,cy,dist
    img = bridge.imgmsg_to_cv2(image, "bgr8")
    img_resized = cv2.resize(img, (640, 640))
    lane_image = np.copy(img_resized)
    apple = canny(lane_image)
    cropped_image = region_of_interest(apple)
    lines = cv2.HoughLinesP(cropped_image, 5, np.pi/4/180, 180, np.array([]), minLineLength=5, maxLineGap=5)
    averaged_lines = average_slope_intercept(lane_image, lines)
    line_image = display_lines(lane_image, averaged_lines)

    birdseye_image = birdseye_view(line_image)
    
    lane_midpoint = int((averaged_lines[0][0] + averaged_lines[1][0]) / 2)
    lane_midpoint2 = int((averaged_lines[0][2] + averaged_lines[1][2]) / 2)
    cv2.line(line_image, (lane_midpoint, lane_image.shape[0]), (lane_midpoint2, lane_image.shape[0] // 2), (255, 0, 0), 10)

    center_offset = lane_image.shape[1] // 2 - lane_midpoint+150
    #print(center_offset)
    cv2.circle(line_image, (lane_midpoint-150, int(float(lane_image.shape[0] // 1.5))), 5, (0, 0, 255), -1)

    pid = PIDController(0.001,0.0001,0.002,0)
    control_output = pid.update(center_offset+20)
    #print(control_output)
    #angular_gain = 0.2
    #twist.linear.x = 0.1
    #twist.angular.z = -angular_gain * control_output
    if detected_class == "Turn" and dist<1.5:
        
        
        twist.linear.x = 0.35
        twist.angular.z = 0
        start_time = time.time()
        while time.time() - start_time < 1.19:
            cmd_vel_pub.publish(twist)

        # Stop the robot for 1 second
        twist.linear.x = 0.0
        cmd_vel_pub.publish(twist)
        

        # Set angular velocity for turning right
        twist.angular.z = -0.9  # rad/s
        twist.linear.x=0.09

        # Publish Twist messages for 3 seconds to turn right
        start_time = time.time()
        while time.time() - start_time < 2.5:
            cmd_vel_pub.publish(twist)
        rospy.loginfo("Detected Turn!")
        #cmd_vel_pub.publish(twist)
        #print(cx,cy,s)
        print(detected_class)
        
        
    elif detected_class == "Go" and dist<1.45:
        rospy.sleep(0.5)

        twist.linear.x = 0.3
        twist.angular.z = 0
        start_time = time.time()
        while time.time() - start_time < 2.1:
            cmd_vel_pub.publish(twist)

        # Stop the robot for 1 second
        twist.linear.x = 0.0
        cmd_vel_pub.publish(twist)
        rospy.sleep(0.5)

    elif detected_class == "Stop" and dist<1.45:
        twist.linear.x = 0.18
        twist.angular.z = 0
        start_time = time.time()
        while time.time() - start_time < 1.0:
            cmd_vel_pub.publish(twist)

        # Stop the robot for 1 second
        twist.linear.x = 0.0
        twist.angular.z = 0
        cmd_vel_pub.publish(twist)
        rospy.sleep(5)

    else:
        angular_gain = 0.2
        twist.linear.x = 0.1
        twist.angular.z = -angular_gain * control_output
        cmd_vel_pub.publish(twist)
        rospy.loginfo("Keeping straight")

    

    #cmd_vel_pub.publish(twist)
    height, width = line_image.shape[:2]
    combo_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)
    birdseye_image = birdseye_view(combo_image)

    cv2.imshow('result', combo_image)
    cv2.imshow('birdseye_view', birdseye_image)
 
    
    cv2.waitKey(1)






#detected_class_subscriber()
# Subscribe to the topic for detected class
rospy.Subscriber('/detected_class', String, detected_class_callback)
rospy.Subscriber('/detected_class_distance', Point, detected_class_distance_callback)
rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, depth_callback)




#rospy.Subscriber('linear_speed', Float64, robot_controller.linear_speed_callback)
#rospy.Subscriber('angular_speed', Float64, robot_controller.angular_speed_callback)

# subscribe to the camera topic
rospy.Subscriber('/zed2/zed_node/left_raw/image_raw_color', Image, process_image)

# spin the node to process incoming messages
rospy.spin()
