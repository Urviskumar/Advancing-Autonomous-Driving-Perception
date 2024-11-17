#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import cv2
import torch
import onnxruntime
from PIL import Image as PILImage
from PIL import ImageDraw
import time
import sys
import os
sys.path.insert(0,"/home/jetson/zed2_recording_for_yolo/yolov5")
from utils.general import non_max_suppression



class ZEDImageSubscriber:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/zed2/zed_node/left/image_rect_color", Image, self.image_callback)
        self.latest_image = None
        
    def image_callback(self, data):
        try:
            # Convert ROS image data to NumPy array
            im = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
            self.latest_image = im[:, :, [2, 1, 0]]
        except Exception as e:
            print(e)

class ObjectDetector:
    def __init__(self, model_path):
        self.session = onnxruntime.InferenceSession(model_path)
        # Define the stop publisher
        #self.stop_publisher = rospy.Publisher('/stopsign', Bool, queue_size=1)
        self.class_publisher = rospy.Publisher('/detected_class', String, queue_size=100)
        self.distance_publisher = rospy.Publisher('/detected_class_distance', Point, queue_size=100)

    def detect_objects(self, image, frame_number):
        # Convert NumPy image array to PIL format
        im_pil = PILImage.fromarray(image)
        
        # Preprocess image
        im_resized = im_pil.resize((640, 640))
        input_data = np.array(im_resized).astype(np.float32) / 255.0  # Convert to float and normalize
        input_data = np.transpose(input_data, (2, 0, 1))  # Channel-first format
        # Add batch dimension
        input_data = np.expand_dims(input_data, axis=0)
        start_time = time.time()
        # Run inference
        results = self.session.run(None, {'images': input_data})
        end_time = time.time()
        print("Time taken for detection: {:.4f} seconds".format(end_time - start_time))
        
        # Post-process detections
        detections = self.post_process(results)
        
        # Visualize detections
        im_with_boxes = self.visualize_detections(im_resized, detections, frame_number)
        return np.array(im_with_boxes)[:, :, [2, 1, 0]]
    
    def post_process(self, results):
        # Post-process YOLOv5 output
        # ... Perform necessary reshaping and non-maximum suppression here ...
        # Return list of detections, each detection is a list [x1, y1, x2, y2, conf, cls]
        detections = results[0]
        detections = non_max_suppression(torch.Tensor(detections), conf_thres=0.5, iou_thres=0.4)[0]
        return detections.tolist()
    
    def visualize_detections(self, image, detections, frame_number):
        
        # Visualize detections on the image
        im_draw = image.copy()
        draw = ImageDraw.Draw(im_draw)
        detected_class = ""
        detected_class_distance = 0.0

        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            cls = int(cls)
            
            color = (255, 0, 0)  # RGB color (red in this case)

            # Draw the dot
            


            # Calculate area
            #s = (x2 - x1) * (y2 - y1)
            cx = x1 +(x2-x1)/2
            cy = y1 + (y2-y1)/2
            c = (cx, cy)
            

            # Assign class name based on class number
            class_name = None  # Default for unknown classes
            if cls == 0:
                class_name = 'Car'
            elif cls == 1:
                class_name = 'Walk'
            elif cls == 2:
                    class_name = 'Speed'
            elif cls == 3:
                class_name = 'Stop'
            elif cls == 4:
                class_name = 'Go'
            elif cls == 5:
                class_name = 'Turn'
            else:
                class_name = None

            # Check area threshold and publish if conditions are met
            if conf >= 0.8:
                detected_class = class_name
                point_msg = Point()
                point_msg.x = cx
                point_msg.y = cy
                #point_msg.z = s

                detected_class_distance = (cx,cy)
                self.distance_publisher.publish(point_msg)
                
                # Publish the detected class name and distance
            self.class_publisher.publish(detected_class)
            #self.distance_publisher.publish(point_msg)

            label = "{}: {:.2f}".format(class_name, conf)
            #color = (0, 255, 0)
              # Coordinates of the dot
            radius = 2  # Radius of the dot

            draw.rectangle([x1, y1, x2, y2], outline=color, width=2)
            draw.text((x1, y1), label, fill=color)
            draw.ellipse([(c[0]-radius, c[1]-radius), (c[0]+radius, c[1]+radius)], fill=color)

        print(detected_class)
        print(detected_class_distance)

        # Save the image with detections
        im_draw_cv2 = np.array(im_draw)[:, :, [2, 1, 0]]
        cv2.imwrite('./images/image_with_detections_{:04d}.jpg'.format(frame_number), im_draw_cv2)  # Save the image
        return im_draw


    
def main():
    frame_number = 0  # Initialize frame number
    rospy.init_node('zed_image_viewer', anonymous=True)
    model_path = './yolo_7.onnx' # Path to your ONNX model
    rate =rospy.Rate(50)
    image_subscriber = ZEDImageSubscriber()
    object_detector = ObjectDetector(model_path)
    
    try:
        while not rospy.is_shutdown():
            if image_subscriber.latest_image is not None:
                image = image_subscriber.latest_image
                im_with_boxes = object_detector.detect_objects(image, frame_number)
                frame_number += 1
                cv2.imshow("Left Camera Image with Detection", im_with_boxes)
                cv2.waitKey(1)
    except KeyboardInterrupt:
        print("Shutting down...")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
