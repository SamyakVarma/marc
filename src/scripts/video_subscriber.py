#!/usr/bin/env python

import threading
from queue import Queue
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from inference_sdk import InferenceHTTPClient
import os

map_width = 30
map_height = 18

class CmdVideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            String,
            'cmdpickplace',
            self.cmd_callback,
            10)
        self.tool_queue = Queue()
        self.running = True
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(
            Image,
            'wrapped_image',
            self.image_callback,
            10)
            
        # Add click coordinates publisher
        self.click_coords_pub = self.create_publisher(
            Point,
            'click_coords',
            10)
        
        # Initialize Roboflow client
        self.client = InferenceHTTPClient(
            api_url="API URL HERE",
            api_key="API KEY HERE"
        )
        
        # latest_frame attribute
        self.latest_frame = None
        
        # thread control
        self.inference_queue = Queue()
        self.inference_thread = threading.Thread(target=self.inference_worker)
        self.inference_thread.daemon = True
        self.inference_thread.start()
        
        self.get_logger().info('Video subscriber node initialized')

    def cmd_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        try:
            command, tool = msg.data.split(':')
            # Add tool to queue immediately on subscription
            self.tool_queue.put(tool)
            # Add to inference queue
            if self.latest_frame is not None:
                self.inference_queue.put((command, tool, self.latest_frame.copy()))
            self.get_logger().info(f'Added to inference queue: {command} - {tool}')
        except ValueError as e:
            self.get_logger().error(f'Invalid command format: {msg.data}')

    def inference_worker(self):
        while rclpy.ok():
            try:
                if not self.inference_queue.empty():
                    command, tool, frame = self.inference_queue.get()
                    
                    # Save frame temporarily
                    temp_path = f'temp_frame_{self.get_clock().now().to_msg().sec}.jpg'
                    cv2.imwrite(temp_path, frame)
                    
                    # Run inference
                    try:
                        result = self.client.infer(temp_path, model_id="mechanical-tools-10000/3")
                        self.get_logger().info(f'Inference result: {result}')
                        
                        # Check if the current tool is in the predictions
                        tool_found = False
                        if 'predictions' in result:
                            for prediction in result['predictions']:
                                if prediction['class'].lower() == tool.lower():
                                    tool_found = True
                                    confidence = prediction['confidence']
                                    # Extract bounding box coordinates
                                    x = prediction['x']
                                    y = prediction['y']
                                    width = prediction['width'] 
                                    height = prediction['height']
                                    # Calculate center point
                                    center_x = x + width/2
                                    center_y = y + height/2
                                    win_height, win_width, c = frame.shape
                                    fin_x = (x*self.map_width/win_width)-(self.map_width/2)
                                    fin_y = ((y*self.map_height/win_height)-self.map_height)*-1                                
                                    # Publish center point coordinates
                                    point_msg = Point()
                                    point_msg.x = float(center_x)
                                    point_msg.y = float(center_y)
                                    point_msg.z = 0.0
                                    self.click_coords_pub.publish(point_msg)
                                    
                                    self.get_logger().info(f'Found tool: {tool} with confidence {confidence}')
                                    self.get_logger().info(f'Tool center point: ({center_x}, {center_y})')
                                    break
                            
                            if not tool_found:
                                self.get_logger().warning(f'Tool {tool} not found in the current frame')
                    
                    except Exception as e:
                        self.get_logger().error(f'Inference failed: {str(e)}')
                    finally:
                        if os.path.exists(temp_path):
                            os.remove(temp_path)
                        if not self.tool_queue.empty():
                            processed_tool = self.tool_queue.get()
                            self.get_logger().info(f'Processed and dequeued tool: {processed_tool}')
                        self.inference_queue.task_done()
            except Exception as e:
                self.get_logger().error(f'Error in inference worker: {str(e)}')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_frame = cv_image
        cv2.imshow("Received Image", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    video_subscriber = CmdVideoSubscriber()

    try:
        rclpy.spin(video_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        video_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()