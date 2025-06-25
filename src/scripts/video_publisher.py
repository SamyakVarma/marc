#!/usr/local/bin/python3.10

import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ArucoDetection_definitions import *

class Detector(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.map_width = 30
        self.map_height = 18
        self.cap = None
        self.desired_aruco_dictionary1 = "DICT_ARUCO_ORIGINAL"
        self.desired_aruco_dictionary2 = "DICT_6X6_50"
        
        self.init_square_points = [
            [10, cv2.CAP_PROP_FRAME_HEIGHT-10],
            [cv2.CAP_PROP_FRAME_WIDTH-10, cv2.CAP_PROP_FRAME_HEIGHT-10],
            [cv2.CAP_PROP_FRAME_WIDTH-10, 10],
            [10, 10]
        ]
        
        self.current_square_points = [
            [10, 400], 
            [400, 400],
            [400, 10],
            [10, 10]
        ]
        self.centerCorner = None
        self.current_center_corner = [[0, 0]]
        self.marker_location_hold = True
        
        self.bridge = CvBridge()
        self.img_publisher = self.create_publisher(Image, 'wrapped_image', 10)
        
        # ArUco dictionaries
        self.ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
        }

    def get_markers(self, vid_frame, aruco_dictionary, aruco_parameters):
        bboxs, ids, rejected = cv2.aruco.detectMarkers(vid_frame, aruco_dictionary, parameters=aruco_parameters)
        if ids is not None:
            ids_sorted = [id_number[0] for id_number in ids]
        else:
            ids_sorted = ids
        return bboxs, ids_sorted

    def mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click_position = (x, y)
            winX, winY, win_width, win_height = cv2.getWindowImageRect("img_wrapped")
            fin_x = (x*self.map_width/win_width)-(self.map_width/2)
            fin_y = ((y*self.map_height/win_height)-self.map_height)*-1
            click_coordinates = {"x": fin_x, "y": fin_y}
            print(f"Mouse clicked at: {click_coordinates}")

    def start_capture(self, camera_id=1):
        """Initialize video capture"""
        self.cap = cv2.VideoCapture(camera_id)
        #increase capture size
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        cv2.namedWindow("img_wrapped")
        cv2.setMouseCallback("img_wrapped", self.mouse_click)
        print("Capture initialized")

    def stop_capture(self):
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()

    def run(self):
        print(f"[INFO] detecting '{self.desired_aruco_dictionary1}' markers...")
        
        this_aruco_dictionary1 = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT[self.desired_aruco_dictionary1])
        this_aruco_parameters1 = cv2.aruco.DetectorParameters_create()
        this_aruco_dictionary2 = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT[self.desired_aruco_dictionary2])
        this_aruco_parameters2 = cv2.aruco.DetectorParameters_create()

        self.start_capture(1)
        square_points = self.current_square_points
        start_time = time.time()
        if 1:
            while rclpy.ok():
                ret, frame = self.cap.read()
                # Detect markers
                markers, ids = self.get_markers(frame, this_aruco_dictionary1, this_aruco_parameters1)
                frame_clean = frame.copy()
                left_corners, corner_ids = getMarkerCoordinates(markers, ids, 0)
                
                # Update marker positions
                if self.marker_location_hold and corner_ids is not None:
                    for i, id_ in enumerate(corner_ids):
                        if id_ <= 4:
                            self.current_square_points[id_-1] = left_corners[i]
                    left_corners = self.current_square_points
                    corner_ids = [1, 2, 3, 4]
                
                # Process frame
                frame_with_square, squareFound = draw_field(frame, left_corners, corner_ids)
                
                if squareFound:
                    square_points = left_corners
                img_wrapped = four_point_transform(frame_clean, np.array(square_points))
                
                # Process wrapped image
                h, w, c = img_wrapped.shape
                marker_foam, ids_foam = self.get_markers(img_wrapped, this_aruco_dictionary2, this_aruco_parameters2)
                left_corner_foam, corner_id_foam = getMarkerCoordinates(marker_foam, ids_foam, 0)
                self.centerCorner = getMarkerCenter_foam(marker_foam)
                
                # Update foam marker position
                if self.marker_location_hold and corner_id_foam is not None:
                    self.current_center_corner[0] = self.centerCorner[0]
                self.centerCorner[0] = self.current_center_corner[0]

                # Draw visualization
                draw_corners(img_wrapped, self.centerCorner)
                img_wrapped = cv2.line(img_wrapped, (0,len(img_wrapped)), (len(img_wrapped[0]),len(img_wrapped)), (0,0,255), 3)
                img_wrapped = cv2.line(img_wrapped, (len(img_wrapped[0])//2,0), (len(img_wrapped[0])//2,len(img_wrapped)), (255,0,0), 2)
                draw_numbers(img_wrapped, left_corner_foam, corner_id_foam)

                # Publish wrapped image
                img_msg = self.bridge.cv2_to_imgmsg(img_wrapped, encoding="bgr8")
                self.img_publisher.publish(img_msg)

                # Display frames
                cv2.imshow('Capture', frame_with_square)
                cv2.imshow('img_wrapped', img_wrapped)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                rclpy.spin_once(self, timeout_sec=0)

        else:
            self.stop_capture()
            return self.centerCorner    
if __name__ == '__main__':
    rclpy.init()
    detector = Detector()
    foam_center = detector.run()
    rclpy.shutdown()