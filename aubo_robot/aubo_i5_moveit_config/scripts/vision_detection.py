from sensor_msgs.msg import CameraInfo
import pyrealsense2 as rs
import cv2
import math
import numpy as np

class RSCamera:
    def __init__(self):
        # camera matrix of realsense
        self.camera_info = CameraInfo()
        # camera_info.K = [616.3787841796875, 0.0, 434.0303955078125, 0.0, 616.4257202148438, 234.33065795898438, 0.0, 0.0, 1.0]
        self.camera_info.K = [931.6937866210938, 0.0, 624.7894897460938, 0.0, 931.462890625, 360.5186767578125, 0.0, 0.0, 1.0]
        self.camera_info.header.frame_id = 'camera_color_optical_frame'
        self.camera_info.height = 720
        self.camera_info.width = 1280
        self.image_size=[640, 480]
        self.points = rs.points()
        self.pipeline= rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.image_size[0], self.image_size[1], 
                                  rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, self.image_size[0], self.image_size[1], 
                                  rs.format.z16, 30)
        self.profile = self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        self.board_center_x = 0
        self.board_center_y = 0

        self.board = [[[0, 0], [0, 0], [0, 0]],
                      [[0, 0], [0, 0], [0, 0]],
                      [[0, 0], [0, 0], [0, 0]]]

        self.h_choice = ''

        self.R = [0, 0, 255]
        self.G = [0, 255, 0]
        self.B = [255, 0, 0]

    def color_image_sample(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        return np.asanyarray(color_frame.get_data())

    def contours_filter(self, contours):
        pieces_contours = []
        for contour in contours:
            if cv2.contourArea(contour)>=50:
                pieces_contours.append(contour)
        return pieces_contours

    def find_center(self, contours):
        centers = []
        for contour in contours:
            M = cv2.moments(contour)
            centers.append([int(M['m10']/M['m00']), int(M['m01']/M['m00'])])
        return centers

    def get_blue_mask(self, hsv_image):
        Lower = np.array([100, 240, 50])
        Upper = np.array([124, 255, 250])
        mask = cv2.inRange(hsv_image, Lower, Upper)
        return mask

    def get_green_mask(self, hsv_image):
        Lower = np.array([35, 140, 50])
        Upper = np.array([77, 255, 250])
        mask = cv2.inRange(hsv_image, Lower, Upper)
        return mask

    def get_red_mask(self, hsv_image):
        Lower = np.array([156, 80, 50])
        Upper = np.array([180, 255, 250])
        mask = cv2.inRange(hsv_image, Lower, Upper)
        return mask

    def board_detect(self):
        red_centers = []
        #color_image = self.color_image_sample()
        while len(red_centers)!=4:
            color_image = self.color_image_sample()
            hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            mask = self.get_red_mask(hsv_image)
            mask,contours,hierarchv = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            red_areas = self.contours_filter(contours)
            red_centers = self.find_center(red_areas)
            print(len(red_centers))
        self.board_center_x = 0
        self.board_center_y = 0
        for center in red_centers:
            print(center)
            self.board_center_x += center[0]
            self.board_center_y += center[1]
        self.board_center_x = self.board_center_x / 4.0
        self.board_center_y = self.board_center_y / 4.0
        
        for i in range(len(self.board)):
            for j in range(len(self.board[i])):
                self.board[j][i][0] = self.board_center_x - (i%3-1)*26
                self.board[j][i][1] = self.board_center_y - (j%3-1)*30
                color_image=cv2.circle(color_image, (int(self.board[i][j][0]), int(self.board[i][j][1])), 
                                       2, self.R, -1)
        return color_image

    def pieces_detect(self):
        color_image = self.color_image_sample()
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        #mask=hsv_image
        if (self.h_choice=='b'):
            mask = self.get_blue_mask(hsv_image)
        elif (self.h_choice=='g'):
            mask = self.get_green_mask(hsv_image)

        mask,contours,hierarchv = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        pieces_contours = self.contours_filter(contours)
        pieces_centers = self.find_center(pieces_contours)

        color_image = cv2.drawContours(color_image, pieces_contours, -1, self.G, 1)
        for center in pieces_centers:
            color_image=cv2.circle(color_image, (center[0], center[1]), 2, self.R, -1)

        cv2.imshow('img', color_image)
        cv2.waitKey(1)
        return pieces_centers

    def distance(self, a, b):
        return math.sqrt(((a[0]-b[0])**2)+((a[1]-b[1])**2))

    def area_match(self, pieces_centers):
        matched_num = []
        for center in pieces_centers:
            for i in range(len(self.board)):
                for j in range(len(self.board[i])):
                    #print(self.distance(center, self.board[i][j]))
                    if self.distance(center, self.board[i][j]) < 12:
                        matched_num.append([i, j])
        return matched_num

    def human_detection(self):
        pieces_centers = self.pieces_detect()
        return self.area_match(pieces_centers)
