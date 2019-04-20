#!/usr/bin/env python
import numpy as np
import rospy, tf, sys, time
from geometry_msgs.msg import Pose, PoseStamped
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
import tf2_ros
import tf2_geometry_msgs
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
import cv2
import pyrealsense2 as rs
from game import *


# camera matrix of realsense
camera_info = CameraInfo()
# camera_info.K = [616.3787841796875, 0.0, 434.0303955078125, 0.0, 616.4257202148438, 234.33065795898438, 0.0, 0.0, 1.0]
camera_info.K = [931.6937866210938, 0.0, 624.7894897460938, 0.0, 931.462890625, 360.5186767578125, 0.0, 0.0, 1.0]
camera_info.header.frame_id = 'camera_color_optical_frame'
camera_info.height = 720
camera_info.width = 1280

chessboard_size = (3,3)
refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

R = [0, 0, 255]
G = [0, 255, 0]
B = [255, 0, 0]


def contours_filter(contours):
    pieces_contours = []
    for contour in contours:
        if cv2.contourArea(contour)>=50:
            pieces_contours.append(contour)
    return pieces_contours

def find_center(contours):
    centers = []
    for contour in contours:
        M = cv2.moments(contour)
        centers.append([int(M['m10']/M['m00']), int(M['m01']/M['m00'])])
    return centers

def get_blue_mask(hsv_image):
    Lower = np.array([100, 240, 50])
    Upper = np.array([124, 255, 250])
    mask = cv2.inRange(hsv_image, Lower, Upper)
    return mask

def get_green_mask(hsv_image):
    Lower = np.array([35, 140, 50])
    Upper = np.array([77, 255, 250])
    mask = cv2.inRange(hsv_image, Lower, Upper)
    return mask

def get_red_mask(hsv_image):
    Lower = np.array([156, 80, 50])
    Upper = np.array([180, 255, 250])
    mask = cv2.inRange(hsv_image, Lower, Upper)
    return mask

def pieces_detect(color_image, piece_type='blue'):
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    if (piece_type=='blue' or piece_type=='b'):
        #print('b')
        mask = get_blue_mask(hsv_image)
    elif (piece_type=='green' or piece_type=='g'):
        #print('g')
        mask = get_green_mask(hsv_image)
    elif (piece_type=='red' or piece_type=='r'):
        mask = get_red_mask(hsv_image)

    mask,contours,hierarchv = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    pieces_contours = contours_filter(contours)
    pieces_centers = find_center(pieces_contours)

    color_image = cv2.drawContours(color_image, pieces_contours, -1, G, 1)
    for center in pieces_centers:
        color_image=cv2.circle(color_image, (center[0], center[1]), 2, R, -1)

    #gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    #circles=cv2.HoughCircles(gray_image,cv2.HOUGH_GRADIENT,1,1,param1=10,param2=20,minRadius=5,maxRadius=8)
    #if circles is not None:
    #    for circle in circles[0]:
    #        x, y, r = int(circle[0]), int(circle[1]), int(circle[2])
    #        color_image=cv2.circle(color_image, (x, y), r, (0,0,255), -1)

    return mask, color_image

    #gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    #gray_image = np.float32(gray_image)
    #dst = cv2.cornerHarris(gray_image, 2, 1, 0.04)
    #dst = cv2.dilate(dst, None)
    #color_image[dst>0.05*dst.max()]=[0, 0, 255]

def robot_player(color_image, depth_image, ai_move):
    # TODO: detect circles or other features in the color image


    # TODO: find the x, y, z of the pick point in the camera coordinate and transform the pick point in the robot base coordinate
    # pick-definetion type: geometry_msgs/Pose
    # trans-definetion type: geometry_msgs/Transform

    # actual move action for the robot player:
    # set_states() # io_interface

    # set our own end effector pose target
    pick.pose.position.z = pick.pose.position.z + 0.05
    group.set_pose_target(pick, end_effector_link='wrist3_Link') #wrist3_Link
    plan = group.plan()
    group.execute(plan)

    # set our own end effector pose target
    pick.pose.position.z = pick.pose.position.z - 0.060
    group.set_pose_target(pick, end_effector_link='wrist3_Link')
    plan = group.plan()
    group.execute(plan)
    time.sleep(1)

    # send signal to executing end-effector
    # set_digital_out(0, False)
    time.sleep(1)

    pick.pose.position.z = pick.pose.position.z + 0.1
    group.set_pose_target(pick, end_effector_link='wrist3_Link')
    plan = group.plan()
    group.execute(plan)
    time.sleep(1)

    # set place position chosed by AI
    place = PoseStamped()
    place.pose.position.x = checkerboard_x[ai_move]
    place.pose.position.y = checkerboard_y[ai_move]
    place.pose.position.z = checkerboard_z

    # tranform place position in camera coordinate to arm coordinate
    place_robot = tf2_geometry_msgs.do_transform_pose(place, trans)
    place_robot.pose.position.z += 0.05 + 0.13
    place_robot.pose.orientation.x = 1
    place_robot.pose.orientation.y = 0.0
    place_robot.pose.orientation.z = 0.0
    place_robot.pose.orientation.w = 0.0
    
    group.set_pose_target(place_robot)
    plan = group.plan()
    group.execute(plan)
    time.sleep(1)
    
    place_robot.pose.position.z = place_robot.pose.position.z - 0.04
    group.set_pose_target(place_robot, end_effector_link='wrist3_Link')
    plan = group.plan()
    group.execute(plan)
    time.sleep(1)
    
    # send signal to executing end-effector
    # set_digital_out(0, True)
    time.sleep(1)
    
    place_robot.pose.position.z = place_robot.pose.position.z + 0.04
    group.set_pose_target(place_robot, end_effector_link='wrist3_Link')
    plan = group.plan()
    group.execute(plan)
    time.sleep(1)
    
    # back to homepose
    group.set_joint_value_target(home_joint_position)
    plan = group.plan()
    group.execute(plan)

def main():
    # ROS node initialization
    rospy.init_node('perception', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")
    group.set_planner_id('RRTConnectkConfigDefault')
    group.set_num_planning_attempts(5)
    group.set_planning_time(5)
    group.set_max_velocity_scaling_factor(0.5)

    # get the transformation between robot base and camera
    tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    listener = tf2_ros.TransformListener(tfBuffer)
    try:
        trans = tfBuffer.lookup_transform('world', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print "Find transform failed"

    # define and go to the home pose waiting for picking instruction
    home_joint_position = [-80.25/180*3.14, 2.0/180*3.14, 125.8/180*3.14, 34.1/180*3.14, 96.1/180*3.14, 12.7/180*3.14]
    group.set_joint_value_target(home_joint_position)
    plan = group.plan()
    # raw_input('Press enter to go the home position: ')
    group.execute(plan)
    time.sleep(1)

    # initiate game
    game = TicTacToe()

    # initiate realsense
    points = rs.points()
    pipeline= rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    align_to = rs.stream.color
    align = rs.align(align_to)

    # capture the initial image,
    time.sleep(3)
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    depth_image_background = np.asanyarray(aligned_depth_frame.get_data())
    color_image_background = np.asanyarray(color_frame.get_data())

    # TODO: detect the tic tac toe game board, compute all the 9 positions of placing X and O


    # TODO: define a function to detect the human's move from color image
    def detect_person_move(color_image, game):
        pass

    # start the game
    while game.gameOver() == False:
        # TODO: Ask the human to place her piece X, detect the move and update the state of the game board


        if game.gameOver() == True:
            break

        # TODO: calculate robot's move and execute it


    print("Game Over. " + game.whoWon() + " Wins")

# initiate realsense
#image_size=[1280, 720]
image_size=[640, 480]

points = rs.points()
pipeline= rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, image_size[0], image_size[1], rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, image_size[0], image_size[1], rs.format.z16, 30)
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
align_to = rs.stream.color
align = rs.align(align_to)

# capture the initial image,
time.sleep(3)


while 1:
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    #aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    #depth_image_background = np.asanyarray(aligned_depth_frame.get_data())
    color_image_background = np.asanyarray(color_frame.get_data())
    hsv, img = pieces_detect(color_image_background, piece_type='r')
    
    cv2.imshow('img', img)
    cv2.imshow('hsv', hsv)
    cv2.waitKey(1)
    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break
