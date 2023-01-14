import cv2 as cv
import numpy as np
from pupil_apriltags import Detector
from networktables import NetworkTables
import os
import glob
import time
import copy

#initialize network tables
NetworkTables.initialize(server='roborio-20-frc.local')
pose_table = NetworkTables.getInstance().getTable('apriltag_poses')
elapsed_time = 0

def calibrate_camera():
  # Define the dimensions of checkerboard
  CHECKERBOARD = (6, 8)

  # stop the iteration when specified accuracy, epsilon, is reached or
  # specified number of iterations are completed.
  criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

  # Vector for 3D points
  threedpoints = []
    
  # Vector for 2D points
  twodpoints = []

  #  3D points real world coordinates
  objectp3d = np.zeros((1, CHECKERBOARD[0] 
                        * CHECKERBOARD[1], 
                        3), np.float32)
  objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0],
                                0:CHECKERBOARD[1]].T.reshape(-1, 2)
  prev_img_shape = None

  # Extracting path of individual image stored in a given directory. Since no path is
  # specified, it will take current directory jpg files alone
  images = glob.glob('./calib_images/*.jpg')

  for filename in images:
    image = cv.imread(filename)
    grayColor = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    # If desired number of corners are found in the image then ret = true
    ret, corners = cv.findChessboardCorners(grayColor, CHECKERBOARD, cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK + cv.CALIB_CB_NORMALIZE_IMAGE)

    # If desired number of corners can be detected then,
    # refine the pixel coordinates and display them on the images of checker board
    if ret == True:
      threedpoints.append(objectp3d)

      # Refining pixel coordinates for given 2d points.
      corners2 = cv.cornerSubPix(grayColor, corners, (11, 11), (-1, -1), criteria)

      twodpoints.append(corners2)

      # Draw and display the corners
      image = cv.drawChessboardCorners(image, CHECKERBOARD, corners2, ret)

    #cv.imshow('img', image)
    #cv.waitKey(0)
    
  cv.destroyAllWindows()    
    
  # Perform camera calibration by passing the value of above found out 3D points (threedpoints)
  # and its corresponding pixel coordinates of the detected corners (twodpoints)
  return cv.calibrateCamera(threedpoints, twodpoints, grayColor.shape[::-1], None, None)

def main(): 
  w = 1280
  h = 720

  cap = cv.VideoCapture(14)
  ret, camera_mtx, distortion_mtx, r_vecs, t_vecs = calibrate_camera()

  detector = Detector(families='tag16h5', decode_sharpening=0.025, refine_edges=5, quad_decimate=0.125, quad_sigma=2)
  # Default Values: nthreads=1 quad_decimate=2 quad_sigma=0 refine_edges=1 decode_sharpening=0.25
  camera_params_detect = [camera_mtx[0,0], camera_mtx[1,1], camera_mtx[2,0], camera_mtx[2,1]]
  
  ret, frame = cap.read()
  # h, w = frame.shape[:2]
  new_camera_mtx, roi = cv.getOptimalNewCameraMatrix(camera_mtx, distortion_mtx, (w, h), 1, (w, h))
  mapx, mapy = cv.initUndistortRectifyMap(camera_mtx, distortion_mtx, None, new_camera_mtx, (w,h), 5)

  k_tag_size = 0.1524
  
  
  
  while(True):
    start_time = time.time()

    ret, frame = cap.read()
    if not ret:
      break


    #undistort
    # undistorted_frame = cv.undistort(frame, camera_mtx, distortion_mtx, None, new_camera_mtx)
    # x, y, w, h = roi
    # undistorted_frame = undistorted_frame[y:y+h, x:x+w]

    #undistorted_frame = cv.remap(frame, mapx, mapy, cv.INTER_LINEAR)

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    debug_image = copy.deepcopy(frame)
    tags = detector.detect(gray_frame, estimate_tag_pose=True, camera_params=camera_params_detect, tag_size=k_tag_size)
    
    #drawing to the frame
    debug_image = draw_tags(debug_image, tags, elapsed_time)
    # Cross Hair
    cv.line(debug_image, (0,0), (debug_image.shape[1], debug_image.shape[0]), (0, 0, 0), 2)
    cv.line(debug_image, (debug_image.shape[1], 0), (0, debug_image.shape[0]), (0, 0, 0), 2)

    elapsed_time = time.time() - start_time

    #key Processing (Exit: Escape Key)
    key = cv.waitKey(1)
    if key == 27:  # ESC
        break

    #show screen
    cv.imshow('AprilTag Detect', debug_image)

    #time.sleep(0.5)

  cap.release()
  cv.destroyAllWindows()

def draw_tags(image, tags, elapsed_time):
  for tag in tags:
    tag_family = tag.tag_family
    tag_id = tag.tag_id
    center = tag.center
    corners = tag.corners

    print(f'Tad ID: {tag_id} | family: {tag_family}')

    center = (int(center[0]), int(center[1]))
    corner_01 = (int(corners[0][0]), int(corners[0][1]))
    corner_02 = (int(corners[1][0]), int(corners[1][1]))
    corner_03 = (int(corners[2][0]), int(corners[2][1]))
    corner_04 = (int(corners[3][0]), int(corners[3][1]))

    # Center
    cv.circle(image, (center[0], center[1]), 5, (0, 0, 255), 2)

    # Sides
    cv.line(image, (corner_01[0], corner_01[1]),
            (corner_02[0], corner_02[1]), (255, 0, 0), 2)
    cv.line(image, (corner_02[0], corner_02[1]),
            (corner_03[0], corner_03[1]), (255, 0, 0), 2)
    cv.line(image, (corner_03[0], corner_03[1]),
            (corner_04[0], corner_04[1]), (0, 255, 0), 2)
    cv.line(image, (corner_04[0], corner_04[1]),
            (corner_01[0], corner_01[1]), (0, 255, 0), 2)

    # Tag Family, Tag ID
    # cv.putText(image,
    #            str(tag_family) + ':' + str(tag_id),
    #            (corner_01[0], corner_01[1] - 10), cv.FONT_HERSHEY_SIMPLEX,
    #            0.6, (0, 255, 0), 1, cv.LINE_AA)
    cv.putText(image, str(tag_id), (center[0] - 10, center[1] - 10),
                cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv.LINE_AA)

    # Offset distances to correct 
    tag.pose_t[2][0] /= 1.6 #offset z axis / distance
    print(tag.pose_t)

    # Create the transformation matrix T
    T = np.concatenate((tag.pose_R, tag.pose_t.reshape(3, 1)), axis=1)
    T = np.concatenate((T, np.array([[0, 0, 0, 1]])), axis=0)

    # Calculate the inverse of T
    T_inv = np.linalg.inv(T)

    # Extract the rotation and translation components from T_inv
    rotation = T_inv[:3, :3]
    translation = T_inv[:3, 3]

    # The pose of the AprilTag relative to the camera is given by the rotation matrix and the translation vector
    #print(f'Tag id: {tag.tag_id},   Rotation matrix: {rotation},   Translation Vector: {translation}')

    # Calculate the pitch angle (in degrees)
    pitch = np.arcsin(-rotation[2, 0]) * 180/np.pi

    # Calculate the roll angle (in degrees)
    roll = np.arctan2(rotation[2, 1], rotation[2, 2]) * 180/np.pi

    # Calculate the yaw angle (in degrees)
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0]) * 180/np.pi

    # Print the Euler angles
    #print("Pitch: {:.2f} degrees".format(pitch))
    #print("Roll: {:.2f} degrees".format(roll))
    #print("Yaw: {:.2f} degrees".format(yaw))
    #print('-----------------------------------------------------')
    pose_table.putNumberArray('tagid'+ str(tag_id), translation);

  # Processing time
  cv.putText(image,
    "Elapsed Time:" + '{:.1f}'.format(elapsed_time * 1000) + "ms",
    (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
    cv.LINE_AA)

  return image

main()