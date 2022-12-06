import numpy as np
import cv2
from pupil_apriltags import Detector
from cscore import CameraServer
import time
import glob

#get camera parameters
#modified only very slightly from https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
#needs about 30 png images of the checkboard(from the camera) in a folder named 'calibration_images' 
def calibrateCamera():
	# termination criteria
	criteria = (cv2.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
	# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
	objp = np.zeros((8*6,3), np.float32)
	objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)
	# Arrays to store object points and image points from all the images.
	objpoints = [] # 3d point in real world space
	imgpoints = [] # 2d points in image plane.
	images = glob.glob('./calibration_images/*.png')
	for fname in images:
		img = cv2.imread(fname)
		gray = cv2.cvtColor(img, cv.COLOR_BGR2GRAY)
		# Find the chess board corners
		ret, corners = cv2.findChessboardCorners(gray, (8,6), None)
		# If found, add object points, image points (after refining them)
		if ret == True:
			objpoints.append(objp)
			corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
			imgpoints.append(corners2)
	return cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


def main():
	#camera resolution width+height
	w=600
	h=600

	#One option for getting the camera stream-- might start using this
	#frc option
	#camera = CameraServer.getInstance().startAutomaticCapture()

	#opencv object to get the camera stream
	vid = cv2.VideoCapture(0)

	#frc options for getting camera stream and outputting a changed camera stream
	#inputStream = CameraServer.getInstance().getVideo()
	#outputStream = CameraServer.getInstance().putVideo("Processed",w,h)

	#get camera parameters we need to estimate pose from apriltag
	#and undistort the image
   	ret, cameraMtx, distMtx, rvecs, tvecs = calibrateCamera()
	
	#get the matrices we need to undistort the frames
	newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cameraMtx, distMtx, (w,h), 1, (w,h))
	
	#initialize the apriltag dector for the apriltag 36h11 tag family
	detector = Detector(families="tag36h11")
	
	#camera parameters for the detector(just the ones it needs)
	camParamsDetect = [cameraMtx[0,0], cameraMtx[1,1], cameraMtx[2,0], cameraMtx[2,1]]
	mapx, mapy = cv2.initUndistortRectifyMap(cameraMtx, distMtx, None, newcameramtx, (w,h), 5)

	#size of the tag in meters, as measured from inside the black border
	kTagSize = .128

	newOrientationMtx = np.matrix('1 0 0; 0 -1 0; 0 0 -1;')

	#initialize networktable to publish the results
	NetworkTables.initialize(server='roborio-20-frc.local')
	pose_table = NetworkTables.getInstance().getTable('apriltag_poses')

	while(True):
		#get start time to measure the time it takes to run this(for framerate)
		start_time = time.time()

		#get the video frame
		ret,frame = vid.read()

		
		#undistort our frame using the camera info from earlier
		unDist = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

		#convert the frame to grayscale and use the detector to detect apriltags
		grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		results = detector.detect(unDist, estimate_tag_pose=True, camera_params=camParamsDetect, tag_size=kTagSize)
		
		#for every detected apriltag, determine our pose relative to it
		#x=right
		#y=up
		#Z=out of apriltag
		for r in results:
			tag_pose = newOrientationMtx @ r.pose_R.T @ (-1 * r.pose_T)
			pose_table.putNumberArray(r.tag_id, tag_pose.toList())
			print("Tag id:", tagid, "Tag pose: ", tag_pose.toList)
		#can use this to annotate apriltags on the an output image
		#outputStream.putFrame(grayframe)

		#print how long it took to process
		print("Processing time=" + str(time.time() - start_time))
main()
