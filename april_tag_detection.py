import cv2 as cv
import numpy as np
import time
from pupil_apriltags import Detector
import glob
import os
import pickle

__author__ = '{Andrew Hwang, Jeong Hwang, Khai Hern Low}'


class AprilTagDetector:
    """
    An AprilTagDetector can detect AprilTags from images
    """

    def __init__(self, directory, size, show=False, calibrate=False):
        """
        Constructs an AprilTagDetector.

        Parameters
        ----------
        directory : str
            The name of the directory containing image files for camera calibration
        size : (int, int)
            The width and height of the video frames
        show : Boolean, optional
            Whether or not to show the images used for camera calibration (default is False)
        calibrate : Boolean, optional
            Whether or not to calibrate a camera and save the coefficients to a file (default is False)
        """
        w = size[0]
        h = size[1]
        if (calibrate):
            CHESSBOARD = (6, 8)  # the dimensions of chessboard
            # criteria for stopping the loop for finding chessboard corners: the specified accuracy, epsilon, is reached or the specified number of iterations are completed.
            criteria = (cv.TERM_CRITERIA_EPS +
                        cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            points3D = []  # vector for 3D chessboard corners
            points2D = []  # vector for 2D chessboard corners
            #  3D chessboard corners
            corners3d = np.zeros((1, CHESSBOARD[0] * CHESSBOARD[1], 3), np.float32)
            corners3d[0, :, :2] = np.mgrid[0:CHESSBOARD[0],
                                        0:CHESSBOARD[1]].T.reshape(-1, 2)
            # for each image file in the specified directory
            for filename in glob.glob(directory):
                image = cv.imread(filename)
                gray_image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
                # find the chessboard corners
                ret, corners = cv.findChessboardCorners(
                    gray_image, CHESSBOARD, cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK + cv.CALIB_CB_NORMALIZE_IMAGE)
                if ret == True:  # if the chessboard corners found
                    # refine the coordinates of the 2D chessboard corners
                    corners2d = cv.cornerSubPix(
                        gray_image, corners, (11, 11), (-1, -1), criteria)
                    # register 3D chessboard corners as object points
                    points3D.append(corners3d)
                    # register 2D chessboard corners as image points
                    points2D.append(corners2d)
                    image = cv.drawChessboardCorners(
                        image, CHESSBOARD, corners2d, ret)  # mark chessboard corners
                if show:
                    cv.imshow('img', image)
                    cv.waitKey(0)
            cv.destroyAllWindows()
            # perform camera calibration based on the 3D chessboard corners and 2D chessboard corners
            ret, camera_mtx, distortion_mtx, r_vecs, t_vecs = cv.calibrateCamera(
                points3D, points2D, gray_image.shape[::-1], None, None)

            # save the coefficients produce by the camera calibration to 'camera_calib_pickle.p' file
            calib_result_pickle = {}
            calib_result_pickle['camera_mtx'] = camera_mtx
            calib_result_pickle['distortion_mtx'] = distortion_mtx
            calib_result_pickle['r_vecs'] = r_vecs
            calib_result_pickle['t_vecs'] = t_vecs
            pickle.dump(calib_result_pickle, open('camera_calib_pickle.p', 'wb' )) 
        else:
            # load the coefficients from the file
            calib_result_pickle = pickle.load(open('camera_calib_pickle.p', 'rb' ))
            camera_mtx = calib_result_pickle['camera_mtx']
            distortion_mtx = calib_result_pickle['distortion_mtx']
            r_vecs = calib_result_pickle['r_vecs']
            t_vecs = calib_result_pickle['t_vecs']

        self.camera_params = [
            camera_mtx[0, 0], camera_mtx[1, 1], camera_mtx[2, 0], camera_mtx[2, 1]]
        new_camera_mtx, roi = cv.getOptimalNewCameraMatrix(
            camera_mtx, distortion_mtx, (w, h), 1, (w, h))
        self.mapx, self.mapy = cv.initUndistortRectifyMap(
            camera_mtx, distortion_mtx, None, new_camera_mtx, (w, h), 5)
        self.detector = Detector(families='tag16h5', decode_sharpening=0.025,
                                 refine_edges=5, quad_decimate=0.125, quad_sigma=2)
        # Default Values: nthreads=1 quad_decimate=2 quad_sigma=0 refine_edges=1 decode_sharpening=0.25

    def tags(self, image, tag_size):
        """
        Detetcs AprilTags from the specified image.

        Parameters
        ----------
        image
            an image
        tag_size
            the tag size (in meters)

        Returns
        -------    
        tags
            TDB
        """
        undistorted_image = cv.remap(
            image, self.mapx, self.mapy, cv.INTER_LINEAR)
        gray_image = cv.cvtColor(undistorted_image, cv.COLOR_BGR2GRAY)
        tags = self.detector.detect(
            gray_image, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=tag_size)
        return tags


def analyze(tag):
    """
    Analyzes the spcified tag.

    Parameters
    ----------
    tag
        TBD

    Returns
    -------    
    translation
        the translation vector
    roll
        the roll (the extent to which the tag facing up)
    pitch
        the pitch (the extent to which the tag facing right)
    yaw
        the yaw (the extent to which the tag rotated counterclockwise)
    """
    # create the transformation matrix T
    T = np.concatenate((tag.pose_R, tag.pose_t.reshape(3, 1)), axis=1)
    T = np.concatenate((T, np.array([[0, 0, 0, 1]])), axis=0)
    # calculate the inverse of T
    T_inv = np.linalg.inv(T)
    rotation = T_inv[:3, :3]
    translation = T_inv[:3, 3]
    roll = np.arctan2(rotation[2, 1], rotation[2, 2]) * 180/np.pi
    pitch = np.arcsin(-rotation[2, 0]) * 180/np.pi
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0]) * 180/np.pi
    return translation, roll, pitch, yaw


def pose_info(tag):
    """
    Returns a string representaton of the pose information of the spcified tag.

    Parameters
    ----------
    tag
        TBD

    Returns
    -------    
    a string representaton of the pose information of the spcified tag in the form of
        tag ID; x; y; z; roll; pitch; yaw
    """
    translation, roll, pitch, yaw = analyze(tag)
    info = f"{tag.tag_id}; " + "{:.3f}; {:.3f}; {:.3f}; {:.2f}; {:.2f}; {:.2f}".format(
        translation[0], translation[1], translation[2], roll, pitch, yaw)
    return info


def draw_tags(image, tags):
    """
    Draws the specifed tags on the spcified imae.

    Parameters
    ----------
    image
        an image
    tags
        TBD

    Returns
    -------    
    image
        the image with AprilTags highlighted
    """
    for tag in tags:
        center = (int(tag.center[0]), int(tag.center[1]))
        corner0 = (int(tag.corners[0][0]), int(tag.corners[0][1]))
        corner1 = (int(tag.corners[1][0]), int(tag.corners[1][1]))
        corner2 = (int(tag.corners[2][0]), int(tag.corners[2][1]))
        corner3 = (int(tag.corners[3][0]), int(tag.corners[3][1]))
        cv.circle(image, center, 5, (0, 0, 255), 2)
        cv.line(image, corner0, corner1, (255, 0, 0), 2)
        cv.line(image, corner1, corner2, (255, 0, 0), 2)
        cv.line(image, corner2, corner3, (0, 255, 0), 2)
        cv.line(image, corner3, corner0, (0, 255, 0), 2)
        cv.putText(image, str(tag.tag_id), (center[0] - 10, center[1] - 10),
                   cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv.LINE_AA)
    return image


if __name__ == "__main__":
    # TODO: create a network table linked to SmartDashboard
    save_images=False
#    save_images=True
    cap = cv.VideoCapture(2)
    tag_size = 0.1524
    detector = AprilTagDetector('./calib_images/*.jpg', (600, 600), show=True, calibrate=False)
    i = -1
    print('frame ID; tag family; tag ID; translation (x); translation (y); translation (z); roll (degrees); pitch (degrees); yaw (degrees)')
    while (True):
        start_time = time.time()
        ret, frame = cap.read()
        if not ret:
            break
        # detect tags from the current frame
        tags = detector.tags(frame, tag_size)
        if len(tags) > 0:
            i = i+1
            if save_images:
                cv.imwrite('images' + os.sep + str(i) + '.jpg', frame)
        for tag in tags:  # for each tag detected
            # calculate the translation vector and the rotation angles
            info = pose_info(tag)
            print(f'{i}; {tag.tag_family}; ' + info)
        # TODO: save an array of pose_info strings in the SmartDashboard (key: tags)
        cv.putText(frame, "elapsed time:" + '{:.1f}'.format((time.time() - start_time) *
                   1000) + "ms", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv.LINE_AA)
        cv.imshow('AprilTag Detection', draw_tags(frame, tags))
        key = cv.waitKey(1)
        if key != -1:  # if some key pressed, then exit
            break
    cap.release()
    cv.destroyAllWindows()

