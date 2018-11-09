import numpy as np
import cv2
import glob
import cv2.aruco as aruco

# Marker Size: 10cm, meaning the whole square (considering the black border is 10cm)
# The _50, 100, 250 or 1000 in the Dictionary are the markers possibilities, meaning the ID can go from 0 to 249, if it's 250.

class ArucoInterface(object):

    # If you want to see the pictures slowly then just change the WAIT_TIME
    def __init__(self, WAIT_TIME=10 , chessb_col=8, chessb_row=6):

        # Our chessboard is composed by 9 rows and 6 columns! Which only 8x6 corners would be detected

        self.WAIT_TIME = WAIT_TIME
        self.chessb_col = chessb_col
        self.chessb_row = chessb_row
        # Path for loading the images
        self.load_image = 'calib_images/*.jpg'
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Note: we would to calibrate the camera again if we change the camera optics (focus for example)
    def calibrate_camera(self):
        WAIT_TIME = self.WAIT_TIME
        chessb_col = self.chessb_col
        chessb_row = self.chessb_row
        load_image = self.load_image
        criteria = self.criteria

        # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((chessb_row*chessb_col,3), np.float32)
        objp[:,:2] = np.mgrid[0:chessb_col,0:chessb_row].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        images = glob.glob(load_image)

        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (chessb_col,chessb_row),None)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (chessb_col,chessb_row), corners2,ret)
                cv2.imshow('img',img)
                cv2.waitKey(WAIT_TIME)

        cv2.destroyAllWindows()
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

        cv_file = cv2.FileStorage("calib_images/test.yaml", cv2.FILE_STORAGE_WRITE)
        cv_file.write("camera_matrix", mtx)
        cv_file.write("dist_coeff", dist)
        # note you *release* you don't close() a FileStorage object
        cv_file.release()
        
    # Just extracts the values we have obtained with the calibrateCamera
    def extract_cameracalib(self):

        cv_file = cv2.FileStorage("calib_images/test.yaml", cv2.FILE_STORAGE_READ)

        print 'Type from read file:', type(cv_file)

        #note we also have to specify the type to retrieve other wise we only get a
        # FileNode object back instead of a matrix

        camera_matrix = cv_file.getNode("camera_matrix").mat()
        dist_matrix = cv_file.getNode("dist_coeff").mat()

        print type(camera_matrix)

        print("camera_matrix : ", camera_matrix.tolist())
        print("dist_matrix : ", dist_matrix.tolist())

        cv_file.release()

    # We choose an image to draw the axis served as reference to determine the pose
    def estimate_pose(self):
        chessb_col = self.chessb_col
        chessb_row = self.chessb_row
        load_image = self.load_image
        criteria = self.criteria

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((chessb_row*chessb_col,3), np.float32)
        objp[:,:2] = np.mgrid[0:chessb_col,0:chessb_row].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        images = glob.glob(load_image)

        #print "rodou 1"
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (chessb_col,chessb_row),None)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (chessb_col,chessb_row), corners2,ret)
            #print "rodou 2"

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
        # Function to draw the axis

        def draw(img, corners, imgpts):
            #print "rodou3"
            corner = tuple(corners[0].ravel())
            img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
            img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
            img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
            return img

        objp = np.zeros((chessb_row*chessb_col,3), np.float32)
        objp[:,:2] = np.mgrid[0:chessb_col,0:chessb_row].T.reshape(-1,2)

        axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

        # Just change to the specific picture we want to
        wished_picture = 'calib_images/left-0001.jpg'

        for fname in glob.glob(wished_picture):
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (chessb_col,chessb_row),None)

            if ret == True:
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

                # Find the rotation and translation vectors.
                _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)
                # project 3D points to image plane
                imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

                img = draw(img,corners2,imgpts)
                cv2.imshow('img',img)
                k = cv2.waitKey(0) & 0xff

        cv2.destroyAllWindows()

    # Tracking the Aruco with a video stream
    def track_aruco(self):
        chessb_col = self.chessb_col
        chessb_row = self.chessb_row
        load_image = self.load_image
        criteria = self.criteria
        marker_size = 0.045

        # Get video stream from camera source, in our case 1 is the argument because we're using
	# a laptop and there are 2 possible video streams (webcam and the logitech camera)
        cap = cv2.VideoCapture(1)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((chessb_row*chessb_col,3), np.float32)
        objp[:,:2] = np.mgrid[0:chessb_col,0:chessb_row].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        # Read all images
        images = glob.glob(load_image)

        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (chessb_col,chessb_row),None)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (chessb_col,chessb_row), corners2,ret)
            #print "entrou1"

        # We are again obtaining the parameters, later im gonna fix this so it is more optmized
        # The Rvecs and Tvecs in this case are not our rotational and translation parameters to determine the pose!
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

        while (True):
            # Getting a frame from video stream
            ret, frame = cap.read()

            # Since we are getting BGR frames we have to convert them
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Later try to use own dictionary to generate less markers since we dont need too much
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_100)

            parameters = aruco.DetectorParameters_create()
            #aruco_dict = aruco.Dictionary_get(aruco.DICT_5x5_50)

            # Lists of ids and the corners belonging to each id
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
            # print "entrou3"

            # Checks before if all the values on the matrix are different than None
            if np.all(ids != None):
                # Estimate pose of each marker and return the values rvet and tvec, NOTE THAT those are DIFFERENT from camera coefficents
                # The second parameter is the size of the marker in meters
                # The length of these vectors are just one, meaning they have 3 columns but just one row! And they store the real pose value
                rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners[0], marker_size, mtx, dist)
                #(rvec-tvec).any() # get rid of that nasty numpy value array error
                #print 'Rotation Vector: ', rvec
                #print 'Translation Vector:', tvec

                # Found the pose, now just quit the loop and send the command to the robot to stop moving as well!
               
                aruco.drawAxis(frame, mtx, dist, rvec[0], tvec[0], 0.1) #Draw Axis
                aruco.drawDetectedMarkers(frame, corners) #Draw A square around the markers


                ###### DRAW ID #####
                cv2.putText(frame, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
		print "Translation vector: ", tvec

            # Display the resulting frame
            cv2.imshow('frame',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()
        _rvec = np.copy(rvec)
        _tvec = np.copy(tvec)
        return _rvec, _tvec


arucao = ArucoInterface()
arucao.track_aruco()
