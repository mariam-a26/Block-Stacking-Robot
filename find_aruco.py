import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
import numpy as np
import DobotDllType as dType

api = dType.load()

def transform_camera_to_world(X_c_new, R, T):
    return R @ X_c_new + T

# Load transformation matrices
R = np.load("R.npy")
T = np.load("T.npy")
# T[2] += 0.008

# Load the predefined dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)


                          
# You should also put the distortion coefficients here                          
dist_coeffs = np.array([[ 2.54554124e-01, -1.86676633e+00  ,  2.42186718e-03,   -1.46515912e-02, 3.95909891e+00]], dtype=np.float32).T

# Camera calibration (Replace with actual values you obtained from your camera calibration)
camera_matrix = np.array([[1.07602275e+03,  0,         2.60138482e+02],
 [  0,         1.07328736e+03, 2.52187525e+02],
 [  0,           0,           1.00000000e+00 ]]
, dtype=np.float32)



# Open webcam
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect markers
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None:
        # Draw detected markers
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Estimate pose of each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
        
        for i in range(len(ids)):
            # Transform the camera pose (tvecs) to the world frame using the transform function
            X_c_new = tvecs[i].flatten()  # Extract translation vector
            X_w_new = transform_camera_to_world(X_c_new, R, T)
            
            # Draw frame axes (this will still be in camera coordinates, no transformation needed)
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)

            # Display transformed world coordinates on screen
            pos_text = f"ID: {ids[i][0]} World Pos: {X_w_new}"
            cv2.putText(frame, pos_text, tuple(corners[i][0][0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print(pos_text)
    
    # Display the resulting frame
    cv2.imshow('Aruco Marker Detection', frame)
    
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    

cap.release()
cv2.destroyAllWindows()
