import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import numpy as np
import cv2

def main():
    # Termination criteria for refining corner locations
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # Define the chessboard dimensions (number of inner corners per chessboard row and column)
    chessboard_size = (9, 6)  # adjust if necessary
    
    # Prepare object points based on the known chessboard dimensions.
    # For example, (0,0,0), (1,0,0), (2,0,0), ..., (8,5,0)
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

    # Arrays to store the object points and image points from all the images.
    objpoints = []  # 3D points in real-world space
    imgpoints = []  # 2D points in image plane

    # Open the default camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return

    print("Press 'c' to capture an image (when the chessboard is visible).")
    print("Press 'q' to quit capturing and compute calibration.")

    captured_images = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Try to find the chessboard corners
        ret_corners, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        display = frame.copy()
        if ret_corners:
            # If found, draw the corners on the image
            cv2.drawChessboardCorners(display, chessboard_size, corners, ret_corners)
        
        cv2.putText(display, f'Images captured: {captured_images}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Camera Calibration', display)
        
        key = cv2.waitKey(1)
        if key & 0xFF == ord('c'):
            if ret_corners:
                # Refine corner detection and store points for calibration
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                objpoints.append(objp)
                imgpoints.append(corners2)
                captured_images += 1
                print(f"Captured image {captured_images}")
            else:
                print("Chessboard not detected. Please adjust your view and try again.")
        elif key & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    # Check if enough images were captured (at least 5 is recommended)
    if captured_images < 5:
        print("Not enough images for calibration. Capture at least 5 good images.")
        return

    # Perform camera calibration to get the camera matrix and distortion coefficients
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)
    
    print("\nCalibration complete.")
    print("Camera Matrix:")
    print(camera_matrix)
    print("\nDistortion Coefficients:")
    print(dist_coeffs)
    
    # Optionally, save calibration parameters to a file for later use
    np.savez("calibration_data.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs,
             rvecs=rvecs, tvecs=tvecs)
    print("\nCalibration data saved to 'calibration_data.npz'.")

if __name__ == "__main__":
    main()
