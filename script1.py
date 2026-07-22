import cv2
import numpy as np
import DobotDllType as dType
import time
import math
# Assuming ece486_starter_code.py is in the same directory
# from ece486_starter_code import move_to_xyz, move_to_home

home_pos = [200,100,50]

def move_to_xyz(api,x,y,z):
    cmdIndx = -1
    execCmd = dType.SetPTPCmd(api,dType.PTPMode.PTPMOVLXYZMode,x,y,z,0,isQueued=0)[0]
    #Allow the command to complete. The robot will stop moving when it's done
    while execCmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(25)

def move_to_home(api):
    move_to_xyz(api,home_pos[0],home_pos[1],home_pos[2])

def move_cartesian(api, x, y, z):
    # if not -120 < z < 0:
    #     print("1")
    #     return "FAILURE"
    # if not 120 <= math.sqrt(x**2 + y**2) <= 280:
    #     print("2")
    #     print(math.sqrt(x**2 + y**2))
    #     return "FAILURE"
    if x < 0:
        print("3")
        return "FAILURE"
    
    move_to_xyz(api, x, y, z)
    pos_x, pos_y, pos_z, _, _, _, _, _ = dType.GetPose(api)
    return (str(pos_x), str(pos_y), str(pos_z))

def initialize_robot(api):
    #detect the robot's com port
    com_port = dType.SearchDobot(api)
    print(dType.SearchDobot(api))
    #if we can't find it, then we can't continue, so exit
    if "COM" not in com_port[0]:
        print("Error: The robot either isn't on or isn't responding. Exiting now")
        exit()
    
    
    #we've found it, so let's try to connect
    state = dType.DobotConnect.DobotConnect_NoError
    for i in range(0,len(com_port)):
        state_full = dType.ConnectDobot(api, com_port[i], 115200)
        state = state_full[0]
        print("STATE FULL:")
        print(state_full)
        #If the connection failed at this point, we also can't proceed, so we need to exit
        if state == dType.DobotConnect.DobotConnect_NoError:
            print("Connected!")
            name = dType.GetDeviceName(api)
            if name[0] == "Not a dobot":
                dType.DisconnectDobot(api)
                continue
            else:
                break
            
    if state != dType.DobotConnect.DobotConnect_NoError:
            print("Can not connect! Exiting")
            exit()    
    """
        stop any queued commands and clear the queue. You HAVE TO do this every time you initialize the robot
        If there are queued commands in the queue, then they will execute first. This can
        cause the robot to go well outside of its allowable range. The simplest way to do this
        is to stop anything that might be running or might try to run, then clear the queue.
        
        Other than at startup, during normal operation you shouldn't have to do this.
    """
    dType.SetQueuedCmdStopExec(api)
    dType.SetQueuedCmdClear(api)
    
    #Set the robot's max speed and acceleration. We're keeping these to 50% of max for safety
    dType.SetPTPCommonParams(api, 50, 50, isQueued=1)
    
    """
        Home the robot. 
    """
    #Set the home position
    dType.SetHOMEParams(api, home_pos[0], home_pos[1], home_pos[2], 0, isQueued=1)
    
    cmdIndx = -1
    """
        Enqueue the home command. This command always begins by moving the robot back to an initialization
        position so that the encoders are reset, then it will move the robot to its home position,
        and finally it will undergo a quick procedure to validate that its encoders are properly set. You definitely
        want to run this every time you initialize the robot
    """
    execCmd = dType.SetHOMECmd(api, temp=0, isQueued=1)[0]
    
    #Execute the three enqueued commands: set the speed/acceleration, set the home position, and move to home
    dType.SetQueuedCmdStartExec(api)
    
    #Allow the homing command to complete. The robot will beep and the LED will turn green
    #when it's ready to go
    while execCmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(25)
        
    #OK, the robot is ready to move!

# --- Dobot API and Initialization ---
# Load the Dobot API
api = dType.load()
# Initialize the robot, connecting to it and setting up basic parameters
initialize_robot(api)

# --- Camera-to-World Transformation Matrices ---
# These R (Rotation) and T (Translation) matrices are assumed to be generated
# from your previous camera-robot calibration lab and saved as R.npy and T.npy.
# Ensure these files are in the same directory as this script.
R = np.load("R.npy")
T = np.load("T.npy")
# A small adjustment to the Z-component of the translation vector, as seen in find_aruco.py
T[0] += 0.006
T[1] += 0.003

# --- Camera Calibration Parameters ---
# IMPORTANT: Replace these with the actual camera_matrix and dist_coeffs
# you obtained from your camera calibration. These are placeholder values
# from the provided find_aruco.py script.
# Liv
camera_matrix = np.array([[487.30365685,   0,        325.0095482 ],
 [  0,         460.73933611, 235.68433861],
 [  0,           0,           1        ]]
, dtype=np.float32)
                          
# You should also put the distortion coefficients here                          
dist_coeffs = np.array([[ 0.04385641, -0.0177826,  -0.04588679,  0.00081661, -0.0048952 ]], dtype=np.float32).T

# --- ArUco Detector Setup ---
# Define the ArUco dictionary (4x4 with 50 IDs) and detector parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# --- Pen Z-heights ---
# IMPORTANT: Adjust these values based on your specific pen setup and the height
# of your paper/table. You should use Dobot Link to manually find these Z-coordinates.
# Z_PEN_UP: The Z-coordinate where the pen is lifted off the paper (e.g., 5mm above contact).
# Z_PEN_DOWN: The Z-coordinate where the pen makes contact with the paper (e.g., 0.5cm lower than Z_PEN_UP).
Z_PEN_UP = 25 # Example: 5mm above the paper surface
Z_PEN_DOWN = 16 # Example: 5mm below Z_PEN_UP, allowing for spring travel

def transform_camera_to_world(X_c_new, R, T):
    """
    Transforms a 3D point from camera coordinates to the robot's world coordinates.
    Args:
        X_c_new (np.array): 3D point in camera coordinates (e.g., translation vector from ArUco pose estimation).
        R (np.array): 3x3 Rotation matrix from camera frame to world frame.
        T (np.array): 3x1 Translation vector from camera frame to world frame.
    Returns:
        np.array: 3D point in world coordinates.
    """
    return R @ X_c_new + T

def place_dot(api, x, y, z_up, z_down):
    """
    Moves the robot to a specified (x, y) position, lowers the pen to place a dot,
    and then lifts the pen back up.
    Args:
        api (DobotDllType.api): The Dobot API instance.
        x (float): X-coordinate in world frame (mm).
        y (float): Y-coordinate in world frame (mm).
        z_up (float): Z-coordinate for pen up position (mm).
        z_down (float): Z-coordinate for pen down position (mm).
    """
    # Move to the target (x,y) with the pen lifted to z_up
    x *= 1000
    y *= 1000
    # z_up *= 100
    # z_down *= 100
    print(f"Moving to X={x:.2f}, Y={y:.2f}, Z={z_up:.2f} (pen up)")
    move_cartesian(api, x, y, z_up)
    time.sleep(1) # Allow robot to settle at the position

    # Lower the pen to z_down to make contact and place the dot
    print(f"Lowering to X={x:.2f}, Y={y:.2f}, Z={z_down:.2f} (pen down)")
    move_cartesian(api, x, y, z_down)
    time.sleep(1) # Allow pen to make contact and leave a mark

    # Lift the pen back to z_up
    print(f"Lifting to X={x:.2f}, Y={y:.2f}, Z={z_up:.2f} (pen up)")
    move_cartesian(api, x, y, z_up)
    time.sleep(1) # Lift pen off paper

def main_part1():
    """
    Main function for Part 1: Calibration Adjustment.
    Continuously detects ArUco markers and allows the user to command the robot
    to move to a detected marker's center to place a dot.
    """
    print("Starting Part 1: Calibration Adjustment")
    print("Ensure your ArUco printout is placed within the robot's workspace.")
    print("Press 'q' in the OpenCV window or type 'q' in the console to quit.")
    print("Type 'r' in the console to refresh the camera view and re-detect markers.")

    cap = cv2.VideoCapture(0) # Open the default webcam (usually 0)
    if not cap.isOpened():
        print("Error: Could not open webcam. Please check camera connection and permissions.")
        return

    while True:
        ret, frame = cap.read() # Read a frame from the camera
        if not ret:
            print("Failed to grab frame from camera. Exiting.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Convert frame to grayscale for marker detection
        corners, ids, _ = detector.detectMarkers(gray) # Detect ArUco markers

        current_frame_markers = {} # Dictionary to store detected marker IDs and their world coordinates
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids) # Draw detected markers on the frame
            # Estimate the pose of each detected marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)

            for i in range(len(ids)):
                marker_id = ids[i][0]
                X_c = tvecs[i].flatten() # Get the translation vector (position) in camera coordinates
                X_w = transform_camera_to_world(X_c, R, T) # Transform to world coordinates
                current_frame_markers[marker_id] = X_w # Store the world coordinates
                
                # Display the marker ID and its world position on the OpenCV window
                pos_text = f"ID: {marker_id} World Pos: X={X_w[0]:.2f}, Y={X_w[1]:.2f}, Z={X_w[2]:.2f}"
                cv2.putText(frame, pos_text, tuple(corners[i][0][0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                print(pos_text) # Also print to console

        cv2.imshow('ArUco Marker Detection (Part 1)', frame) # Display the camera feed

        # Handle keyboard input for quitting or refreshing
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Quitting camera feed.")
            break

        # Prompt user for action in the console
        user_input = input("Enter ArUco ID to move to (or 'q' to quit, 'r' to refresh camera view): ").strip()
        if user_input.lower() == 'q':
            break
        elif user_input.lower() == 'r':
            print("Refreshing camera view and re-detecting markers...")
            continue # Continue loop to get a new frame and re-scan

        try:
            target_id = int(user_input)
            if target_id in current_frame_markers:
                target_pos_world = current_frame_markers[target_id]
                print(f"Attempting to move to ArUco ID {target_id} at world coordinates: X={target_pos_world[0]:.2f}, Y={target_pos_world[1]:.2f}, Z={target_pos_world[2]:.2f}")

                # Move to the detected marker, place a dot, then move to home position
                place_dot(api, target_pos_world[0], target_pos_world[1], Z_PEN_UP, Z_PEN_DOWN)
                print("Moving robot to home position (out of camera's view).")
                move_to_home(api)
                print("Robot moved to home. Observe the dot and adjust calibration if needed. Re-run this script for further adjustments.")
            else:
                print(f"ArUco ID {target_id} was not detected in the current frame. Please ensure it's visible and try again.")
        except ValueError:
            print("Invalid input. Please enter an integer ID, 'q', or 'r'.")
        except Exception as e:
            print(f"An error occurred during robot movement: {e}")
            break # Exit on unexpected errors

    cap.release() # Release the camera resource
    cv2.destroyAllWindows() # Close all OpenCV windows
    print("Part 1 script finished.")

if __name__ == "__main__":
    main_part1()
