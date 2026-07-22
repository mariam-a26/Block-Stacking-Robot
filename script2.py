import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
import numpy as np
import DobotDllType as dType
import time
import math
# Assuming ece486_starter_code.py is in the same directory
# from ece486_starter_code import initialize_robot, move_to_xyz, move_to_home

# --- Dobot API and Initialization ---
# Load the Dobot API
api = dType.load()

home_pos = [200,100,50]

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
    
"""
    Move the robot to the given x, y, z coordinates using PTP Linear XYZ Mode. This command will block until the motion
    is complete. You almost always want to run this rather than the straight SetPTPCmd, because you shouldn't be sending
    multiple motion commands to the robot without queueing them first, and we want to run everything in unqueued mode
"""
def move_to_xyz(api,x,y,z):
    cmdIndx = -1
    execCmd = dType.SetPTPCmd(api,dType.PTPMode.PTPMOVLXYZMode,x,y,z,0,isQueued=0)[0]
    #Allow the command to complete. The robot will stop moving when it's done
    while execCmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(25)

"""
    Move the robot to the given joint angles using PTP Linear ANGLE mode
    We will default J4 to zero, since it only matters if you have an end effector attached
"""
def move_joint_angles(api,J1,J2,J3,J4=0):
    cmdIndx = -1
    
    execCmd = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJANGLEMode, J1, J2, J3, J4, isQueued = 0)[0]
    #Allow the command to complete. The robot will stop moving when it's done
    while execCmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(25)

    
    
"""
    Move the robot to it's home position. Note: this will use basic PTP motion, rather than
    SetHOMECmd, since SetHOMECmd will re-run the sensor initialization stuff that we don't
    need during normal operation
"""
def move_to_home(api):
    move_to_xyz(api,home_pos[0],home_pos[1],home_pos[2])

def move_cartesian(api, x, y, z):
    if not -120 < z < 0:
        return "FAILURE"
    if not 120 <= math.sqrt(x**2 + y**2) <= 280:
        print(math.sqrt(x**2 + y**2))
        return "FAILURE"
    if x < 0:
        return "FAILURE"
    
    move_to_xyz(api, x, y, z)
    pos_x, pos_y, pos_z, _, _, _, _, _ = dType.GetPose(api)
    return (str(pos_x), str(pos_y), str(pos_z))

# Initialize the robot, connecting to it and setting up basic parameters
initialize_robot(api)

# --- Camera-to-World Transformation Matrices ---
# These R (Rotation) and T (Translation) matrices are assumed to be generated
# from your previous camera-robot calibration lab and saved as R.npy and T.npy.
# Ensure these files are in the same directory as this script.
R = np.load("R.npy")
T = np.load("T.npy")
# A small adjustment to the Z-component of the translation vector, as seen in find_aruco.py
# T[2] += 0.008

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
    and then lifts the pen back up. Records the actual robot pose at dot placement.
    Args:
        api (DobotDllType.api): The Dobot API instance.
        x (float): X-coordinate in world frame (mm).
        y (float): Y-coordinate in world frame (mm).
        z_up (float): Z-coordinate for pen up position (mm).
        z_down (float): Z-coordinate for pen down position (mm).
    Returns:
        tuple: Actual (x, y, z) position of the robot's tool frame when the dot was placed.
    """
    # Ensure pen is up before moving to new XY coordinates
    current_pose = dType.GetPose(api)
    if current_pose[2] < z_up: # If current Z is below the pen-up height, lift it first
        print(f"Lifting pen from current Z={current_pose[2]:.2f} to Z={z_up:.2f} before moving.")
        move_to_xyz(api, current_pose[0], current_pose[1], z_up)
        time.sleep(0.5) # Short delay for lift

    # Move to the target (x,y) with the pen lifted to z_up
    print(f"Moving to X={x:.2f}, Y={y:.2f}, Z={z_up:.2f} (pen up)")
    x *= 1000
    y *= 1000
    move_to_xyz(api, x, y, z_up)
    time.sleep(1) # Allow robot to settle at the position

    # Lower the pen to z_down to make contact and place the dot
    print(f"Lowering to X={x:.2f}, Y={y:.2f}, Z={z_down:.2f} (pen down)")
    move_to_xyz(api, x, y, z_down)
    time.sleep(1) # Allow pen to make contact and leave a mark

    # Get the actual robot pose at the moment the dot is placed
    actual_pose_at_dot = dType.GetPose(api)
    print(f"Actual robot pose at dot placement: X={actual_pose_at_dot[0]:.2f}, Y={actual_pose_at_dot[1]:.2f}, Z={actual_pose_at_dot[2]:.2f}")

    # Lift the pen back to z_up
    print(f"Lifting to X={x:.2f}, Y={y:.2f}, Z={z_up:.2f} (pen up)")
    move_to_xyz(api, x, y, z_up)
    time.sleep(1) # Lift pen off paper

    return actual_pose_at_dot[0], actual_pose_at_dot[1], actual_pose_at_dot[2]

def get_initial_marker_positions():
    """
    Captures a single frame from the webcam and detects all visible ArUco markers,
    returning their world coordinates. This is done once at the beginning of Part 2
    to avoid re-detecting markers during trajectory execution.
    Returns:
        dict: A dictionary mapping ArUco ID to its world coordinates [X, Y, Z].
    """
    cap = cv2.VideoCapture(0) # Open the default webcam
    if not cap.isOpened():
        print("Error: Could not open webcam for initial marker detection. Please check camera connection.")
        return {}

    print("Capturing initial marker positions... Ensure all ArUco markers on your printout are visible.")
    # Give camera a moment to warm up
    time.sleep(2)
    ret, frame = cap.read() # Read a single frame
    cap.release() # Release the camera immediately after capturing one frame
    cv2.destroyAllWindows() # Close any lingering OpenCV windows

    if not ret:
        print("Failed to grab frame for initial marker detection. No markers will be available.")
        return {}

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Convert frame to grayscale
    corners, ids, _ = detector.detectMarkers(gray) # Detect markers

    detected_markers_world_coords = {}
    if ids is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
        for i in range(len(ids)):
            marker_id = ids[i][0]
            X_c = tvecs[i].flatten() # Get translation vector in camera coordinates
            X_w = transform_camera_to_world(X_c, R, T) # Transform to world coordinates
            detected_markers_world_coords[marker_id] = X_w
            print(f"Detected ID: {marker_id} at World Pos: X={X_w[0]:.2f}, Y={X_w[1]:.2f}, Z={X_w[2]:.2f}")
    else:
        print("No ArUco markers detected in the initial frame. Please ensure they are visible.")

    return detected_markers_world_coords

def is_within_restricted_workspace(x, y, min_x, max_x, min_y, max_y):
    """
    Checks if a given (x, y) coordinate is within the dynamically restricted workspace.
    Args:
        x (float): X-coordinate to check.
        y (float): Y-coordinate to check.
        min_x (float): Minimum X boundary of the restricted workspace.
        max_x (float): Maximum X boundary of the restricted workspace.
        min_y (float): Minimum Y boundary of the restricted workspace.
        max_y (float): Maximum Y boundary of the restricted workspace.
    Returns:
        bool: True if the point is within the workspace, False otherwise.
    """
    return min_x <= x <= max_x and min_y <= y <= max_y

def main_part2():
    """
    Main function for Part 2: Simple Trajectories.
    Guides the robot through a sequence of ArUco markers, placing dots
    and adhering to workspace restrictions.
    """
    print("Starting Part 2: Simple Trajectories")
    print("This script will first scan for all visible ArUco markers to define the robot's restricted workspace.")
    print("Then, you will enter a sequence of marker IDs for the robot to visit.")

    # Step 1: Get initial marker positions and define the workspace
    detected_markers_map = get_initial_marker_positions()
    if not detected_markers_map:
        print("No markers detected. Cannot proceed with trajectory. Please ensure markers are visible and try again.")
        return

    # Calculate the restricted XY workspace based on the detected markers
    # Add a small buffer around the min/max detected coordinates
    all_x = [pos[0] for pos in detected_markers_map.values()]
    all_y = [pos[1] for pos in detected_markers_map.values()]

    if not all_x or not all_y:
        print("Could not determine workspace bounds from detected markers. Exiting.")
        return

    # Define workspace boundaries with a 10mm buffer around the detected markers
    min_x_workspace = min(all_x) - 10
    max_x_workspace = max(all_x) + 10
    min_y_workspace = min(all_y) - 10
    max_y_workspace = max(all_y) + 10

    print(f"\nDynamically Restricted XY Workspace: X from {min_x_workspace:.2f}mm to {max_x_workspace:.2f}mm, Y from {min_y_workspace:.2f}mm to {max_y_workspace:.2f}mm")
    print(f"Pen Z-heights: UP={Z_PEN_UP:.2f}mm, DOWN={Z_PEN_DOWN:.2f}mm")

    # Step 2: Get the trajectory sequence from the user
    trajectory_ids_str = input("Enter a sequence of ArUco marker IDs separated by spaces (e.g., '121 123 231 -1' to stop input): ").strip()
    trajectory_ids = []
    try:
        raw_ids = trajectory_ids_str.split()
        for s_id in raw_ids:
            num_id = int(s_id)
            if num_id < 0: # Negative number indicates end of input
                break
            trajectory_ids.append(num_id)
    except ValueError:
        print("Invalid input format. Please enter space-separated integers for marker IDs.")
        return

    if not trajectory_ids:
        print("No valid trajectory IDs entered. Exiting.")
        return

    print(f"\nTrajectory sequence received: {trajectory_ids}")
    print("\nExecuting trajectory...")

    recorded_data = [] # List to store data for each dot placement: (marker_id, camera_detected_pos, robot_actual_pen_pos)

    # Step 3: Execute the trajectory
    for i, marker_id in enumerate(trajectory_ids):
        print(f"\n--- Processing ArUco ID: {marker_id} ---")

        # Check if the marker was detected in the initial scan
        if marker_id not in detected_markers_map:
            print(f"ArUco ID {marker_id} was not detected in the initial scan. Skipping this marker.")
            continue

        # Get the target position from the initial scan results
        target_pos_world = detected_markers_map[marker_id]
        target_x, target_y, _ = target_pos_world # We only care about X and Y for movement, Z is controlled by Z_PEN_UP/DOWN

        # Check if the target position is within the restricted workspace
        if not is_within_restricted_workspace(target_x, target_y, min_x_workspace, max_x_workspace, min_y_workspace, max_y_workspace):
            print(f"Target position (X={target_x:.2f}, Y={target_y:.2f}) for ID {marker_id} is outside the restricted workspace. Skipping this marker.")
            continue

        print(f"Moving to target ID {marker_id} at (X={target_x:.2f}, Y={target_y:.2f})")

        # Move to marker, place dot, and record the actual robot position
        actual_x, actual_y, actual_z = place_dot(api, target_x, target_y, Z_PEN_UP, Z_PEN_DOWN)
        recorded_data.append({
            'marker_id': marker_id,
            'camera_detected_pos': target_pos_world.tolist(), # Convert numpy array to list for logging
            'robot_actual_pen_pos': [actual_x, actual_y, actual_z]
        })

        if i + 1 < len(trajectory_ids):
            if trajectory_ids[i + 1] != marker_id:
                # Move robot to home position (out of camera's view) before proceeding to the next marker
                print("Moving robot to home position (out of camera's view) before next move.")
                move_to_home(api)
        time.sleep(1) # Small delay before starting the next movement

    print("\nTrajectory execution complete.")
    print("\n--- Recorded Data Summary ---")
    if recorded_data:
        for data_point in recorded_data:
            print(f"ID: {data_point['marker_id']}")
            print(f"  Camera Detected (World): X={data_point['camera_detected_pos'][0]:.2f}, Y={data_point['camera_detected_pos'][1]:.2f}, Z={data_point['camera_detected_pos'][2]:.2f}")
            print(f"  Robot Actual Pen Tip:    X={data_point['robot_actual_pen_pos'][0]:.2f}, Y={data_point['robot_actual_pen_pos'][1]:.2f}, Z={data_point['robot_actual_pen_pos'][2]:.2f}")
            print("-" * 20)
    else:
        print("No markers were successfully visited during the trajectory.")

    print("Part 2 script finished.")

if __name__ == "__main__":
    main_part2()
