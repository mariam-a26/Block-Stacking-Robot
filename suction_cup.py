import DobotDllType as dType
import time
import numpy as np
import threading
import cv2

#Useful global variables
# --- These are status strings that you might see, so we're defining them here ---
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

#always begin with this line, or you can't connect to the robot at all. Just don't
#remove this line and keep it at the top of your code
api = dType.load()

"""
These coordinates are to the left of the robot's x axis and slight above the xy plane, viewed from
the top. This is a useful home position when dealing with the vision labs, since it moves
the robot out of the way. You can change the coordinates here if you really want.
"""
home_pos = [200,100,50]

R = np.load("R.npy")
T = np.load("T.npy")

# Calibration adjustments
# T[0] += 0.006
# T[1] += 0.003

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
Z_PEN_UP = 25
Z_PEN_DOWN = 16

# --- Block Height for Stacking ---
BLOCK_HEIGHT = 15

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
    
    
def rotate_end_effector(api,angle):
    if angle <= 90 and angle >= -90:
        pose = dType.GetPose(api)
        cmdIndx = -1
        execCmd = dType.SetPTPCmd(api,dType.PTPMode.PTPMOVLXYZMode,pose[0],pose[1],pose[2],angle,isQueued=0)[0]
        #Allow the command to complete. The robot will stop moving when it's done
        while execCmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
            dType.dSleep(25)
        
def release_suction(api):
    #arguments are: api, enable control = 1, state=0 "off", isQueued = 0
    dType.SetEndEffectorSuctionCup(api,1,0,0)[0]
    #This command just gets sent, there is no feedback, so we need to wait until the pump turns off
    #We don't need to wait as long as we do for the gripper
    dType.dSleep(50)

def engage_suction(api):
    #arguments are: api, enable control = 1, state=1 "on", isQueued = 0
    dType.SetEndEffectorSuctionCup(api,1,1,0)[0]
    #This command just gets sent, there is no feedback, so we need to wait until the pump turns off
    #We don't need to wait as long as we do for the gripper
    dType.dSleep(50)
    
    
def stop_pump(api):
    #Yeah, I know it says suction cup. it's actually controlling the pneumatic pump
    dType.SetEndEffectorSuctionCup(api,1,0,0)[0]
    #This command just gets sent, there is no feedback, so we need to wait until the pump turns off
    #We don't need to wait as long as we do for the gripper
    dType.dSleep(50)

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

def detect_red_blocks(frame):
    """
    Detects red colored blocks in the given frame.
    Returns a list of (x, y) pixel coordinates of the block centroids.
    """
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range for red color in HSV
    # These values might need tuning based on lighting and specific red color
    # Lower red range (for full red, may be split across 0 and 180 in H)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)

    # Upper red range
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)

    red_mask = mask1 + mask2

    # Morphological operations to clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.erode(red_mask, kernel, iterations=2)
    red_mask = cv2.dilate(red_mask, kernel, iterations=2)

    # Find contours in the mask
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    red_block_positions = []
    min_area = 500 # Minimum contour area to consider as a block (tune this)
    max_area = 50000 # Maximum contour area (tune this)

    for contour in contours:
        area = cv2.contourArea(contour)
        if min_area < area < max_area:
            # Calculate moments for centroid
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                red_block_positions.append((cX, cY))
                cv2.circle(frame, (cX, cY), 5, (0, 255, 255), -1) # Draw centroid
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2) # Draw contour

    return red_block_positions

def build_tower():
    print("Starting red block detection and stacking.")
    print("Ensure your ArUco printout is placed within the robot's workspace.")
    print("Place red blocks within the camera's view.")
    print("Press 'q' in the OpenCV window to quit.")

    cap = cv2.VideoCapture(0) # Open the default webcam (usually 0)
    if not cap.isOpened():
        print("Error: Could not open webcam. Please check camera connection and permissions.")
        return

    try:
        current_stack_height = 0 # Keep track of how many blocks are stacked

        while True:
            ret, frame = cap.read() # Read a frame from the camera
            if not ret:
                print("Failed to grab frame from camera. Exiting.")
                break

            # --- Detect ArUco Markers ---
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            min_aruco_id = float('inf')
            target_aruco_pos_world = None
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)

                detected_aruco_markers = {}
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    X_c = tvecs[i].flatten()
                    X_w = transform_camera_to_world(X_c, R, T)
                    detected_aruco_markers[marker_id] = X_w
                    
                    pos_text = f"ID: {marker_id} World Pos: X={X_w[0]:.2f}, Y={X_w[1]:.2f}, Z={X_w[2]:.2f}"
                    cv2.putText(frame, pos_text, tuple(corners[i][0][0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if detected_aruco_markers:
                    min_aruco_id = min(detected_aruco_markers.keys())
                    target_aruco_pos_world = detected_aruco_markers[min_aruco_id]
                    print(f"\nTarget ArUco Marker (lowest ID): {min_aruco_id} at X={target_aruco_pos_world[0]:.2f}, Y={target_aruco_pos_world[1]:.2f}, Z={target_aruco_pos_world[2]:.2f}")
                else:
                    print("No ArUco markers detected. Cannot determine stacking target.")
                    cv2.imshow('ArUco Marker and Red Block Detection', frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    continue # Skip to next frame if no target marker

            else:
                print("No ArUco markers detected. Cannot determine stacking target.")
                cv2.imshow('ArUco Marker and Red Block Detection', frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                continue # Skip to next frame if no target marker


            # --- Detect Red Blocks ---
            red_block_pixel_coords = detect_red_blocks(frame)
            if red_block_pixel_coords:
                print(f"Detected {len(red_block_pixel_coords)} red blocks.")

                if target_aruco_pos_world is not None:
                    print("Initiating stacking sequence...")

                    # Project 2D red block pixels into 3D (camera frame)
                    red_block_world_coords = []

                    for u, v in red_block_pixel_coords:
                        # 2D pixel to normalized image plane
                        pixel = np.array([[u], [v], [1]])
                        ray = np.linalg.inv(camera_matrix) @ pixel  # direction of ray in camera frame

                        # Assume block lies on Z = plane_z in CAMERA frame
                        # You can get this from tvecs[0][2] if ArUco is on same surface
                        assumed_plane_z = tvecs[0][0][2]  # Z in camera frame of ArUco marker (adjust as needed)

                        scale = assumed_plane_z / ray[2][0]  # scale ray so z = plane_z
                        point_camera = ray * scale  # (x, y, z) in camera coordinates

                        # Convert to world coordinates
                        point_world = transform_camera_to_world(point_camera, R, T)
                        red_block_world_coords.append(point_world.flatten())

                    for i, block_pos in enumerate(red_block_world_coords):
                        print(f"Picking up Block {i+1} at X={block_pos[0]:.2f}, Y={block_pos[1]:.2f}")
                        
                        # Move to above the block
                        move_to_xyz(api, block_pos[0], block_pos[1], Z_PEN_UP)
                        # Move down to pick up
                        move_to_xyz(api, block_pos[0], block_pos[1], Z_PEN_DOWN)
                        engage_suction(api)
                        dType.dSleep(500) # Wait for suction to engage

                        # Lift block
                        move_to_xyz(api, block_pos[0], block_pos[1], Z_PEN_UP)
                        dType.dSleep(200) # Small delay

                        # Calculate target Z for stacking
                        # Add target_aruco_pos_world[2] to account for the base height of the marker.
                        target_z_stack = target_aruco_pos_world[2] + Z_PEN_DOWN + (current_stack_height * BLOCK_HEIGHT)
                        print(f"Placing Block {i+1} on stack at target Z: {target_z_stack:.2f}")

                        # Move to above the stacking location
                        move_to_xyz(api, target_aruco_pos_world[0], target_aruco_pos_world[1], target_z_stack + Z_PEN_UP)
                        # Move down to place
                        move_to_xyz(api, target_aruco_pos_world[0], target_aruco_pos_world[1], target_z_stack)
                        release_suction(api)
                        dType.dSleep(500) # Wait for suction to release

                        # Lift off the stack
                        move_to_xyz(api, target_aruco_pos_world[0], target_aruco_pos_world[1], Z_PEN_UP)
                        
                        current_stack_height += 1
                        print(f"Stack height is now: {current_stack_height} blocks.")
                        move_to_home(api) # Return to home position after each block

                    print("All detected red blocks have been stacked!")
                    stop_pump(api) # Ensure pump is off at the end
                    break # Exit after stacking all blocks

            cv2.imshow('ArUco Marker and Red Block Detection', frame) # Display the camera feed

            # Handle keyboard input for quitting
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Quitting camera feed.")
                break

    finally:
        cap.release() # Release the camera resource
        cv2.destroyAllWindows() # Close all OpenCV windows
        stop_pump(api) # Ensure pump is off if we exit early
    
    
#Before running and commands, always run this
initialize_robot(api)

build_tower()

# """
#     Here is a sample script rotates the end effector, the engages and disengages suction a few times, and finally
#     ensures the pump is off
# """
# rotate_end_effector(api,90)
# rotate_end_effector(api,-90)
# rotate_end_effector(api,0)
# rotate_end_effector(api,180) #this should just not move!

# engage_suction(api)
# dType.dSleep(2000) #otherwise we can't tell if it's working
# release_suction(api)
# dType.dSleep(2000) #otherwise we can't tell if it's working
# engage_suction(api)
# dType.dSleep(2000) #otherwise we can't tell if it's working
# release_suction(api)
# stop_pump(api)



