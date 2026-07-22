import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
import numpy as np
import re
import time
from uw_llm import generate_vision
import DobotDllType as dType
import threading
from PIL import Image, ImageDraw #pip install pillow
import json

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
#update x by 0.008
T[0] += 0.013
T[1] += 0.010

# Calibration adjustments
# T[0] += 0.006
# T[1] += 0.003

# --- Camera Calibration Parameters ---
# IMPORTANT: Replace these with the actual camera_matrix and dist_coeffs
# you obtained from your camera calibration. These are placeholder values
# from the provided find_aruco.py script.
# Liv
dist_coeffs = np.array([[ 0.0263835,  -0.03901205, -0.00010248, -0.00059746, -0.0594707 ]], dtype=np.float32).T

# Camera calibration (Replace with actual values you obtained from your camera calibration)
camera_matrix = np.array([[595.92330163,   0.     ,    257.1158604 ],
 [  0.    ,     590.58436344, 206.10123259],
 [  0.  ,         0.    ,       1.        ]]
, dtype=np.float32)



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
Z_PEN_UP = 30
Z_PEN_DOWN = -45

# --- Block Height for Stacking ---
BLOCK_HEIGHT = 0

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

# Step 1: Detect all colored blocks (basic HSV)
def detect_colored_blocks(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masks = {}
    
    # Define color ranges (tune if needed)
    colors = {
        'red':   ([0, 100, 100], [10, 255, 255], [160, 100, 100], [180, 255, 255]),
        'blue':  ([100, 150, 0], [140, 255, 255]),
        'green': ([40, 70, 70], [80, 255, 255])
    }

    positions = []

    for color, bounds in colors.items():
        if color == "red":
            lower1, upper1, lower2, upper2 = [np.array(b) for b in bounds]
            mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
        else:
            lower, upper = [np.array(b) for b in bounds]
            mask = cv2.inRange(hsv, lower, upper)

        # Clean up
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if 500 < area < 50000:
                M = cv2.moments(c)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                positions.append((color, cx, cy))
                cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
                cv2.putText(frame, f"{color} ({cx},{cy})", (cx+5, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

    return positions

# Step 2: Parse LLM response like "Move red block at (123, 456) to (300, 150)"
def parse_llm_response(response):
    instructions = []
    matches = re.findall(r"([a-z]+) block at \((\d+),\s*(\d+)\) to \((\d+),\s*(\d+)\)", response, re.IGNORECASE)
    for match in matches:
        color, x1, y1, x2, y2 = match
        instructions.append({
            'color': color.lower(),
            'src_pixel': (int(x1), int(y1)),
            'dst_pixel': (int(x2), int(y2))
        })
    print(instructions)
    return instructions

# Step 3: Convert pixel → camera → world (Z inferred from marker plane)
def pixel_to_world(u, v, Zc, camera_matrix, dist_coeffs, R, T):
    """
    Converts 2D pixel coordinates (u, v) to 3D world coordinates (Xw, Yw, Zw),
    including distortion correction.

    This function first undistorts the pixel coordinates using the camera matrix
    and distortion coefficients. Then, it converts the undistorted pixel coordinates
    to 3D camera coordinates using an assumed depth (Zc). Finally, it uses the
    provided camera_to_world_func to transform these camera coordinates into
    world coordinates.

    Args:
        u (float): The x-coordinate of the pixel in the image plane.
        v (float): The y-coordinate of the pixel in the image plane.
        Zc (float): The depth of the point in the camera's coordinate system.
                    This is crucial as a 2D pixel alone cannot determine 3D position.
        camera_matrix (np.array): A 3x3 NumPy array representing the camera's
                                  intrinsic matrix (K).
                                  Typically: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
        dist_coeffs (np.array): A NumPy array representing the distortion coefficients
                                (k1, k2, p1, p2, k3, ...).
        R (np.array): A 3x3 NumPy array representing the rotation matrix
                      from camera to world coordinates.
        T (np.array): A 3x1 NumPy array representing the translation vector
                      from camera to world coordinates.
        camera_to_world_func (function): A function that takes camera coordinates,
                                         R, and T, and returns world coordinates.

    Returns:
        np.array: A 3x1 NumPy array representing the (Xw, Yw, Zw) coordinates
                  in the world frame.
    """
    # Ensure inputs are NumPy arrays
    camera_matrix = np.array(camera_matrix).reshape(3, 3)
    dist_coeffs = np.array(dist_coeffs)
    R = np.array(R).reshape(3, 3)
    T = np.array(T).reshape(3, 1)

    # 1. Undistort the pixel coordinates
    # Reshape pixel coordinates for cv2.undistortPoints: (N, 1, 2)
    pixel_coords_distorted = np.array([[u, v]], dtype=np.float32).reshape(-1, 1, 2)

    # Undistort points. This returns normalized (x, y) coordinates in the camera frame
    # as if there were no distortion.
    # The output is typically (N, 1, 2) where N is the number of points.
    undistorted_points = cv2.undistortPoints(
        pixel_coords_distorted,
        camera_matrix,
        dist_coeffs,
        P=camera_matrix # Use the same camera matrix for projection after undistortion
    )

    # Extract the undistorted pixel coordinates (u_undistorted, v_undistorted)
    u_undistorted = undistorted_points[0, 0, 0]
    v_undistorted = undistorted_points[0, 0, 1]

    # Extract focal lengths and principal point from the camera matrix for clarity
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    # 2. Convert undistorted pixel coordinates to 3D camera coordinates (Xc, Yc, Zc)
    # These are now effectively "ideal" pixel coordinates.
    # Xc = (u_undistorted - cx) * Zc / fx
    # Yc = (v_undistorted - cy) * Zc / fy
    # However, cv2.undistortPoints with P=camera_matrix already gives you points
    # that are effectively in pixel coordinates (scaled by fx, fy and shifted by cx, cy)
    # as if they were projected by an ideal pinhole camera.
    # So, we can directly use the formula:
    Xc = (u_undistorted - cx) * Zc / fx
    Yc = (v_undistorted - cy) * Zc / fy
    # Zc is already known and provided as an argument

    camera_coords = np.array([Xc, Yc, Zc]).reshape(3, 1)

    # 3. Convert camera coordinates to world coordinates using the provided function
    world_coords = transform_camera_to_world(camera_coords, R, T)

    return world_coords

# Step 4: Execute action
def execute_move(src_world, dst_world, num_blocks):
    # Move above source
    print("src_world from fctn: ",  src_world)
    move_to_xyz(api, src_world[0], src_world[1], Z_PEN_UP)
    move_to_xyz(api, src_world[0], src_world[1], Z_PEN_DOWN)
    engage_suction(api)
    dType.dSleep(500)

    print("done")

    # Lift
    move_to_xyz(api, src_world[0], src_world[1], Z_PEN_UP + 30*num_blocks)

    # Move above destination
    move_to_xyz(api, dst_world[0], dst_world[1], Z_PEN_UP + 30*num_blocks)
    move_to_xyz(api, dst_world[0], dst_world[1], Z_PEN_DOWN + 30*num_blocks)
    release_suction(api)
    move_to_xyz(api, dst_world[0], dst_world[1], Z_PEN_UP + 30*num_blocks)
    dType.dSleep(500)


def resize_image(input_path, output_path, scale=0.25):
    with Image.open(input_path) as img:
        new_size = (int(img.width * scale), int(img.height * scale))
        resized_img = img.resize(new_size)
        resized_img.save(output_path)

def boundingBox():
    identify_prompt = """
    You are picture with an orange block and a green block as well as ArUco markers on it. Output a bounding box around the orange block and one around the green block in JSON format. Use the labels "orange" and "green". Output ONLY the bounding boxes in the format:

    {"bbox_2d": [x0, y0, x1, y1], "label": "orange"}

    """

    input_path = "."
    resized_path = "."
    resized_img_name = "objects_resized.JPG"
    img = "objects.JPG"

    """
    We need to resize the image because the GPU we are running the model on doesn't QUITE have enough memory
    to run on a full resolution image of our cameras. 0.75x scale is fine. HOWEVER, this means that you need to 
    move the bounding box to the right place - since all pixel coordinates are scaled by 0.75, you need to re-map
    to the actual pixel coordinates before trying to pick stuff up or you will be in the wrong place.
    """

    resize_image(img,resized_img_name,0.75)
    resized_img_path = os.path.join(resized_path, resized_img_name)

    response = generate_vision(identify_prompt, resized_img_path, fast=False)
    print(response)
    bbox_obj = json.loads(response.strip("`\n json"))  # Now expects a single dict, not a list

    # --- Draw bounding box on the image ---
    with Image.open(resized_img_path) as image:
        for i in range(len(bbox_obj)):
            draw = ImageDraw.Draw(image)
            box = bbox_obj[i]["bbox_2d"]
            label = bbox_obj[i]["label"]
            draw.rectangle(box, outline="red", width=3)
            draw.text((box[0] + 10, box[1] + 10), label, fill="red")

        output_img_path = "boxed_" + img
        image.save(output_img_path)
        print(f"Saved final image with box to {output_img_path}")

    # Save the sub-image
    base_name, ext = os.path.splitext(img)
    with Image.open(resized_img_path) as image:
        out_name = f"{label}_0_{base_name}{ext}"
        out_path = os.path.join(resized_path, out_name)
        region = image.crop(box)
        region.save(out_path)
        print(f"Saved {label} region to {out_path}")

    return response


def run_llm_robot_task():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Camera error")
        return

    initialize_robot(api)
    # move_to_home(api)

    # Step 1: Capture frame
    ret, frame = cap.read()
    if not ret:
        print("Frame error")
        return

    # Optional: Save for LLM vision
    cv2.imwrite("objects.JPG", frame)

    object_json_response = boundingBox()
    bBoxData = json.loads(object_json_response.strip("`\n json"))
    bBoxCorners = []
    bBoxLabel = []
    for i in range(len(bBoxData)):
        bBoxCorners.append(bBoxData[i]["bbox_2d"])
        bBoxLabel.append(bBoxData[i]["label"])

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect markers
    corners, ids, _ = detector.detectMarkers(gray)

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)

    world_pos = []

    for i in range(len(ids)):
        # Transform the camera pose (tvecs) to the world frame using the transform function
        X_c_new = tvecs[i].flatten()  # Extract translation vector
        X_w_new = transform_camera_to_world(X_c_new, R, T)
        world_pos.append(X_w_new)

        # Display transformed world coordinates on screen
        pos_text = f"ID: {ids[i][0]} World Pos: {X_w_new}"
        print(pos_text)

    
    # Step 4: Estimate Z from ArUco marker
    Z_plane = tvecs[0][0][2]

    src_world = []
    for i in range(len(bBoxCorners)):
        src_pixel = ((bBoxCorners[i][0] + bBoxCorners[i][2])/2, (bBoxCorners[i][1] + bBoxCorners[i][3])/2)
        src_world.append(pixel_to_world(src_pixel[0]/0.75, src_pixel[1]/0.75, Z_plane, camera_matrix, dist_coeffs, R, T))

    print("src_world: ", src_world)

    # execute_move(src_world*1000, world_pos[2]*1000)
    # execute_move(world_pos[2]*1000, world_pos[0]*1000)
    # # Step 2: Ask LLM
    # prompt = "Give movement instructions to stack the green and then the orange block on top of each other on top of aruco marker with ID 3. Format each instruction like: 'Move orange block at (x1, y1) to (x2, y2)'"
    # llm_response = generate_vision(prompt, "boxed_objects.JPG", fast=True)
    # print("LLM Response:\n", llm_response)

    # # # Step 3: Parse instructions
    # instructions = parse_llm_response(llm_response)

    # # Step 5: Execute actions
    num = 0
    for i in range(len(src_world)):
        # src_world = pixel_to_world(inst['src_pixel'][0]/0.75, inst['src_pixel'][1]/0.75, Z_plane, camera_matrix, dist_coeffs, R, T)
        # dst_world = pixel_to_world(inst['dst_pixel'][0]/0.75, inst['dst_pixel'][1]/0.75, Z_plane, camera_matrix, dist_coeffs, R, T)
        # print(f"Moving {inst['color']} block from {src_world} to {dst_world}")
        execute_move(src_world[i]*1000, world_pos[2]*1000, num)
        num += 1

    cap.release()
    cv2.destroyAllWindows()
    stop_pump(api)

"""
    Here is a sample script rotates the end effector, the engages and disengages suction a few times, and finally
    ensures the pump is off
"""
# initialize_robot(api)
# move_to_home(api)

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

if __name__ == "__main__":
    run_llm_robot_task()
