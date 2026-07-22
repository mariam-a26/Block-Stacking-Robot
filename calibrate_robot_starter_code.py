import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
import numpy as np
import DobotDllType as dType
import time
import threading
import math

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
    
#init robot
initialize_robot(api)

# Initialize ArUco detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Camera calibration (Replace with actual values you obtained from your camera calibration)
# Arya
# camera_matrix = np.array([[623.30725699,  0,         373.38443498],
#  [  0,         612.31049678, 303.31577113],
#  [  0,           0,           1        ]]
# , dtype=np.float32)
                          
# # You should also put the distortion coefficients here                          
# dist_coeffs = np.array([[ -1.84651048,  10.04703071,   0.12345707,   0.17785339, -26.70846071]], dtype=np.float32).T
# You should also put the distortion coefficients here                          
dist_coeffs = np.array([[ 0.0263835,  -0.03901205, -0.00010248, -0.00059746, -0.0594707 ]], dtype=np.float32).T

# Camera calibration (Replace with actual values you obtained from your camera calibration)
camera_matrix = np.array([[595.92330163,   0.     ,    257.1158604 ],
 [  0.    ,     590.58436344, 206.10123259],
 [  0.  ,         0.    ,       1.        ]]
, dtype=np.float32)


# # Liv
# camera_matrix = np.array([[487.30365685,   0,        325.0095482 ],
#  [  0,         460.73933611, 235.68433861],
#  [  0,           0,           1        ]]
# , dtype=np.float32)
                          
# # You should also put the distortion coefficients here                          
# dist_coeffs = np.array([[ 0.04385641, -0.0177826,  -0.04588679,  0.00081661, -0.0048952 ]], dtype=np.float32).T

# [[487.30365685   0.         325.0095482 ]
#  [  0.         460.73933611 235.68433861]
#  [  0.           0.           1.        ]]

# Distortion Coefficients:
# [[ 0.04385641 -0.0177826  -0.04588679  0.00081661 -0.0048952 ]]

# Open webcam
cap = cv2.VideoCapture(0)

# Robot positions to move to (you should add your positions here - there are not NEARLY enough points to make this work right now)
robot_positions = [
    # (124,0,-13,0),
    # (230,2,-20,0.5),
    # (269,-0.7,-80,-0.1),
    # (265,-0.6,-20,-0.1),
    # (245.5,18,40,4),
    # (191,14,68,4),
    # (176,-75,68,-23),
    # (174,-93,0.479,-28),
    # (220,-114,-14,-27),
    # (238,-68,-14,-16),
    # (197,-48,50,-13),
    # (196,101,30,27),
    # (259,42,-5,9),
    # (227,113,-50,26),
    # (269,125,-20,25),
    # (288,105,29.20),
    # (282,75,-36,14),
    # (305,69,44,12),
    # (218,157,15,35),
    # (175,159,-50,42),
    # (205,130,0.4,32),
    # (258,109,6.08,22.9),
    # (258,86,56,18),
    # (248,73,-43,16),
    # (288,73,15,14),
    # (293,39,-60,7),
    # (288,20,-4,4.15),
    # (277,-24,-68,-5),
    # (266,-69,5,-14),
    # (247,-117,-57,-25),
    # #new
    (266,-23,-33,20),
    (240,-15,-4,21),
    (269,-13,-14,22),
    (264,29,7,31),
    (253,6.10,26,26),
    (251,21,9,30),
    (247,66,-8,40),
    (260,77,-26,41),
    (243,92,-59,46),
    (257,90,-70,44),
    (285,65,-40,38),
    (289,66,-86,38),
    (298,-0.4,-84,25),
    (318,15,-51,28),
    (313,58,-50,35),
    (234,50,-86,37),
    (249,80,-95,43),
    (238,32,-100,33),
    (298,14,-56,28),
    (205,-23,-93,18),
    (210,121,-95,55)

]

# Data storage for transformation fitting
camera_points = []  # Detected marker positions in camera frame
robot_points = []   # Corresponding robot world-frame positions

# Move the robot and collect data
for pos in robot_positions:
    print(f"Moving to: {pos}")
    
    # ret = move_cartesian(api,pos[0],pos[1],pos[2])
    # if ret == "FAILURE":
    #     continue
    move_to_xyz(api,pos[0],pos[1],pos[2])
    
    # Get actual robot position in world frame
    robot_pose = dType.GetPose(api)
    X_r, Y_r, Z_r = robot_pose[:3]  # Extract world coordinates

    # Capture frame
    ret, frame = cap.read()
    if not ret:
        print("Camera error!")
        continue

    # Convert to grayscale and detect markers
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
        for i in range(len(ids)):
            # Store corresponding data points
            camera_points.append(tvecs[i].flatten())
            
            """
               WARNING: the robot's pose is given in millimeters. The camera's coordinates are given in meters. 
               We have done this conversion for you here, but you have to remember it was done because the coordinates that
               the transformation matrix will output will be in meters. If you do not convert back to millimeters before moving
               the robot it will assume you're sending it to a position inside of its own base.
            """
            robot_points.append([X_r/1000, Y_r/1000, Z_r/1000])
            print(f"Recorded: Camera {tvecs[i].flatten()} -> Robot {X_r, Y_r, Z_r}")

# Release camera
cap.release()
cv2.destroyAllWindows()

# Save data for transformation computation
np.save("camera_points.npy", np.array(camera_points))
np.save("robot_points.npy", np.array(robot_points))
print("Data collection complete. Run transformation fitting script next!")
