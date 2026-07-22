import DobotDllType as dType
import time
import threading
import math 
import numpy as np

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
        dType.dSleep(15)

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


def forward_kinematics(L, T):
    """
    Compute forward kinematics for 3-DOF RRR robot.

    Parameters:
    - L: [L1, L2, L3] link lengths
    - T: [T1, T2, T3] joint angles in radians

    Returns:
    - pos: [x, y, z] end-effector position
    - orient: 3x3 orientation matrix
    """
    T1, T2, T3 = T
    L1, L2, L3 = L

    T1 = np.deg2rad(T1)
    T2 = np.deg2rad(T2)
    T3 = np.deg2rad(T3)

    c1 = np.cos(T1)
    s1 = np.sin(T1)
    c2 = np.cos(T2)
    c3 = np.cos(T3)
    s2 = np.sin(T2)
    s3 = np.sin(T3)

    x = c1*(L3*c3 + L2*s2 + 59.7)
    y = s1*(L3*c3 + L2*s2 + 59.7)
    z = L2*c2 - L3*s3
    pos = [x,y,z]
    return pos

def get_jacobian(L, T):
    T1, T2, T3 = T
    # T1 = np.deg2rad(T1)
    # T2 = np.deg2rad(T2)
    # T3 = np.deg2rad(T3)
    L1, L2, L3 = L
    off = 59.7

    J = np.array([
        [-np.sin(T1)*(L2*np.sin(T2) + L3*np.cos(T3) + off),
         L2*np.cos(T2)*np.cos(T1),
         -L3*np.sin(T3)*np.cos(T1)],
        
        [np.cos(T1)*(L2*np.sin(T2) + L3*np.cos(T3) + off),
         L2*np.cos(T2)*np.sin(T1),
         -L3*np.sin(T3)*np.sin(T1)],
        
        [0,
         -L2*np.sin(T2),
         -L3*np.cos(T3)]
    ])

    return J

def ik(L, T_init, pos_desired, max_iter, tol, alpha, solver=2, lam=0.01):
    T = np.array(T_init).reshape(-1)  # radians

    for i in range(max_iter):
        pos_current = forward_kinematics(L, T)
        pos_err = np.array(pos_desired).reshape(3,) - pos_current

        if np.linalg.norm(pos_err) < tol:
            break

        J_pos = get_jacobian(L, T)

        if solver == 1:
            dT = alpha * np.linalg.pinv(J_pos) @ pos_err
        elif solver == 2:
            JJt = J_pos @ J_pos.T
            dT = alpha * J_pos.T @ np.linalg.solve(JJt + (lam**2) * np.eye(J_pos.shape[0]), pos_err)
        else:
            dT = alpha * J_pos.T @ pos_err

        T = T + dT  # radians

    return T, pos_current


def is_within_joint_workspace(T1, T2, T3, L2=135, L3=147):
    """
    Check if (T1, T2, T3) lies within the defined joint-space workspace.

    Parameters:
    - T1, T2, T3: Joint angles in radians
    - L2, L3: Link lengths (must be positive)

    Returns:
    - True if joint angles satisfy all workspace constraints, False otherwise
    """
    # Constraint C: cos(T1) >= 0 ⇒ T1 ∈ [−π/2, π/2]
    if not (-np.pi/2 <= T1 <= np.pi/2):
        return False

    # Constraint A: 120 ≤ L3*cos(T3) + L2*sin(T2) ≤ 280
    expr_A = L3 * np.cos(T3) + L2 * np.sin(T2)
    if not (120 <= expr_A <= 280):
        print(f"expr_A: {expr_A}")
        return False

    # Constraint B: 0 < L2*cos(T2) - L3*sin(T3) < 120 note: this is because we are working with positive values for z but workspace considered it in negative
    expr_B = L2 * np.cos(T2) - L3 * np.sin(T3)
    if not (-120 < expr_B < 0):
        print(f"expr_B: {expr_B}")
        return False

    return True   
    
"""
    Move the robot to it's home position. Note: this will use basic PTP motion, rather than
    SetHOMECmd, since SetHOMECmd will re-run the sensor initialization stuff that we don't
    need during normal operation
"""
def move_to_home(api):
    move_to_xyz(api,home_pos[0],home_pos[1],home_pos[2])
    
def move_cartesian(api, x, y, z):
    if not -120 < z < 0:
        return "FAILURE 1"
    if not 120 <= math.sqrt(x**2 + y**2) <= 280:
        print(math.sqrt(x**2 + y**2))
        return "FAILURE 2"
    if x < 0:
        return "FAILURE 3"
    
    move_to_xyz(api, x, y, z)
    pos_x, pos_y, pos_z, _, _, _, _, _ = dType.GetPose(api)
    return (str(pos_x), str(pos_y), str(pos_z))
    
def get_joint(api, x, y, z):
   
    move_to_xyz(api, x, y, z)
    pos_x, pos_y, pos_z, _, j1, j2, j3, _ = dType.GetPose(api)
    return (str(pos_x), str(pos_y), str(pos_z), str(j1), str(j2), str(j3))
    
#Before running and commands, always run this
initialize_robot(api)

"""
    Here is a sample script that moves the robot to a position, then moves back to home, then to another position, five times
    
    It also prints the pose of the robot. Then, we move the robot by joint angle just to show how it's done.
"""
"""for i in range(0, 5):
    if i % 2 == 0:
        offset = 50
    else:
        offset = -50
    
    #move to the postion
    move_to_xyz(api,200 + offset, offset, offset)
    
    Get actual robot position in world frame. This list contains [x,y,z,r,J1,J2,J3,J4]. 
    x,y,z are in MILLIMETERS, r depends on the end-effector and can usually be ignored in our labs, J1-J4 are in degrees. J4 is not really used here, but it is
    the rotation angle of the end effector. This matters if you have the gripper or suction cup installed, and it will physically
    rotate it about the z axis
    
    robot_pose = dType.GetPose(api)
    print(robot_pose)
    #Back to home
    move_to_home(api)

print("PTP Motions done. Moving in Joint Space now")
#move by joint angles, in degrees    
move_joint_angles(api,0,45,45)"""

#Back to home
print("home")
move_to_home(api)


# ------------ Workspace Tests ------------

def test_valid_point_inside_workspace():
    assert is_within_joint_workspace(np.pi/4, np.pi/4, np.pi/4) == True

def test_x_negative_outside_workspace():
    # T1 = π (~180°), which flips x to negative
    assert is_within_joint_workspace(np.pi, -0.79, 0.52) == False

def test_z_too_low():
    # T2 = T3 = -π/2 → max downward position
    assert is_within_joint_workspace(0.0, -np.pi/2, -np.pi/2) == False

def test_z_too_high():
    # T2 = π/2 lifts arm straight up
    assert is_within_joint_workspace(0.0, np.pi/2, 0.0) == False

def test_radial_distance_too_small():
    # Fully folded configuration; radial distance near zero
    assert is_within_joint_workspace(0.0, np.pi, 0.0) == False

def test_radial_distance_too_large():
    # Fully extended arm, long links
    assert is_within_joint_workspace(0.0, 0.0, 0.0) == False

# ------------------------- Test with actual robot -------------------------
def validate_fk_against_robot(T1, T2, T3):
    # Step 1: Check joint constraints
    assert is_within_joint_workspace(T1, T2, T3), f"Joint angles out of workspace: T1={T1}, T2={T2}, T3={T3}"

    # Step 2: Compute expected FK
    link_lengths = [0, 135, 147]

    T1 = np.rad2deg(T1)
    T2 = np.rad2deg(T2)
    T3 = np.rad2deg(T3)
    T = [T1, T2, T3]
    
    # print(f"before T1={T1}, T2={T2}, T3={T3}")
    expected = forward_kinematics(link_lengths, T)

    # print(f"after T1={T1}, T2={T2}, T3={T3}")

    # Step 3: Move robot
    move_joint_angles(api, T1, T2, T3)

    # Step 4: Read pose from robot
    pose = dType.GetPose(api)  # should return a dict or list: {'x':..., 'y':..., 'z':...}
    actual = (pose[0], pose[1], pose[2])

    # Step 5: Validate
    error = np.linalg.norm(np.array(expected) - np.array(actual))
    assert error <= 1e-3, f"FK mismatch: expected={expected}, actual={actual}, error={error:.2f}mm"

def test_fk_valid_case_1():
    validate_fk_against_robot(0.0, np.pi/2, np.pi/4)

def test_fk_valid_case_2():
    validate_fk_against_robot(0.0, np.pi/3, np.pi/3)

def test_fk_edge_of_workspace():
    validate_fk_against_robot(np.pi/2, 0.7, -1.2)

def test_fk_center_pose():
    validate_fk_against_robot(0.0, 0.0, 0.0)

def test_fk_valid_positive_sweep():
    # Moderate angles in positive range
    validate_fk_against_robot(np.pi/6, np.pi/6, -np.pi/4)

def test_fk_valid_negative_sweep():
    # Moderate angles in negative range
    validate_fk_against_robot(-np.pi/6, -np.pi/4, np.pi/6)

def test_fk_valid_near_max_T2():
    # T2 near upper bound
    validate_fk_against_robot(0.0, np.pi/2 - 0.01, -np.pi/3)

def test_fk_valid_near_min_T2():
    # T2 near lower bound
    validate_fk_against_robot(0.0, -np.pi/2 + 0.01, np.pi/3)

def test_fk_valid_edge_T1_positive():
    # T1 near upper bound
    validate_fk_against_robot(np.pi/2 - 0.01, 0.2, -0.5)

def test_fk_valid_edge_T1_negative():
    # T1 near lower bound
    validate_fk_against_robot(-np.pi/2 + 0.01, -0.2, 0.5)

def test_fk_valid_maximum_reach():
    # Joint angles that push reach to max radial distance
    validate_fk_against_robot(0.0, 0.2, -0.2)

def test_fk_valid_high_elbow_configuration():
    # Shoulder and elbow up
    validate_fk_against_robot(0.3, 0.8, -0.7)

def test_fk_valid_low_elbow_configuration():
    # Shoulder and elbow down
    validate_fk_against_robot(-0.3, -0.6, 0.5)

def test_fk_out_of_workspace_should_fail():
    try:
        validate_fk_against_robot(np.pi, 1.2, 0.0)
    except AssertionError as e:
        print(f"Correctly failed: {e}")

def test_fk_invalid_T1_exceeds_limit():
    try:
        validate_fk_against_robot(np.pi, 0.1, -0.2)
    except AssertionError as e:
        print(f"Correctly failed T1 limit: {e}")

def test_fk_invalid_expr_A_violation():
    try:
        validate_fk_against_robot(0.0, -1.4, 2.0)
    except AssertionError as e:
        print(f"Correctly failed Constraint A: {e}")

def test_fk_invalid_expr_B_violation():
    try:
        validate_fk_against_robot(0.0, 0.0, 0.0)  # Known to fail constraint B
    except AssertionError as e:
        print(f"Correctly failed Constraint B: {e}")



# workspace and fk verification
# test_valid_point_inside_workspace()
# test_x_negative_outside_workspace()
# test_z_too_low()
# test_z_too_high()
# test_radial_distance_too_small()
# test_radial_distance_too_large()
# print("All tests passed.")

# Workspace and fk and robot verification
# print('1 start')
# test_fk_valid_case_1()
# print('1 end')
# test_fk_valid_case_2()
# test_fk_edge_of_workspace()
# test_fk_center_pose()
# test_fk_valid_positive_sweep()
# test_fk_valid_negative_sweep()
# test_fk_valid_near_max_T2()
# test_fk_valid_near_min_T2()
# test_fk_valid_edge_T1_positive()
# test_fk_valid_edge_T1_negative()
# test_fk_valid_maximum_reach()
# test_fk_valid_high_elbow_configuration()
# test_fk_valid_low_elbow_configuration()
# test_fk_out_of_workspace_should_fail()
# test_fk_invalid_T1_exceeds_limit()
# test_fk_invalid_expr_A_violation()
# test_fk_invalid_expr_B_violation()
# print("All tests passed.")

link_lengths = [0, 135, 147]
solver = 2
lamb = 0.5     
alpha = 0.01 
max_iter = 10000
tol = 1e-4
T_init = [np.pi/4, np.pi/4, np.pi/2]

pos_desired = [120, 120, -110]

# print(f"starting position {forward_kinematics(link_lengths, T_init)}")
# print("-----------")
# pos_desired = [110, 100, 0]
# T, pos = ik(link_lengths, T_init, pos_desired, max_iter, tol, alpha, solver, lamb)
# move_joint_angles(api, T[0], T[1], T[2])
# pos_expected = forward_kinematics(link_lengths, T)
# pos_x, pos_y, pos_z, _, _, _, _, _ = dType.GetPose(api)
# error = np.linalg.norm(np.array(pos_expected) - np.array([pos_x, pos_y, pos_z]))
# if error > 1e-3:
#     print(f"error: {error}")
# else:
#     print("success")

#LAB 1

'''print("Testing our new function!!!!!!!!!!!!!!!!!!!")

print("test 1 (should fail):")
print(move_cartesian(api, 0, 0, 0))

print("test 2 (should fail):")
print(move_cartesian(api, 0, 0, -120))

print("test 3 (should fail):")
print(move_cartesian(api, 1000, 1000, -10))

print("test 4 (should fail):")
print(move_cartesian(api, 1, 1, -10))

print("test 5 (should fail):")
print(move_cartesian(api, -100, 100, -10))

print("test 6 (should succeed):")
print(move_cartesian(api, 100, 100, -10))'''

# with open("lab1_robot_data.txt", "w") as file:
#     for i in range(0, 10):
#             print("round " +str(i+1))
#             file.write("Round " +str(i+1)+ ":\n")
#             radius = 125
#             z = -10
#             x = 0
            
#             while x <= radius:
#                 y = math.sqrt(radius**2 - x**2)
#                 theo_line = "Theoretical: (" + str(x) + ", " + str(y) + ", " + str(z) + ")"
#                 real_x, real_y, real_z = move_cartesian(api, x, y, z)
#                 real_line = "Real: (" + real_x + ", " + real_y + ", " + real_z + ")"
#                 lines = [real_line, theo_line]
#                 print("(" + real_x + ", " + real_y + ", " + real_z + ")")
#                 file.writelines(line + "\n" for line in lines)
#                 x += 5

#             x = radius
#             while x >= 0:
#                 y = math.sqrt(radius**2 - x**2)
#                 theo_line = "Theoretical: (" + str(x) + ", " + str(-y) + ", " + str(z) + ")"
#                 real_x, real_y, real_z = move_cartesian(api, x, -y, z)
#                 real_line = "Real: (" + real_x + ", " + real_y + ", " + real_z + ")"
#                 lines = [real_line, theo_line]
#                 print("(" + real_x + ", " + real_y + ", " + real_z + ")")
#                 file.writelines(line + "\n" for line in lines)
#                 x -= 5
            
#             move_to_home(api)


    

#All done!