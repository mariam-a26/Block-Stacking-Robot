import numpy as np

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

    # T1 = np.deg2rad(T1)
    # T2 = np.deg2rad(T2)
    # T3 = np.deg2rad(T3)

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

def format_vector(vec):
    """Formats a numpy array of 3 elements with fixed-point notation."""
    return f"[{vec[0]:.8f}  {vec[1]:.8f}  {vec[2]:.8f}]"

def verify_fk_from_joints(filename, L, tol=1e-3):
    with open(filename, 'r') as file:
        lines = file.readlines()

    num_match = 0
    num_miss = 0
    for idx, line in enumerate(lines):
        # Clean and split line
        parts = line.strip().strip('()').split()
        if len(parts) < 6:
            print(f"[Line {idx+1}] ⚠️ Skipped (incomplete): {line}")
            continue

        # Parse values
        values = [float(p) for p in parts]
        x_expected, y_expected, z_expected = values[0:3]
        J1, J2, J3 = values[3:6]

        # Forward kinematics
        pos_fk = forward_kinematics(L, [J1, J2, J3])

        pos_expected = np.array([x_expected, y_expected, z_expected])
        error = np.linalg.norm(pos_fk - pos_expected)

        if error < tol:
            print(f"[Line {idx+1}] ✅ Match")
            num_match += 1
        else:
            print(f"[Line {idx+1}] ❌ Mismatch:")
            print(f"    FK position:       {format_vector(pos_fk)}")
            print(f"    Expected position: {format_vector(pos_expected)}")
            print(f"    Error: {error:.6f}")
            num_miss += 1
            return
    print(f"Matched: {num_match} Missed: {num_miss}")

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
        return False

    # Constraint B: 0 < L2*cos(T2) - L3*sin(T3) < 120 note: this is because we are working with positive values for z but workspace considered it in negative
    expr_B = L2 * np.cos(T2) - L3 * np.sin(T3)
    if not (0 < expr_B < 120):
        return False

    return True

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
    T = [T1, T2, T3]
    expected = forward_kinematics(link_lengths, T)

    # Step 3: Move robot
    move_joint_angles(api, T1, T2, T3)

    # Step 4: Read pose from robot
    pose = GetPose(api)  # should return a dict or list: {'x':..., 'y':..., 'z':...}
    actual = (pose['x'], pose['y'], pose['z'])

    # Step 5: Validate
    error = np.linalg.norm(np.array(expected) - np.array(actual))
    assert error <= 1e-3, f"FK mismatch: expected={expected}, actual={actual}, error={error:.2f}mm"

def test_fk_valid_case_1():
    validate_fk_against_robot(0.0, 0.3, -0.8)

def test_fk_valid_case_2():
    validate_fk_against_robot(np.pi/6, -0.4, 0.5)

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


if __name__ == "__main__":
    # workspace and fk verification
    # test_valid_point_inside_workspace()
    # test_x_negative_outside_workspace()
    # test_z_too_low()
    # test_z_too_high()
    # test_radial_distance_too_small()
    # test_radial_distance_too_large()
    # print("All tests passed.")

    # Workspace and fk and robot verification
    # test_fk_valid_case_1()
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


    # Example usage:
    link_lengths = [0, 135, 147]
    verify_fk_from_joints('Lab2DesignData.txt', link_lengths)

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
    # print(f"angles: {T}")
    # print(f"position: {pos}")
    # print(f"desired position: {pos_desired}")
