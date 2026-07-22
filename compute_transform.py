import numpy as np

def find_rigid_transform(X_c, X_r):
    """ Computes optimal rotation (R) and translation (T) from camera frame to world frame. """
    centroid_c = np.mean(X_c, axis=0)
    centroid_r = np.mean(X_r, axis=0)

    X_c_centered = X_c - centroid_c
    X_r_centered = X_r - centroid_r

    H = X_c_centered.T @ X_r_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    T = centroid_r - R @ centroid_c
    return R, T

# Load collected data
X_c = np.load("camera_points.npy")
X_r = np.load("robot_points.npy")

# Compute transformation
R, T = find_rigid_transform(X_c, X_r)

print("Computed Rotation Matrix (R):")
print(R)

print("\nComputed Translation Vector (T):")
print(T)

# Save transformation
np.save("R.npy", R)
np.save("T.npy", T)
print("Transformation saved!")
