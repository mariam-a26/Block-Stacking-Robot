# Vision-Guided Autonomous Block Stacking Robot

An end-to-end robotics and computer vision pipeline that uses vision-enabled language models to provide object localization from 2D, transforming visual pixel bounding boxes into 3D world coordinates to drive autonomous pick-and-place and block-stacking routines via a robotic arm and suction end-effector.

---

## 📹 Robot Block Stacking Demo
<video src="robot_block_stacker.mp4" controls width="100%" autoplay loop muted>
  Your browser does not support the video tag.
</video>
---

## Overview & Architecture

Instead of hardcoding object locations or relying strictly on traditional color thresholding, this project leverages a **Vision-Language Model (VLM)** to interpret natural language prompts and locate target objects.

### Technical Workflow
1. **Perception & VLM Prompting:** An overhead camera captures the workspace containing ArUco markers and target blocks. The image is passed to the VLM with a structured prompt to retrieve JSON-formatted 2D bounding boxes (`[x0, y0, x1, y1]`).
2. **Kinematic & Coordinate Transformation:** 
   * Extract target centroids from 2D pixel bounding boxes.
   * Map 2D pixel coordinates to 3D camera coordinates using intrinsic camera matrices and distortion coefficients.
   * Transform 3D camera frame coordinates to the 3D real-world robot frame using calibrated extrinsic transformation matrices ($R$ and $T$).
3. **Execution & Control:** Python control scripts trigger the robotic arm and end-effector suction cup to execute precise pick, place, and stacking trajectories.
