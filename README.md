# Robotic Arm Path Planning

> Tabletop mobile manipulator: arm design, kinematics and motion-planning simulation.

## Overview

This script implements the robotic arm design, kinematics, and motion planning simulation for a tabletop mobile manipulator. At startup, the system randomly generates non-overlapping 3D desktop obstacles (such as laptops, cups, and books) on an 1800 x 800 mm table. When a user inputs a target 3D coordinate (X, Y, Z) via the command-line menu, the algorithm first automatically optimizes the two arm link lengths (L1 and L2) between 300 and 600 mm to achieve the most compact arm span while restricting the inner dead zone (<= 350 mm). It then solves the 2-DOF planar Inverse Kinematics (IK) within joint angle limits and performs 3D collision detection against the obstacles; if the target is unreachable or a collision is detected from the current position, an auto-replanning module searches for an optimal collision-free mobile base pose and generates a detour path. During arm movement, the end-effector's vertical Z-height dynamically follows the upper contour of underlying obstacles with a 60 mm safety clearance before descending to grasp the object. As its output, the program prints detailed step-by-step execution logs (selected link parameters, IK joint angles, base waypoints, and 3D trajectory coordinates) to the console and pops up two synchronized tkinter visualization windows: a Top-Down X-Y View showing the table layout, chassis detour path, arm posture, and 2D trajectory, alongside a Side X-Z View illustrating obstacle heights and the dynamic Z-axis obstacle-avoidance profile.

## Workflow

![Robotic Arm Path Planning Workflow](Robotic%20Arm%20Path%20Planning%20Workflow.png)

## Reachable Workspace & Inner Dead-Zone

![Reachable Workspace and Inner Dead-Zone of the 2R Robotic Arm](Reachable%20Workspace%20and%20Inner%20Dead-Zone%20of%20the%202R%20Robotic%20Arm.png)

## Key Parameters

- Table size: 1800 × 800 mm
- Link lengths L1, L2: optimized between 300–600 mm
- Inner dead zone: ≤ 350 mm
- Safety clearance above obstacles: 60 mm
- Visualization: two synchronized tkinter windows (Top-Down X-Y, Side X-Z)

## Running

```bash
python "arm5.0(Final Code).py"
