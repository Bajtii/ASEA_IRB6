# 🤖 Robot Simulation Program

## Kinematics, Trajectory Planning & 3D Visualization System

<img src="https://github.com/user-attachments/assets/7de31f62-e007-4c59-b80f-4881c8733fc9" width="1000"/>

---

# 📌 Project Description

This project is an interactive robotic manipulator simulation system focused on:

- forward kinematics
- inverse kinematics
- trajectory planning
- real-time 3D visualization

The application simulates a **4-DOF robotic arm** and allows the user to:

- control robot joints manually
- compute end-effector position
- generate trajectories
- visualize robot movement in real time
- control the robot using Cartesian coordinates (XYZ)

The project was created as a robotics and motion-planning educational platform using Python and mathematical modeling.

---

# 🎯 Main Features

- 3D robot visualization
- Forward kinematics
- Inverse kinematics
- Interactive joint control
- Cartesian space control
- Trajectory interpolation
- Real-time animation
- 2D trajectory drawing
- End-effector tracking
- Interactive GUI

---

# 🖼️ Application Preview

## 3D Robot Visualization

<img src="https://github.com/user-attachments/assets/2912af8b-e3f4-4906-8170-729d10f47525" width="700"/>

---

## Joint Control Using Sliders

<img src="https://github.com/user-attachments/assets/59cd51c7-c097-4dae-a7f2-16149c11ea0e" width="1000"/>

---

## Joint Angle Input Fields

<img src="https://github.com/user-attachments/assets/f5da7755-e521-47b4-b9ed-d15d26710e91" width="250"/>

---

## Trajectory Creation

<img src="https://github.com/user-attachments/assets/df07523f-7099-4217-8e9e-1406193a8234" width="1000"/>

<img src="https://github.com/user-attachments/assets/6e0b5228-3fd8-4bde-800d-79dbbae36ebf" width="700"/>

---

## 2D Path Drawing Interface

<img src="https://github.com/user-attachments/assets/6ed26cdd-64d8-4431-9a44-57dec3ef837a" width="700"/>

---

## Trajectory Animation

<img src="https://github.com/user-attachments/assets/9aedb0ae-244a-4c02-b62e-90fb9a5987c2" width="700"/>

---

# ⚙️ Technologies Used

## Programming Language

- Python 3.x

## Libraries

- NumPy
- Matplotlib

## Robotics Concepts

- Denavit–Hartenberg convention
- Homogeneous transformation matrices
- Forward kinematics
- Inverse kinematics
- Trajectory interpolation

---

# 🧠 System Architecture

The project consists of modules responsible for robot simulation and motion planning.

---

# 🔁 System Workflow

## 1️⃣ Robot Initialization

- robot dimensions
- joint configuration
- visualization environment

---

## 2️⃣ Forward Kinematics

User controls robot via:

- sliders
- text inputs

System computes:

- joint positions
- end-effector position

---

## 3️⃣ Position Computation

Uses:

- trigonometry
- rotation matrices
- transformations

---

## 4️⃣ Inverse Kinematics

Input:

- X, Y, Z

Output:

- joint angles
- valid configurations

---

## 5️⃣ Trajectory Planning

- save positions
- interpolate motion
- animate robot path

---

## 6️⃣ Real-Time Animation

- smooth motion
- trajectory playback
- end-effector path

---

## 7️⃣ 2D Path Drawing

- manual drawing
- mouse input
- planar trajectory testing

---

# 📂 Project Structure

```text
robot-simulation-program/
│
├── assets/
│   └── images/
│       ├── main_view.png
│       ├── robot_view.png
│       ├── sliders.png
│       ├── textboxes.png
│       ├── trajectory.png
│       ├── drawing.png
│       └── demo.gif
│
├── utils/
│   └── matrix_transformation.py
│
├── main.py
├── requirements.txt
└── LICENSE
```

---
