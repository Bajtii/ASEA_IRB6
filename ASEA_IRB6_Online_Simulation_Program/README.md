# 🤖 Robot Simulation Program

## Kinematics, Trajectory Planning & 3D Visualization System

![Main View](https://github.com/user-attachments/assets/7de31f62-e007-4c59-b80f-4881c8733fc9)

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

- ✅ 3D robot visualization
- ✅ Forward kinematics
- ✅ Inverse kinematics
- ✅ Interactive joint control
- ✅ Cartesian space control
- ✅ Trajectory interpolation
- ✅ Real-time animation
- ✅ 2D trajectory drawing
- ✅ End-effector tracking
- ✅ Interactive GUI

---

# 🖼️ Application Preview

## 3D Robot Visualization

![Robot Visualization](assets/images/robot_view.png)

---

## Joint Control Using Sliders

![Sliders](assets/images/sliders.png)

---

## Joint Angle Input Fields

![Text Inputs](assets/images/textboxes.png)

---

## Trajectory Creation

![Trajectory Creation](assets/images/trajectory.png)

---

## 2D Path Drawing Interface

![2D Drawing](assets/images/drawing.png)

---

## Trajectory Animation

![Trajectory Animation](assets/images/demo.gif)

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

The project consists of several main modules responsible for robot simulation and motion planning.

---

# 🔁 System Workflow

## 1️⃣ Robot Initialization

The system initializes:

- robot dimensions
- joint configuration
- visualization environment

Robot parameters include:

- link lengths
- offsets
- workspace dimensions

---

## 2️⃣ Forward Kinematics

The user controls robot joints using:

- sliders
- text input fields

The system computes:

- joint positions
- end-effector coordinates
- robot geometry

in real time.

---

## 3️⃣ Position Computation

The robot model is generated using:

- trigonometric equations
- rotation matrices
- coordinate transformations

This creates a full 3D representation of the manipulator.

---

## 4️⃣ Inverse Kinematics

The user can enter:

- X coordinate
- Y coordinate
- Z coordinate

The system calculates:

- required joint angles
- reachable configurations
- valid robot positions

This enables Cartesian robot control.

---

## 5️⃣ Trajectory Planning

The application allows the user to:

- save robot positions
- create motion sequences
- interpolate trajectories
- animate smooth movement

Trajectory points can be added using:

- joint angles
- Cartesian coordinates

---

## 6️⃣ Real-Time Animation

The simulator interpolates motion between positions and visualizes:

- smooth robot movement
- continuous end-effector path
- real-time trajectory playback

---

## 7️⃣ 2D Path Drawing

An additional window allows the user to:

- draw trajectories manually
- create planar paths
- test path planning concepts

using mouse interaction.

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
├── README.md
└── LICENSE
