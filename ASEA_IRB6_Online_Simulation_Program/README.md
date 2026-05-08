# 🤖 Robot Simulation Program

### Kinematics, Trajectory Planning & 3D Visualization System
<img width="1378" height="784" alt="image" src="https://github.com/user-attachments/assets/cc0436d8-02a2-45dd-8cfd-4482a5806af3" />

---

## 📌 Project Description

This project focuses on building an **interactive robotic manipulator simulator** with support for **forward and inverse kinematics**, **trajectory planning**, and **real-time 3D visualization**.

The system allows users to simulate a **4-DOF robotic arm**, control joint angles, and generate motion trajectories using both **joint space** and **Cartesian space (XYZ coordinates)**.

The project integrates:

* **Numerical computation (NumPy)**
* **3D visualization (Matplotlib)**
* **Robot kinematics (DH parameters)**
* **Interactive GUI components (sliders, buttons, inputs)**

It serves as a complete educational tool for understanding **robotics, kinematics, and motion planning**.

---

## 🎯 Project Goal

The primary objective was to design a system that:

* Simulates a robotic arm in 3D space
* Implements **forward and inverse kinematics**
* Allows trajectory creation and playback
* Provides an intuitive interactive interface

The project focuses on **mathematical modeling and visualization**, rather than hardware control.

---

## 🔁 System Workflow

The simulation operates through the following stages:

---

### 1️⃣ Robot Initialization

The system initializes:

* Robot geometric parameters (link lengths, offsets)
* Joint configuration (4 DOF)
* Visualization environment

---

### 2️⃣ Joint Control (Forward Kinematics)

User controls robot via:

* Sliders (Theta1–Theta4)
<img width="1359" height="743" alt="image" src="https://github.com/user-attachments/assets/f1b1c3df-12df-4ab9-92b2-1ad883108485" />

* Text input fields
<img width="239" height="133" alt="image" src="https://github.com/user-attachments/assets/ed468538-3c4f-405d-ab21-be5289d1fa9f" />

The system computes:

* End-effector position (X, Y, Z)
* Joint positions

Based on **forward kinematics equations**.

---

### 3️⃣ Position Computation

Robot joint positions are calculated using:

* Trigonometric relations
* Rotation around base axis
* Link transformations

This stage builds the full **robot geometry in 3D space**.

---

### 4️⃣ Inverse Kinematics

User can input target position:

X, Y, Z

The system:

* Computes required joint angles
* Validates solution
* Handles multiple configurations

This allows intuitive **Cartesian control of the robot**.

---

### 5️⃣ 3D Visualization

Robot is rendered in real time:

* Links and joints plotted in 3D
* End-effector highlighted
* Axes and workspace limits defined


---

### 6️⃣ Trajectory Creation

User can:

* Save joint configurations
* Add positions via XYZ coordinates
* Remove last position

These points form a **motion sequence**.
<img width="1320" height="713" alt="image" src="https://github.com/user-attachments/assets/229ee7f8-9b72-48b7-87b9-e9f53ab685b4" />

---

### 7️⃣ Trajectory Interpolation & Playback

The system:

* Interpolates positions between points
* Converts XYZ → joint angles (IK)
* Animates smooth robot motion

Trajectory is visualized as:

* Continuous path
* Animated movement

![Trajectory Animation](https://github.com/user-attachments/assets/8778de54-9121-48e7-bcd3-8ebf83dc69a7)

---

### 8️⃣ 2D Path Drawing Interface

Additional window allows:

* Drawing trajectory using mouse
* Recording path points
* Visualizing planar motion
<img width="602" height="463" alt="image" src="https://github.com/user-attachments/assets/5f796e44-aeb4-4e7a-8e44-6eb49afab5e7" />

This feature simulates **manual path planning**.

<img width="2274" height="1248" alt="image" src="https://github.com/user-attachments/assets/67300e45-6c64-45f8-b505-a686c4d5ecae" />

---

## ⚙️ Technologies and Tools

* Python 3.x
* NumPy
* Matplotlib
* Custom DH transformation module

---

## 🧠 Core Concepts Implemented

* Forward kinematics
* Inverse kinematics
* Denavit–Hartenberg (DH) convention
* Homogeneous transformation matrices
* Trajectory interpolation
* Cartesian ↔ Joint space mapping
* Interactive GUI (Matplotlib widgets)

---

## 🧩 Features

* Real-time robot control
* 3D visualization of manipulator
* End-effector position tracking
* Inverse kinematics solver
* Trajectory recording and playback
* Cartesian coordinate input
* Interactive 2D trajectory drawing
* Smooth motion interpolation

---

## 📂 Project Structure

```
robot-simulation-program/
│
├── main.py                  # Main simulation script
├── requirements.txt        # Dependencies
├── README.md               # Documentation
│
├── utils/
│   └── matrix_transformation.py   # DH matrices
│
└── assets/
    ├── demo.gif
    ├── robot_view.png
    ├── trajectory.png
    └── drawing.png
```

---

## 🔌 Simulation Responsibilities

The main application is responsible for:

* Computing robot kinematics
* Rendering 3D visualization
* Handling user interaction
* Managing trajectory logic

---

## 🖥️ Mathematical Engine Responsibilities

* Compute transformation matrices
* Solve inverse kinematics
* Validate position accuracy
* Convert between coordinate systems

---

## 🚀 Possible Future Improvements

* GUI using PyQt or Tkinter
* Collision detection
* Path optimization algorithms
* ROS integration
* Export/import trajectories
* Web-based visualization (Three.js)
* Support for more DOF

---


## 👨‍💻 Author

Robotics simulation project focused on:

* kinematics
* motion planning
* interactive visualization

---

## 📜 License

MIT License
