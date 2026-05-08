# 🤖 Robot Simulation Program

## Kinematics, Trajectory Planning & 3D Visualization System

<img src="https://github.com/user-attachments/assets/cc0436d8-02a2-45dd-8cfd-4482a5806af3" width="1000"/>

---

# 📌 Project Description

This project focuses on building an interactive robotic manipulator simulator with support for:

- forward kinematics
- inverse kinematics
- trajectory planning
- real-time 3D visualization

The system allows users to simulate a **4-DOF robotic arm**, control joint angles, and generate motion trajectories using both:

- Joint Space
- Cartesian Space (XYZ coordinates)

The project integrates:

- Numerical computation (NumPy)
- 3D visualization (Matplotlib)
- Robot kinematics (DH parameters)
- Interactive GUI components

---

# 🎯 Project Goal

The main objective was to design a system that:

- simulates a robotic arm in 3D space
- implements forward and inverse kinematics
- allows trajectory creation and playback
- provides an intuitive GUI interface

---

# 🔁 System Workflow

## 1️⃣ Robot Initialization

The system initializes:

- robot geometric parameters
- link lengths
- offsets
- visualization environment

---

## 2️⃣ Joint Control (Forward Kinematics)

The robot can be controlled using sliders and text input fields.

### Slider Control

<img src="https://github.com/user-attachments/assets/f1b1c3df-12df-4ab9-92b2-1ad883108485" width="1000"/>

### Text Inputs

<img src="https://github.com/user-attachments/assets/ed468538-3c4f-405d-ab21-be5289d1fa9f" width="300"/>

The system computes:

- end-effector position
- joint coordinates
- robot geometry

using forward kinematics equations.

---

## 3️⃣ Position Computation

Robot joint positions are calculated using:

- trigonometric relations
- base-axis rotations
- homogeneous transformations

---

## 4️⃣ Inverse Kinematics

The user can input target coordinates:

- X
- Y
- Z

The system computes required joint angles automatically.

---

## 5️⃣ 3D Visualization

The robot is rendered in real time with:

- links and joints
- workspace visualization
- coordinate axes
- end-effector tracking

---

## 6️⃣ Trajectory Creation

The user can:

- save robot positions
- create motion sequences
- interpolate trajectories

<img src="https://github.com/user-attachments/assets/229ee7f8-9b72-48b7-87b9-e9f53ab685b4" width="1000"/>

---

## 7️⃣ Trajectory Interpolation & Playback

The simulator interpolates trajectory points and animates smooth robot motion.

![Trajectory Animation](https://github.com/user-attachments/assets/8778de54-9121-48e7-bcd3-8ebf83dc69a7)

---

## 8️⃣ 2D Path Drawing Interface

The application also includes a 2D drawing interface for manual trajectory generation.

<img src="https://github.com/user-attachments/assets/5f796e44-aeb4-4e7a-8e44-6eb49afab5e7" width="600"/>

---

## Full Application Window

<img src="https://github.com/user-attachments/assets/67300e45-6c64-45f8-b505-a686c4d5ecae" width="1200"/>

---

# ⚙️ Technologies and Tools

- Python 3.x
- NumPy
- Matplotlib

---

# 🧠 Core Concepts Implemented

- Forward kinematics
- Inverse kinematics
- Denavit–Hartenberg convention
- Transformation matrices
- Motion interpolation
- Cartesian ↔ Joint space conversion

---

# 🧩 Features

- Real-time robot control
- 3D visualization
- End-effector tracking
- Inverse kinematics solver
- Trajectory recording and playback
- Cartesian coordinate control
- Interactive GUI

---

# 📂 Project Structure

```text
robot-simulation-program/
│
├── main.py
├── requirements.txt
├── README.md
├── LICENSE
│
├── utils/
│   └── matrix_transformation.py
```

---

# 🚀 Future Improvements

Potential future extensions:

- ROS integration
- collision detection
- path optimization
- PyQt GUI
- Web-based visualization
- support for additional DOF

---

# 👨‍💻 Author

Robotics simulation project focused on:

- kinematics
- trajectory planning
- robot visualization

---

# 📜 License

MIT License
