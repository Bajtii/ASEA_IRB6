# 🤖 Robot Drawing Control System

<img width="1915" height="1122" alt="image" src="https://github.com/user-attachments/assets/08082fd1-39de-4f94-abed-b5fedbe66abf" />

A desktop application created in **C# WPF** for controlling a robotic drawing arm using **computer vision**, **image processing**, and **robotic kinematics**.

The system allows a user to load an image, automatically generate contours, convert them into robot trajectories, calculate inverse kinematics, and send motion commands to an Arduino-controlled robotic arm.

---

# 📌 Project Goals

The main objective of this project was to create a complete robotic drawing pipeline capable of:

- Loading images
- Detecting contours
- Generating robot trajectories
- Calculating inverse kinematics
- Controlling a robotic arm
- Simulating robot motion in real time
- Communicating with Arduino through serial communication

The project combines several engineering domains:

- Computer Vision
- Image Processing
- Robotics
- Kinematics
- Real-Time Systems
- Embedded Communication

---

# 🏗️ System Architecture

The application is divided into several independent modules:

```text
+---------------------------------------------------+
|                  WPF GUI Layer                    |
+---------------------------------------------------+
                ↓
+---------------------------------------------------+
|             Image Processing Module               |
|        (OpenCV / EmguCV Algorithms)               |
+---------------------------------------------------+
                ↓
+---------------------------------------------------+
|           Trajectory Generation System            |
+---------------------------------------------------+
                ↓
+---------------------------------------------------+
|        Inverse / Forward Kinematics Module        |
+---------------------------------------------------+
                ↓
+---------------------------------------------------+
|          Serial Communication Interface           |
|                 (Arduino Control)                 |
+---------------------------------------------------+
                ↓
+---------------------------------------------------+
|                 Robotic Arm System                |
+---------------------------------------------------+
```

---

# 🖥️ GUI Layer (WPF)

The graphical user interface was built using:

- **C#**
- **WPF (Windows Presentation Foundation)**

The GUI allows the user to:

- Load images
- Preview generated contours
- Configure COM port and baud rate
- Rotate images
- Modify contour density
- Configure drawing height (Z axis)
- Adjust contour filtering threshold
- Simulate robot movement
- Send trajectories to Arduino

---

# 🧠 Image Processing Module

<img width="1911" height="1116" alt="image" src="https://github.com/user-attachments/assets/153b8b1c-09c5-4445-910c-1d0b3830ff92" />


The application uses:

- **OpenCV**
- **EmguCV (.NET wrapper for OpenCV)**

for image analysis and contour extraction.

---

# 🖼️ Image Processing Pipeline

```text
Input Image
    ↓
Grayscale Conversion
    ↓
Binary Thresholding
    ↓
Image Inversion
    ↓
Contour Detection
    ↓
Contour Hierarchy Analysis
    ↓
Contour Filtering
    ↓
Point Density Optimization
    ↓
Coordinate Scaling
    ↓
Trajectory Generation
```

---

# 🔍 Implemented Computer Vision Algorithms

## 1. Grayscale Conversion

The input image is converted into grayscale to simplify image processing operations.

---

## 2. Binary Thresholding

The application converts the image into a binary representation:

```text
Black pixels  → contour
White pixels → background
```

This operation improves contour detection accuracy.

---

## 3. Image Inversion

Binary inversion is used to make contours easier to extract using OpenCV contour detection functions.

---

## 4. Contour Detection

Contours are extracted using:

```cpp
FindContours()
ContourArea()
```

The algorithm detects all object boundaries from the image.

---

## 5. Contour Hierarchy Analysis

The system analyzes parent-child contour relationships to distinguish between:

- outer contours,
- internal contours,
- duplicated edges.

---

## 6. Double Edge Filtering

A custom filtering algorithm removes duplicated contours caused by thick image edges.

The algorithm compares:

```text
child contour area / parent contour area
```

If the ratio exceeds a configurable threshold, the contour is ignored.

This significantly improves trajectory quality and reduces redundant robot motion.

---

## 7. Point Density Optimization

The application dynamically reduces the number of contour points.

Benefits:

- lower memory usage,
- smaller trajectory size,
- faster robot execution,
- smoother serial communication,
- improved performance.

---

# 🤖 Robotics and Kinematics

## Inverse Kinematics

The system computes robot joint angles for every generated trajectory point.

Computed angles:

```text
theta1
theta2
theta3
theta4
```

The implementation includes:

- trigonometric calculations,
- geometric transformations,
- workspace validation,
- angle clamping,
- singularity handling,
- forward kinematics verification.

---

## Forward Kinematics

Forward kinematics is implemented using:

- Denavit–Hartenberg transformations,
- transformation matrices,
- coordinate system conversions.

The system validates whether the generated joint angles reproduce the desired Cartesian position.

---

# 📐 DH Transformation Matrices

Implemented transformations:

- Rotation around Z axis
- Translation along Z axis
- Translation along X axis
- Rotation around X axis

The project reproduces Arduino-side kinematic calculations with full compatibility.

---

# ✏️ Robot Trajectory Generation

The application converts image contours into robot movement trajectories.

Each contour includes:

- pen-up movement,
- drawing movement,
- contour closing movement.

Trajectory points are represented as:

```text
(X, Y, Z)
```

coordinates and then transformed into robot joint angles.

---

# 🎞️ Real-Time Simulation

<img width="1912" height="1123" alt="image" src="https://github.com/user-attachments/assets/b9323f82-c368-4dfb-b10a-c3a71ed93c2d" />


The system includes a real-time drawing simulator.

Simulation features:

- animated trajectory playback,
- pen-up / pen-down visualization,
- trajectory preview,
- bitmap rendering,
- drawing visualization.

The simulator is implemented using:

```csharp
DispatcherTimer
```

with configurable FPS.

---

# 🔌 Serial Communication

Communication with the robotic arm is implemented using:

```csharp
System.IO.Ports.SerialPort
```

Supported features:

- configurable COM ports,
- configurable baud rates,
- Arduino synchronization,
- command-based communication,
- chunked trajectory transmission.

---

# 📡 Communication Protocol

| Command | Description |
|---|---|
| `1` | Move robot to home position |
| `4` | Execute trajectory |
| `5` | Enter trajectory receiving mode |
| `back` | Return to main menu |

---

# ⚙️ Technologies Used

## Languages

- C#
- XAML

## Frameworks

- .NET
- WPF

## Libraries

- EmguCV
- OpenCV
- System.IO.Ports

---

# 🚀 Main Features

## Image Processing

- Image loading
- Image rotation
- Binary thresholding
- Contour extraction
- Contour filtering
- Density optimization

## Robot Control

- Inverse kinematics
- Forward kinematics
- Trajectory generation
- Angle computation
- Workspace validation

## Communication

- Serial communication
- Arduino integration
- Chunked data transfer

## Visualization

- Real-time simulation
- Drawing preview
- Trajectory animation

---

# 🎯 Engineering Objectives

This project demonstrates:

- integration of robotics and computer vision,
- practical image processing techniques,
- robotic arm control systems,
- trajectory planning,
- embedded communication,
- real-time simulation systems.

---

# 🔮 Possible Future Improvements

Potential future extensions include:

- SVG import support,
- G-code support,
- AI-based contour optimization,
- automatic calibration,
- trajectory smoothing,
- ROS integration,
- camera-based feedback system,
- real-time closed-loop control,
- multi-axis robot support.

---

# 📚 Educational Value

This project can serve as:

- a robotics learning platform,
- a computer vision demonstration,
- a robotic drawing prototype,
- an engineering thesis project,
- a research foundation.

---

# 📸 Real Robot
<img width="983" height="455" alt="image" src="https://github.com/user-attachments/assets/72e8e499-34a3-47f1-8617-c27c8a0ad187" />

<img width="983" height="439" alt="image" src="https://github.com/user-attachments/assets/2ba4717b-8a2d-4241-bdd3-ada2d02de257" />


