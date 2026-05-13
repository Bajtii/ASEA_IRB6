# 4-Axis UART DC Motor Controller

## Overview

This project is a low-level embedded motion control firmware designed for a 4-axis robotic system powered by stepper motors.  
The software runs on an Arduino-compatible microcontroller and acts as a subordinate motion controller for a higher-level robotic application or external master controller.

The firmware continuously listens for commands transmitted over UART (`Serial`) and dynamically generates synchronized `STEP/DIR` control signals for external stepper motor drivers.

The controller is designed for robotics, automation systems, CNC applications, and custom motion-control platforms where reliable real-time motor control is required.

---

# Features

- 4-axis stepper motor control
- UART-based communication interface
- Real-time trajectory execution
- Dynamic trajectory buffering in RAM
- Coordinated multi-axis motion
- Homing procedure using limit switches
- Manual axis control
- Angle-to-step conversion
- Position tracking system
- Synchronized pulse generation
- Runtime trajectory streaming

---

# UART Motion Command Interface

The controller continuously listens for UART commands at:

```cpp
115200 baud
```

Trajectory points can be streamed directly from:

- PC applications
- Raspberry Pi / SBC systems
- External robotic controllers
- Motion planning software

Example trajectory format:

```cpp
{10.0, -20.0, 30.0, 0.0},
{15.0, -25.0, 35.0, 0.0},
```

After receiving trajectory data, the firmware:

1. Parses incoming angle values
2. Converts angles into motor steps
3. Calculates step deltas
4. Synchronizes all axes
5. Generates real-time STEP pulses
6. Executes coordinated robotic motion

---

# Supported Functions

## Homing Procedure

Each axis supports automatic homing using dedicated limit switches.

### Homing sequence

1. Move axis toward endstop
2. Detect limit switch activation
3. Reset internal position counters
4. Move robot into calibrated startup position

This ensures repeatable positioning and proper robot initialization.

---

## Coordinated Multi-Axis Motion

The firmware supports simultaneous motion of all 4 stepper motors while maintaining synchronized timing between axes.

This enables:

- Smooth robotic trajectories
- Coordinated joint movement
- Accurate positioning
- Repeatable motion sequences
- External trajectory playback

---

## Manual Axis Control

Two manual control modes are implemented.

### Angle Mode

Move selected motor by a specified angle in degrees.

### Step Mode

Move selected motor using a direct step count.

This functionality is useful for:

- calibration
- testing
- debugging
- manual positioning

---

# Hardware Configuration

## Stepper Motor Pins

| Axis | DIR Pin | STEP Pin |
|------|------|------|
| X | 5 | 18 |
| Y | 4 | 2 |
| Z | 21 | 22 |
| A | 25 | 26 |

---

## Limit Switch Pins

| Axis | Home Pin |
|------|------|
| X | 14 |
| Y | 12 |
| Z | 27 |
| A | 33 |

---

## Enable Pin

```cpp
#define ENABLE_PIN 32
```

---

# UART Serial Menu

After startup, the firmware exposes a simple UART command menu:

```text
1: Home Robot
2: Select Axis and Angle
3: Select Axis and Steps
4: Execute Trajectory
5: Send Trajectory Angles
```

---

# Motion Processing Pipeline

The motion system processes data in the following order:

```text
UART Input
    ↓
Trajectory Parsing
    ↓
Angle Validation
    ↓
Angle → Step Conversion
    ↓
Delta Step Calculation
    ↓
STEP/DIR Signal Generation
    ↓
Motor Driver Execution
```

---

# Angle-to-Step Conversion

The firmware converts joint angles into motor steps using calibrated conversion coefficients.

Example:

```cpp
target[0] = lroundf(-thetas[0] * 452.548f);
```

This allows the robotic system to operate directly in angular space while maintaining low-level control over external motor drivers.

---

# Dynamic Trajectory Buffer

Trajectory points are dynamically allocated in RAM using:

```cpp
realloc()
```

Advantages:

- No fixed trajectory size
- Scalable motion sequences
- Efficient memory management
- Runtime trajectory streaming
- Flexible motion execution

---

# Safety Features

- Endstop-based homing
- Driver enable/disable control
- Controlled pulse timing
- Position tracking
- Invalid trajectory rejection
- UART packet validation
- Motion synchronization

---

# Possible Applications

- Robotic arm controller
- CNC subsystem
- Pick-and-place robot
- Industrial automation platform
- Motion-control slave module
- Educational robotics projects
- Embedded automation systems
- Research and prototyping platforms

---

# Dependencies

- Arduino Framework
- Arduino Serial Library

---

# Future Improvements

Planned or possible future extensions:

- Acceleration/deceleration ramps
- S-curve trajectory profiles
- FreeRTOS integration
- Binary UART communication protocol
- ROS2 communication bridge
- Closed-loop encoder feedback
- DMA-based pulse generation
- G-code interpreter
- ESP32 WiFi telemetry
- CAN bus communication
- Real-time trajectory interpolation

---
