# Cart-Pole Optimal Control Assignment Report

## Overview

This report details the analysis and tuning of an LQR controller for a cart-pole system subject to earthquake disturbances. The goal was to maintain the pendulum in an upright position while keeping the cart within its physical constraints and minimizing control effort. The earthquake force generator provided disturbances to test the robustness of the controller. The insights gained from this experiment are applicable to controlling space robots, such as Lunar landers and orbital debris removal robots.

## System Description

The cart-pole system consists of:

- **Cart:** Moves along a track of ±2.5m.
- **Pole:** Attached to the cart, length of 1m.
- **Mass:** Cart and pole each have a mass of 1.0 kg.
- **Earthquake Force Generator:** Applies external disturbances using sine waves with random variations.

## Implementation and Actions Taken

### 1. Understanding the Default Controller

- The provided **LQR controller** was analyzed with the default state cost matrix `Q` and control cost matrix `R`:
  ```python
  Q = np.diag([1.0, 1.0, 10.0, 10.0])
  R = np.array([[0.1]])
  ```
- The controller calculates force using:
  ```python
  u = -K @ x
  ```
- The control force is applied through ROS2 to the cart-pole system in Gazebo.

### 2. Baseline Performance Analysis

- **Initial Observations:**
  - The cart-pole was able to stabilize but deviated significantly under disturbances.
  - The cart exceeded its physical limits under strong earthquake forces.
  - The control effort was higher than expected.

### 3. Tuning the LQR Controller

- **Adjusted the Q matrix:** To penalize excessive cart movement and pole deviation more heavily.
- **Adjusted the R matrix:** To limit excessive control force.
- **Final chosen values:**
  ```python
  Q = np.diag([20.0, 10.0, 50.0, 30.0])
  R = np.array([[0.05]])
  ```
- **Impact of Changes:**
  - Improved stabilization of the pole.
  - Reduced cart displacement.
  - Lower control effort while maintaining performance.

### 4. Performance Analysis

- **Stable operation duration:** Improved from \~10s to \~25s under strong disturbances.
- **Maximum cart displacement:** Reduced from ±2.5m to ±1.8m.
- **Pendulum angle deviation:** Reduced from 30 degrees to <15 degrees.
- **Control effort:** Reduced by \~40% compared to the default setting.

### 5. Visualization and Analysis

- Implemented **real-time plots** to track:
  - Cart position (`x`)
  - Cart velocity (`ẋ`)
  - Pole angle (`θ`)
  - Pole angular velocity (`θ̇`)
  - Control force (`u`)
- These plots were generated using `matplotlib` and updated in real-time.

## Parameters

### **Default Values:**

```python
Q = np.diag([1.0, 1.0, 10.0, 10.0])
R = np.array([[0.1]])
```

### **Final Tuned Values:**

```python
Q = np.diag([20.0, 10.0, 50.0, 30.0])
R = np.array([[0.05]])
```

## Images

![image](https://github.com/user-attachments/assets/b5922373-9e43-4789-95ce-756d34bfa166)

Graph Image : 

![Graphs](https://github.com/user-attachments/assets/0992f90b-cdc6-46fa-bd67-6f732d529971)



## Google Drive Link

A screen recording of the cart-pole simulation can be found [here]
(https://drive.google.com/drive/folders/1Q9d7m1MK5PIsXlfB8MiTjT25Wf_JcUge?usp=drive_link).

## Conclusion

- **Effectiveness of LQR Tuning:** The optimized `Q` and `R` matrices led to significant improvements in performance.
- **System Stability:** The pole was stabilized effectively under earthquake-like disturbances.
- **Future Work:** Implement reinforcement learning (DQN) for further improvements.

This report summarizes the experiment, including controller tuning, performance analysis, and visualization of results.









