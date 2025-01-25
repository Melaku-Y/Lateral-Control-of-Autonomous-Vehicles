# Lateral Control of Autonomous Vehicles

## Introduction

This project focuses on designing and implementing robust **lateral control** for autonomous vehicles using both **Linear Time-Invariant (LTI)** and **Linear Parameter-Varying (LPV)** controllers. The objective is to track a reference trajectory efficiently using **state feedback control** techniques while considering vehicle dynamics. The project was developed and tested using **MATLAB & Simulink**.

### **Control Objectives:**

- **Reference Trajectory Generation**: Develop smooth and feasible paths for autonomous vehicles.
- **Kinematic and Dynamic Modeling**: Use both **kinematic models** and **2-DOF bicycle models** for control analysis.
- **LTI Control Design**: Implement an **LQR (Linear Quadratic Regulator)** for lateral control.
- **LPV Control Design**: Extend to **polytopic LPV control**, improving robustness against varying vehicle speeds.
- **Simulation & Evaluation**: Compare the performance of LTI vs LPV controllers in different scenarios.

---

## **Tools and Technologies**

- **Framework**: MATLAB & Simulink
- **Programming Language**: MATLAB
- **Development Environment**: MATLAB R2023a
- **Version Control**: GitHub

---

## **Vehicle Modeling**

### **1. Kinematic Model**

The kinematic model represents the vehicle’s motion in a **global coordinate system (X, Y)**:

Where:

- &#x20;are the global positions
- &#x20;is the yaw angle
- &#x20;is the longitudinal velocity

### **2. Dynamic (Bicycle) Model**

The **2-DOF linear bicycle model** is used for control synthesis, capturing lateral dynamics:

Where:

- &#x20;\= lateral acceleration
- &#x20;\= yaw moment of inertia
- &#x20;\= distances from the CoG to axles
- &#x20;\= lateral tire forces

By linearizing this model, a **state-space representation** is obtained.

---

## **Control Strategies**

### **1. LTI Control (LQR Design)**

The **LQR controller** is designed by solving:

where:

- **Q** penalizes state deviations (yaw rate, lateral velocity)
- **R** penalizes control effort (steering angle)

A discrete-time LQR is implemented using MATLAB’s `dlqr` function.

### **2. LPV Control (Polytopic Model)**

To handle varying vehicle speeds, the **LPV control framework** is used, interpolating between different LTI controllers:

where  represents scheduling parameters like velocity.

A **polytopic controller** is synthesized by solving Linear Matrix Inequalities (LMIs).

---

## **Simulation Scenarios and Results**

### **Scenario 1: Reference Tracking with LQR (LTI)**

- **Conditions**: Constant vehicle speed.
- **Results**:
  - The LQR effectively tracks the reference trajectory.
  - Some deviations in sharp turns.

### **Scenario 2: Reference Tracking with LPV Control**

- **Conditions**: Varying vehicle speeds (5–20 m/s).
- **Results**:
  - LPV controller adapts better to speed variations.
  - Improved robustness and trajectory tracking compared to LQR.

### **Scenario 3: Noisy Environment**

- **Conditions**: Process noise added to simulate real-world disturbances.
- **Results**:
  - LPV controller maintains better stability compared to LQR.
  - LQR struggles with noise-induced errors.

---

## **Usage Instructions**

### **Step 1: Run Simulink Models**


1. Open **MATLAB** and navigate to the project folder.
2. Open `simulator_lqr_lpv.slx` in Simulink.
3. Modify simulation parameters if needed.
4. Click **Run** to start the simulation.

### **Step 2: Run MATLAB Scripts**

- To test the system, run:
  ```matlab
  run Project.m
  ```

---

## **Results and Visualizations**

Below are some visualizations of the **path tracking** results:

### **Scenario: LQR Control**



### **Scenario: LPV Control**



---

## **Conclusion**

- The **LQR controller** performs well in constant-speed scenarios but struggles with varying speeds and noise.
- The **LPV-based controller** outperforms LQR in **robustness, adaptability, and accuracy**.
- Using **polytopic interpolation**, the LPV controller provides smooth transitions for different vehicle speeds.
- This work highlights the **importance of adaptive control strategies** in autonomous vehicle applications.

---

## **Thank You!**

