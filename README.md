# 🛸 3D Autonomous Drone Interceptor Simulation (ROS 2 & Gazebo)

An end-to-end **Guidance, Navigation, and Control (GNC)** simulation framework for high-speed 3D aerial interception in ROS 2 and Gazebo. The system deploys an Extended Kalman Filter (EKF) to process noisy sensor measurements from a target UAV (Shahed-136) and guides an interceptor drone (Sting-II) using a 3D True Proportional Navigation Guidance (PNG) algorithm with Continuous Collision Detection (CCD).

---

## 🔑 Key Features

* **GNC Architecture**: Full decoupling between target state estimation, 3D guidance generation, and kinematic actuation.
* **Extended Kalman Filter (EKF)**: C++ implementation for real-time 3D target state estimation under sensor noise.
* **3D True Proportional Navigation (PNG)**: Advanced closed-loop guidance law commanding acceleration vectors to achieve direct physical interception.
* **Continuous Collision Detection (CCD)**: Ray-segment distance projection algorithm preventing physical tunneling effects at high intercept velocities ($>25\text{ m/s}$).
* **ROS 2 & Gazebo Integration**: Real-time simulation environment using ROS 2 communication nodes, custom QoS profiles, and Gazebo entity state services.
* **Material Folder**: Contains explained maths and sketches in spanish. Also gazebo simulation videos.
* **3D Drones Geometry**: Not included in tis repo. Recommended to download it and load following `sim_launch.py`.

---

## 🏗️ System Architecture & GNC Breakdown


```text
				   +-----------------------------------+
                   |    Gazebo Simulation Environment  |
                   +-----------------------------------+
                               |             ^
         Noisy Position        |             | SetEntityState
        /target/measurement    v             |
                   +-------------------+     |
                   | C++ EKF Estimator |     |
                   +-------------------+     |
                               |             |
       Estimated Pose          |             |
   /target/estimated_pose      v             |
                   +-------------------+     |
                   | Python Sting PNG  |-----+
                   | Guidance Controller     |
                   +-------------------+     |
                               |             |
                               +-------------+
                          /target/intercepted (Bool)
```

### 1. Navigation ($N$)
* **Target Estimation**: C++ EKF Node processes raw noisy measurements to generate optimal 3D target position estimates ($\mathbf{p}_t$).
* **Velocity Estimation**: Low-pass Exponential Moving Average (EMA) filter derives smooth target velocity vectors ($\mathbf{v}_t$) from finite differences.

### 2. Guidance ($G$)
* **3D PNG Law**: Computes commanded acceleration ($\mathbf{a}\_{\text{cmd}}$) based on Line of Sight (LOS) angular rate ($\mathbf{\Omega}\_{\text{LOS}}$) and closing velocity ($v\_{\text{closing}}$).
* **Pursuit Vector**: Blends proportional navigation acceleration ($\mathbf{a}_{PN}$) with a direct pursuit vector to guarantee trajectory convergence.

### 3. Control ($C$)
* **Kinematic Point-Mass Model**: Applies acceleration saturation limits ($90\text{ m/s}^2$) and speed limits ($26\text{ m/s}$) using discrete Euler integration.
* **Attitude Alignment**: Converts velocity vectors into spatial quaternions ($q_x, q_y, q_z, q_w$) with mesh offset correction to align the interceptor model with its velocity vector in Gazebo.

---

## 📐 Mathematical Formulation

### 1. Line of Sight (LOS) & Relative Kinematics
$$\mathbf{r}_{LOS} = \mathbf{p}_t - \mathbf{p}_s, \quad \mathbf{u}_{LOS} = \frac{\mathbf{r}_{LOS}}{\Vert{}\mathbf{r}_{LOS}\Vert{}}$$

$$\mathbf{v}_{rel} = \mathbf{v}_t - \mathbf{v}_s, \quad v_{closing} = -\mathbf{u}_{LOS} \cdot \mathbf{v}_{rel}$$

$$\mathbf{\Omega}_{LOS} = \frac{\mathbf{r}_{LOS} \times \mathbf{v}_{rel}}{\Vert{}\mathbf{r}_{LOS}\Vert{}^2}$$

### 2. 3D Proportional Navigation Guidance (PNG)
$$\mathbf{a}_{PN} = N \cdot v_{closing} \cdot \left( \mathbf{\Omega}_{LOS} \times \mathbf{u}_{LOS} \right) \quad (N = 4.5)$$

$$\mathbf{a}_{cmd} = \mathbf{a}_{PN} + 2.0 \cdot \left( \mathbf{u}_{LOS} \cdot v_{max} - \mathbf{v}_s \right)$$

### 3. Continuous Collision Detection (CCD)
To avoid tunneling during high-speed displacement in discrete time steps ($\Delta t = 0.02\text{ s}$), the algorithm evaluates the minimum distance between the target and the motion segment $\Delta \mathbf{p}_s$:

$$t^* = \text{clamp}\left( \frac{(\mathbf{p}_t - \mathbf{p}_s(k)) \cdot \Delta \mathbf{p}_s}{\Vert{}\Delta \mathbf{p}_s\Vert{}^2}, 0.0, 1.0 \right)$$

$$\mathbf{p}_{closest} = \mathbf{p}_s(k) + t^* \cdot \Delta \mathbf{p}_s$$

$$\text{Interception Confirmed } \iff \Vert{}\mathbf{p}_{closest} - \mathbf{p}_t\Vert{} \le 0.5\text{ m}$$

---

## 📦 Prerequisites & Dependencies

* **OS**: Ubuntu 22.04 LTS
* **ROS 2**: Humble Hawksbill
* **Simulator**: Gazebo Classic / Ignition Gazebo
* **Dependencies**:
  * `geometry_msgs`, `std_msgs`, `gazebo_msgs`, `rclcpp`, `rclpy`
  * `Eigen3` (C++ linear algebra)
  * `NumPy` (Python vector kinematics)

---

## 🚀 Build & Installation

1. **Clone the repository into your ROS 2 workspace**:
```bash
mkdir -p ~/uav_ws/src
cd ~/uav_ws/src
git clone https://github.com/ignagonmel/Interceptor_Drone_Project.git
```
2. **Build the workspace**:
```bash
cd ~/uav_ws
colcon build --symlink-install
source install/setup.bash
```

---


## 🎬 Execution Guide
**Launch the simulation pipeline using 4 independent terminals**:

**Terminal 1: Launch Gazebo Environment**
```bash
source ~/uav_ws/install/setup.bash
ros2 launch uav_gazebo_sim sim_launch.py
```

**Terminal 2: Run Target UAV Trajectory Generator**
```bash
source ~/uav_ws/install/setup.bash
ros2 run target_sim shahed_flight_controller
```

**Terminal 3: Run C++ EKF State Estimator**
```bash
source ~/uav_ws/install/setup.bash
ros2 run target_ekf_cpp ekf_estimator_node
```

**Terminal 4: Run Sting-II PNG Guidance Controller**
```bash
source ~/uav_ws/install/setup.bash
ros2 run uav_gazebo_sim sting_guidance_controller
```
