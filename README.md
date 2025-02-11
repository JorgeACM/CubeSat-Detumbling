# 🛰️ CubeSat Detumbling – Modeling and Control  

This project focuses on **modeling and controlling the attitude dynamics** of a CubeSat for detumbling, stabilizing its orientation by reducing rotational velocity. It implements **two control strategies** using **magnetorquers** and **reaction wheels** to bring the CubeSat to a stable state, with realistic constraints on actuator capabilities.  

## 📌 Objectives  

The goal of this project is to **implement and evaluate two detumbling algorithms**:  

1️⃣ **B-dot Control (Magnetorquers)**: A proportional controller that uses the rate of change of the magnetic field to reduce angular velocity.  
2️⃣ **PD Control (Reaction Wheels)**: A proportional-derivative controller that uses reaction wheels to bring the satellite to a stable orientation.  

The expected outcome is to **reduce the CubeSat's rotational rates (ω) to near zero** and stabilize its **quaternion orientation (q)** over time, ensuring successful detumbling.  

## 📂 Project Structure  
```
├── solver.m                # Main script to run the simulation
├── attitude_model.m        # Satellite attitude dynamics model
├── control_algorithm.m     # Attitude control algorithm
├── Plots                   # Folder with all plots obtained from different Gains
└── README.md               # This file
```

## 🚀 How to Run  

### 1️⃣ Clone the Repository  
```bash
git clone https://github.com/JorgeACM/CubeSat-Detumbling.git
cd CubeSat-Detumbling
```

### 2️⃣ Open MATLAB and Set the Working Directory
### 3️⃣ Run the Simulation
Execute the **solver.m** script to simulate the detumbling process.
### 4️⃣ View the Results  
The script generates **plots** for:  
- **Rotational rates (ω) decay over time**  
- **Quaternion evolution (q) reaching a stable attitude**  
- **Reaction wheel momentum changes**  

Additionally, the simulation calculates:  
- **Settling time**, measuring how long it takes for the satellite to stabilize.

## 🎯 Control Strategies  

### 🔹 Configuration 1: Magnetorquers (B-dot Control)  
- Uses three magnetorquers (one per axis).  
- Maximum dipole moment: ±1 Am².  
- Relies on the **measured magnetic field (Bb)** in the body frame.  

### 🔹 Configuration 2: Reaction Wheels (PD Controller)  
- Uses three reaction wheels (one per axis).  
- **Max torque**: ±0.001 Nm.  
- **Max angular momentum**: ±0.1 Nms.  
- Adjusts torque based on **quaternion error and angular velocity**.  

To switch between control methods, modify `solver.m`:  

```matlab
% control = "magnetorquer1";  % B-dot
% control = "magnetorquer2";  % PD with magnetorquers
control = "wheels";         % PD with reaction wheels
```
## 📊 Performance Evaluation  

The performance of each detumbling algorithm is evaluated based on:  

- **Rotational velocity decay**: How fast the angular velocity (ω) approaches **zero** over time.  
- **Quaternion stabilization**: Ensuring the CubeSat achieves a **stable final orientation** by aligning with the desired quaternion (q).  
- **Settling time measurement**: Determining how long the stabilization process takes, measured using the system’s response.  
- **Actuator constraints compliance**: Ensuring that the **commanded torque and dipole moments** remain within the physical limits of the reaction wheels and magnetorquers.  

## 📚 Dependencies  

- MATLAB  
- **Control System Toolbox** (recommended)  
- **Aerospace Toolbox** (optional, for advanced simulations)  

## 🔍 Future Improvements  

🔹 **Real sensor data integration** for real-time control evaluation.  
🔹 **Implementation on embedded systems** to test in hardware-in-the-loop simulations.  
🔹 **Testing with different CubeSat configurations** to validate robustness under varying conditions.    

## 📜 License  

This project was developed as part of the **Advanced Guidance, Navigation, and Control** coursework for the **MSc Space Engineering** program at the **University of Surrey**.  

The code is **open-source** and available under the **MIT License**.  

---

🚀 **Developed by Jorge Chavarín** | 2024 | MATLAB Simulation  
