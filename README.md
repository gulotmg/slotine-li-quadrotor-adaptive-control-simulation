# Slotine-Li Adaptive Control for Quadrotor Trajectory Tracking

This repository contains the **MATLAB/Simulink** implementation and comparative performance analysis of a **Slotine-Li Adaptive Control architecture with an unmodeled disturbance estimator** applied to a quadrotor UAV, submitted as part of the exam of "Distributed and Mobile Robotics" in the MSc in Electronics Engineering at University of Palermo. The system is designed for high-accuracy trajectory tracking under parametric uncertainties (mass and inertia variations) and external atmospheric disturbances (Dryden wind gusts).

A direct performance comparison is conducted against a standard Slotine-Li Adaptive Controller and a classical Proportional-Derivative (PD) Linear Controller.

---
## Getting Started & Simulation Setup

### Prerequisites
* MATLAB R2025a or newer
* Simulink
* Aerospace Blockset *(required for the Dryden Wind Turbulence Model)*

### Running the Simulation

1. Run the initialization script in MATLAB workspace to load estimator initial conditions $\hat{\boldsymbol{\pi}}_0$:
   ```matlab
   run('pi_hat_init.m')
   ```
2. Open and run the Simulink simulation model:
   ```matlab
   open_system('quadrotor_adaptive_control.slx')
   ```
---
## Download and Clone

### Clone with Git

```bash
git clone https://github.com/gulotmg/slotine-li-quadrotor-adaptive-control-simulation.git
cd slotine-li-quadrotor-adaptive-control-simulation
```

### Download as ZIP

[Download ZIP](https://github.com/gulotmg/slotine-li-quadrotor-adaptive-control-simulation/archive/refs/heads/main.zip)

---
## Project Information & Academic Context

* **Authors:** Marco Gulotta, Antonino Daidone
* **Course Supervisor:** Prof. Adriano Fagiolini
* **Institution:** University of Palermo (*Università degli Studi di Palermo*) — MSc in Electronics Engineering

---

## Key Features & Technical Highlights

* **Nonlinear Quadrotor Dynamics:** Full 6-DOF nonlinear model simulating real physical constraints and spatial coupling.
* **Slotine-Li Adaptive Control:** Online estimation of unknown physical parameters ($\hat{\boldsymbol{\pi}}$) with a $\sigma$-leakage modification to guarantee parameter boundedness.
* **Disturbance Estimator / Observer:** Real-time extraction and compensation of unmodeled external forces (e.g., atmospheric wind gusts).
* **MIL-F-8785C Dryden Wind Model:** Realistic atmospheric turbulence injection applied at $t=85\text{ s}$.
* **Physical Constraints:** Actuator saturation limits (thrust/torque bounds) and motor power loss modeling (up to 20%).
* **Comparative Benchmarking:** Simultaneous simulation comparing:
  1. *Adaptive Control + Disturbance Estimator (Nonlinear Plant)*
  2. *Standard Adaptive Control (Nonlinear Plant)*
  3. *Proportional-Derivative (PD) Control (Linearized Plant)*

---

## Simulink Architecture Overview

The Simulink model (`quadrotor_adaptive_control.slx`) is structured into three primary operational sections:

```text
[ Trajectory Generation ] ---> [ 3 Parallel Controller Branches ] ---> [ Scopes & Performance Metrics ]
```

### 1. Trajectory Generation (`Desired trajectories and speed`)
Generates continuous desired position, velocity, and acceleration references $[X_d, Y_d, Z_d]^T$ combining ramps, sine waves, and step functions to emulate complex flight maneuvers.

### 2. Parallel Control Branches
* **Top Branch:** Nonlinear Quadrotor + Adaptive Controller + Disturbance Estimator.
* **Middle Branch:** Nonlinear Quadrotor + Standard Slotine-Li Adaptive Controller.
* **Bottom Branch:** Linearized Quadrotor + Classic Proportional-Derivative (PD) Controller.

### 3. Disturbances & Parametric Variations
* **Mass & Inertia Steppers:** Step changes in total mass $m_t$ and rotational inertia tensor elements $(I_{xx}, I_{yy}, I_{zz})$.
* **Wind Stepper:** Enables the Dryden wind turbulence model at $t=85\text{ s}$ per MIL-F-8785C aeronautical standards.
* **Motor Power Loss Subsystem:** Simulates a sudden 20% loss of motor efficiency across individual rotors.

---

## Mathematical Formulation

### Slotine-Li Adaptive Control Law
The control torque and total thrust vector $\boldsymbol{\tau}$ is computed using the regressor matrix $\mathbf{Y}$ and the parameter estimate vector $\hat{\boldsymbol{\pi}}$:

$$\boldsymbol{\tau}=\mathbf{Y}\hat{\boldsymbol{\pi}}-\mathbf{K}_d\mathbf{e}-\mathbf{K}_s\mathbf{s}$$

Where:
* $\mathbf{e}=\mathbf{q}-\mathbf{q}_d$ represents the generalized trajectory error vector.
* $\mathbf{s}=\dot{\mathbf{e}}+\boldsymbol{\Lambda}\mathbf{e}$ defines the sliding surface manifold vector.
* $\hat{\boldsymbol{\pi}}$ is updated online using a robust adaptation law with $\sigma$-leakage to prevent parameter drift:

$$\dot{\hat{\boldsymbol{\pi}}}=\begin{cases}-\mathbf{K}_{\pi}(\mathbf{Y}^T\mathbf{s})-\sigma_{\pi}(\hat{\boldsymbol{\pi}}-\boldsymbol{\pi}_{\text{nom}})&\text{if }\|\mathbf{s}\|>0.02\\\\-\sigma_{\pi}(\hat{\boldsymbol{\pi}}-\boldsymbol{\pi}_{\text{nom}})&\text{if }\|\mathbf{s}\|\le0.02\end{cases}$$

### Classic Linear PD Control Law
For the linearized quadrotor dynamics, the vertical thrust variation $\Delta F$ and orientation control torques $(\tau_\phi,\tau_\theta,\tau_\psi)$ are governed by:

$$\Delta F=m(\ddot{z}_d-k_{v,z}(\dot{z}-\dot{z}_d)-k_{p,z}(z-z_d))$$

$$\tau_\phi=I_{xx}(\ddot{\phi}_c-k_{v,\phi}(\dot{\phi}-\dot{\phi}_c)-k_{p,\phi}(\phi-\phi_c))$$

$$\tau_\theta=I_{yy}(\ddot{\theta}_c-k_{v,\theta}(\dot{\theta}-\dot{\theta}_c)-k_{p,\theta}(\theta-\theta_c))$$

$$\tau_\psi=I_{zz}(-k_{v,\psi}\dot{\psi}-k_{p,\psi}(\psi-\bar{\psi}))$$

---

## Performance Results & Comparative Summary

| Metric / Scenario | Adaptive + Disturbance Estimator | Standard Adaptive Control | Linear PD Controller |
| :--- | :--- | :--- | :--- |
| **Parametric Mass/Inertia Step** | Fast convergence, minimal transient error spike | Good parameter adaptation, minor error transient | Degraded stability, persistent steady-state offset |
| **Dryden Wind Gusts ($t=85\text{ s}$)** | Near-zero steady-state tracking error | Uncompensated drift under continuous wind forces | High amplitude oscillations and persistent tracking error |
| **Motor Power Loss (20%)** | Robust compensation with minimal trajectory disturbance | Moderate trajectory tracking degradation | Severe trajectory deviation and potential instability |

* **Linear controller PD position tracking error**
<img width="1917" height="953" alt="linear" src="https://github.com/user-attachments/assets/35c5c90b-89b2-4d79-97bb-cdf329119784" />

* **Adaptive controller position tracking error**

<img width="1916" height="952" alt="adaptive" src="https://github.com/user-attachments/assets/b963c039-f1f1-49d5-a236-8701fed5d4f4" />



* **Adaptive controller + disturbance estimator position tracking error**
<img width="1916" height="956" alt="adaptive+estimator" src="https://github.com/user-attachments/assets/2466d990-4d95-4c39-afef-22be3b8a91ed" />




**Conclusion:** Integrating an unmodeled disturbance observer alongside the Slotine-Li adaptive algorithm provides better rejection of both parameteres variations (i.e. mass and inertia) and external unmodeled disturbance forces.




---


## Academic Integrity & Disclaimer
This repository contains the dynamic models, observers, and Slotine-Li adaptive control 
algorithms developed for academic and portfolio demonstration purposes. 

Under the University of Palermo's Academic Integrity and Ethical Regulations, 
students are strictly prohibited from copying, plagiarizing, or submitting any 
part of this material to fulfill requirements for university courses or exams. 
Any unauthorized submission constitutes academic misconduct.


   
