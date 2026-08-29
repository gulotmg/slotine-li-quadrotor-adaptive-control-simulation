# Slotine-Li Adaptive Control for Quadrotor Trajectory Tracking

This repository contains the MATLAB/Simulink implementation and comparative performance analysis of a Slotine-Li adaptive control architecture with an additional residual-based disturbance compensation branch applied to a quadrotor model.

The system evaluates trajectory tracking under selected mass and inertia variations, external wind-related disturbances and actuator effectiveness variations.

A direct performance comparison is conducted against a standard Slotine-Li Adaptive Controller and a classical Proportional-Derivative (PD) Linear Controller.

---

## Getting Started & Simulation Setup

### Prerequisites

* MATLAB R2025a or newer
* Simulink
* Aerospace Blockset *(required for the Dryden Wind Turbulence Model)*

### Running the Simulation

1. Run the initialization script in the MATLAB workspace to load the initial parameter estimates $\hat{\boldsymbol{\pi}}_0$:

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

* **Nonlinear Quadrotor Dynamics:** 12-state nonlinear rigid-body model including translational and rotational dynamics, attitude kinematics and thrust/orientation coupling.
* **Slotine-Li Adaptive Control:** Online estimation of selected physical parameters $\hat{\boldsymbol{\pi}}$, including mass and principal inertia parameters, using a regressor-based adaptive law with sigma leakage.
* **Residual-Based Disturbance Compensation:** Additional compensation based on a model-derived lateral disturbance residual calculated from simulated acceleration and thrust-related quantities.
* **MIL-F-8785C Dryden Wind Model:** Realistic atmospheric turbulence injection using a continuous Dryden wind-turbulence block at $t=85\,\text{s}$ in the included simulation configuration.
* **Physical Constraints:** Actuator saturation limits and configurable per-motor effectiveness variations, including a 20% effectiveness reduction in the tested configuration.
* **Comparative Benchmarking:** Simultaneous simulation comparing:
  1. *Adaptive Control + Residual Disturbance Compensation (Nonlinear Plant)*
  2. *Standard Adaptive Control (Nonlinear Plant)*
  3. *Proportional-Derivative (PD) Control (Linearized Plant)*

The conclusions apply to the included model, parameter values, controller gains, initial conditions and disturbance scenarios. They are not intended as a formal proof of global stability, general robustness or physical flight performance.

---

## Simulink Architecture Overview

The Simulink model (`quadrotor_adaptive_control.slx`) is structured into three primary operational sections:

```text
[ Trajectory Generation ] ---> [ 3 Parallel Controller Branches ] ---> [ Scopes & Performance Metrics ]
```

### 1. Trajectory Generation (`Desired trajectories and speed`)

Generates continuous desired position, velocity and acceleration references $[X_d,Y_d,Z_d]^T$, combining ramps, sine waves and step functions to represent different simulated flight maneuvers.

### 2. Parallel Control Branches

* **Top Branch:** Nonlinear quadrotor model + adaptive controller + residual-based disturbance compensation.
* **Middle Branch:** Nonlinear quadrotor model + standard Slotine-Li adaptive controller.
* **Bottom Branch:** Linearized quadrotor model + classical PD controller.

### 3. Disturbances & Parametric Variations

* **Mass & Inertia Steppers:** Step changes in total mass $m_t$ and selected rotational inertia parameters $(I_{xx},I_{yy},I_{zz})$.
* **Wind Stepper:** Enables the Dryden wind-turbulence block at $t=85\,\text{s}$ in the included scenario.
* **Motor Effectiveness Variation:** Applies configurable scaling factors to the individual rotor contributions.

---

## Mathematical Formulation

### Slotine-Li Adaptive Control Law

The total thrust and control torques $\boldsymbol{\tau}$ are computed using a regressor matrix $\mathbf{Y}$ and the estimated parameter vector $\hat{\boldsymbol{\pi}}$:

$$
\boldsymbol{\tau} = \mathbf{Y}\hat{\boldsymbol{\pi}} - \mathbf{K}_d\mathbf{e} - \mathbf{K}_s\mathbf{s}
$$

Where:

* $\mathbf{e}=\mathbf{q}-\mathbf{q}_d$ represents the selected generalized trajectory-error vector.
* $\mathbf{s}=\dot{\mathbf{e}}+\boldsymbol{\Lambda}\mathbf{e}$ defines the sliding-surface vector.
* $\hat{\boldsymbol{\pi}}$ contains the online estimates of the selected mass and principal inertia parameters.

The implemented parameter-update law includes sigma leakage:

$$
\dot{\hat{\boldsymbol{\pi}}} =
\begin{cases}
-\mathbf{K}_{\pi}(\mathbf{Y}^{T}\mathbf{s}) - \sigma_{\pi}(\hat{\boldsymbol{\pi}}-\boldsymbol{\pi}_{\mathrm{nom}}) & \text{if } \|\mathbf{s}\| > 0.02 \\
-\sigma_{\pi}(\hat{\boldsymbol{\pi}}-\boldsymbol{\pi}_{\mathrm{nom}}) & \text{if } \|\mathbf{s}\| \le 0.02
\end{cases}
$$

The leakage term is included to limit parameter drift in the implemented simulation. A formal stability or convergence proof is not provided in this repository.

### Residual-Based Disturbance Compensation

The additional compensation branch calculates a lateral disturbance residual from the simulated translational acceleration and the modeled lateral thrust contribution:

$$
\hat{\mathbf{d}}_{xy} = m_0 \begin{bmatrix} \dot{u} \\ \dot{v} \end{bmatrix} - \begin{bmatrix} F_x \\ F_y \end{bmatrix}
$$

The resulting residual is used to modify the nominal lateral acceleration command.

This calculation depends on the nominal mass and on modeling assumptions, including a negligible wind contribution in the vertical channel. The result should therefore be interpreted as a model-based compensation term specific to the included simulation.

### Classic Linear PD Control Law

For the linearized quadrotor dynamics, the vertical thrust variation $\Delta F$ and orientation-control torques $(\tau_\phi,\tau_\theta,\tau_\psi)$ are governed by:

$$
\Delta F = m \left( \ddot{z}_d - k_{v,z}(\dot{z}-\dot{z}_d) - k_{p,z}(z-z_d) \right)
$$

$$
\tau_\phi = I_{xx} \left( \ddot{\phi}_c - k_{v,\phi}(\dot{\phi}-\dot{\phi}_c) - k_{p,\phi}(\phi-\phi_c) \right)
$$

$$
\tau_\theta = I_{yy} \left( \ddot{\theta}_c - k_{v,\theta}(\dot{\theta}-\dot{\theta}_c) - k_{p,\theta}(\theta-\theta_c) \right)
$$

$$
\tau_\psi = I_{zz} \left( -k_{v,\psi}\dot{\psi} - k_{p,\psi}(\psi-\bar{\psi}) \right)
$$

---

## Performance Results & Comparative Summary

| Metric / Scenario | Adaptive + Residual Compensation | Standard Adaptive Control | Linear PD Controller |
| :--- | :--- | :--- | :--- |
| **Parametric Mass/Inertia Step** | Lower transient tracking error in the included simulation | Parameter adaptation with a visible transient | Larger tracking error in the included scenario |
| **Dryden Wind Gusts ($t=85\,\text{s}$)** | Lower simulated tracking error after compensation | Tracking degradation under the same disturbance | Larger simulated tracking error |
| **Motor Effectiveness Reduction** | Better simulated trajectory retention in the included case | Moderate simulated degradation | Larger simulated trajectory deviation |

* **Linear controller PD position tracking error**

<img width="1917" height="953" alt="linear" src="https://github.com/user-attachments/assets/35c5c90b-89b2-4d79-97bb-cdf329119784" />

* **Adaptive controller position tracking error**

<img width="1916" height="952" alt="adaptive" src="https://github.com/user-attachments/assets/b963c039-f1f1-49d5-a236-8701fed5d4f4" />

* **Adaptive controller + residual disturbance compensation position tracking error**

<img width="1916" height="956" alt="adaptive+compensation" src="https://github.com/user-attachments/assets/2466d990-4d95-4c39-afef-22be3b8a91ed" />

The comparisons above are qualitative and refer to the included simulation plots. The repository does not include a statistical study, experimental validation or formal robustness certification.

**Conclusion:** In the included simulation configuration, adding the residual-based disturbance compensation branch reduces the simulated tracking error under the tested disturbance and parameter-variation scenarios compared with the two reference branches. This conclusion is configuration-dependent and does not establish general stability or real-world flight performance.

---

## Academic Integrity & Disclaimer

This repository contains dynamic models, adaptive-control implementations, residual-based disturbance compensation and comparative simulations developed for academic and portfolio demonstration purposes.

The results depend on the selected model, controller gains, initial conditions, saturation limits and disturbance profiles. The model is not a validated flight-control implementation and has not been tested on a physical quadrotor.

Under the University of Palermo's Academic Integrity and Ethical Regulations, students are strictly prohibited from copying, plagiarizing or submitting any part of this material to fulfil requirements for university courses or exams. Any unauthorized submission constitutes academic misconduct.
