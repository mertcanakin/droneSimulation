# Quadcopter Drone Simulation

This repository contains a MATLAB simulation of a quadcopter with different control strategies and dynamics.

## How to Run
Simply execute either script directly in MATLAB - run `attitude_control.m` for basic attitude stabilization or `position_control.m` for more comprehensive position control scenarios with customizable parameters.

## Features
- **Nonlinear quadcopter dynamics** 
- **Motor dynamics (1st order)**
- **Sensor noise**
- **Wind disturbances**
- **Switchable controller**: Choice of PID, SMC, or INDI (set `controller_type` at the top of the script)
- **Slung load dynamics**

```
quadcopter-control/
├── attitude_control.m 
├── position_control.m         # Main simulation scripts
├── functions/
│   ├── slung_load_dynamics.m  # Slung dynamics calculation
│   ├── slung_compensator.m    # Swing damping
│   ├── rotor_thrust_mapping.m # Thrust allocation matrix
│   ├── helix_trajectory.m     # Helical path generator
│   ├── lpf.m                  # Low pass filter
│   ├── sat.m                  # Saturation function
│   └── log_and_plot.m         # Results visualization
├── logs/                      # Logs are automatically recorded here
├── figures/                   # Figures are automatically recorded here
```

## System Assumptions

### Quadrotor Assumptions
- The quadrotor is modeled as a rigid body

- 1st order motor dynamics are included

- Sensor noises are incorporated in the model

- Wind disturbances are applied as both vertical forces and torques

### Slung Load Assumptions
- The cable has constant length and is massless

- The load is modeled as a point mass

- Movement in the x–z and y–z planes is independent (decoupled dynamics)


## Dynamics Overview

### Translational Dynamics
The translational accelerations are computed as:
```
ax = ( (-cos(phi)*sin(theta)*cos(psi) - sin(phi)*sin(psi)) * Fz ...
      + Fx_slung + Fx_wind ) / m_total;

ay = ( (-cos(phi)*sin(theta)*sin(psi) + sin(phi)*cos(psi)) * Fz ...
      + Fy_slung + Fy_wind ) / m_total;

az = ( cos(phi)*cos(theta)*Fz + Fz_slung + Fz_wind ) / m_total - g;
```

### Rotational Dynamics
The torques about each axis are computed from the differences in rotor thrusts, arm length, and yaw drag coefficient. Angular accelerations are then:
```
dp = tau_phi / Ixx
```
(similar for dq, dr)

### Motor Dynamics
Each motor's thrust is modeled as a first-order lag:
```
T_dot = (T_command - T_actual) / tau_motor
```
This simulates the delay between commanded and actual thrust.

### Sensor Noise
Sensor measurements for altitude, velocity, angles, and angular rates are corrupted with Gaussian noise to simulate real-world sensor imperfections.

### Slung Load Dynamics
```
ddalpha = - (g / L) * sin(alpha) ...  % gravity effect
          - d * dalpha ...            % damping
          - ax_est / L;               % quad acceleration

% Rotational acceleration in y axis
% Normally coupled but due to singularity it was assumed as decoupled
ddbeta = - (g / L) * sin(beta) ...    
         - d * dbeta ...           
         - ay_est / L;   
```
<div align="center">
  <img src="quad_animation.gif" alt="Result" width="40%">
</div>  
