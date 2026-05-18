# UAV Simulation

MATLAB simulation framework for fixed-wing and multirotor UAVs with multiple control strategies.

## Platforms

- **Multirotor**: Quadcopter with position and attitude control
- **Fixed-wing**: Longitudinal dynamics with pitch and airspeed control

## How to Run

Run the desired main script in MATLAB:
- `MC_position_control.m` — quadcopter position tracking
- `FW_pitch_control.m` — fixed-wing pitch and airspeed control

Set `controller_type` at the top of the script to switch between controllers where applicable.


## Features
- **Nonlinear quadcopter dynamics** 
- **Motor dynamics**
- **Sensor noise**
- **Wind disturbances**
- **Switchable controller**: Choice of PID, SMC, or INDI (set `controller_type` at the top of the script)
- **Slung load dynamics**

## Repository Structure
```
quadcopter-control/
├── MC_position_control.m       # Multirotor main script
├── FW_pitch_control.m          # Fixed-wing main script
├── functions/
│   └── controller_class/       # Controller classes (Base, PID, SMC, INDI)
├── logs/                       # Auto-generated logs
└── figures/                    # Auto-generated plots
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

<div align="center">
  <img src="quad_animation.gif" alt="Result" width="40%">
</div>  
