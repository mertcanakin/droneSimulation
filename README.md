# Quadcopter Drone Simulation

This repository contains a MATLAB simulation of a quadcopter with two control strategies: PID and Sliding Mode Control (SMC). The simulation models the dynamics, motor dynamics, sensor noise.

## Features
- **Nonlinear quadcopter dynamics** 
- **Motor first-order dynamics
- **Sensor noise**
- **Switchable controller**: PID or SMC (set `controller_type` at the top of the script)

## How to Run
1. Open `droneSimulation.m` in MATLAB.
2. Set `controller_type = 0` for PID or `controller_type = 1` for SMC at the top of the script.
3. Run the script. Plots will be generated at the end of the simulation.

## Dynamics Overview
The quadcopter is modeled with the following states:
- **z**: altitude
- **phi, theta, psi**: roll, pitch, yaw angles
- **p, q, r**: angular rates (roll, pitch, yaw)

### Translational Dynamics
The vertical (z) acceleration is computed as:
```
az = (sum of rotor thrusts) / mass - gravity
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

## PID Controller
The PID controller computes control inputs based on the error between desired and measured states:
- **Altitude**: PID on altitude error
- **Attitude**: PD on roll, pitch, and yaw errors

The control law for altitude, for example, is:
```
T_total = m * (g + Kp_z*e_z + Kd_z*de_z + Ki_z*int_ez)
```
where `e_z` is the altitude error.

## Sliding Mode Control (SMC)
The SMC controller uses sliding surfaces for each controlled variable:
```
s = lambda*e + de
```
where `e` is the error and `de` its derivative. The control law includes a discontinuous term to drive the system to the sliding surface:
```
T_total = m * (g + lambda_z*de_z + K_z*sign(s_z))
```
A saturation function is used to reduce chattering.
