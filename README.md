# active-quarter-car-suspension
Project for the Identification, Estimation and Robust Control course, for the Master’s Degree in Automation and Robotics, academic year 2025/2026. MATLAB/Simulink implementation of an active quarter-car suspension system with Kalman filter and robust control design.

## Repository Structure

This repository is organized to keep reusable Simulink components, validation models, MATLAB logic, and generated results clearly separated. The goal is to make the project easy to navigate, version, and extend as the identification and control stages evolve.

```text
project-root/
├── libraries/
├── models/
├── core/
├── scripts/
└── params.m
```

### `libraries/`
Reusable Simulink libraries containing the main building blocks of the project.  
Typical contents include:
- `lib_plant.slx`: the quarter-car plant and its internal subsystems.
- `lib_sensors.slx`: sensor models with realistic noise and non-idealities.
- `lib_filters.slx`: filter-related blocks, if implemented in Simulink.
- `lib_utils.slx`: shared utility blocks such as saturation, noise, or road profile generators.

### `models/`
Standalone Simulink models used for validation and integration tests.  
Typical contents include:
- plant validation models.
- single-sensor validation models.
- multi-sensor validation models.
- estimation test benches for EKF, RTS smoothing, and the alternative filtering method.

### `core/`
MATLAB `.m` files containing the main numerical and algorithmic logic.  
Typical contents include:
- plant dynamics functions.
- sensor models written in MATLAB.
- EKF prediction and update routines.
- Jacobian computation functions.
- RTS smoother implementation.
- small shared utilities used by both scripts and Simulink blocks.

### `scripts/`
Entry-point MATLAB scripts used to run simulations, post-process results, and generate plots.  
Typical contents include:
- parameter initialization.
- validation runs for plant and sensors.
- EKF and RTS execution scripts.
- comparison and plotting scripts.

### `params.m`
Single source of truth for nominal plant parameters, sensor settings, and filter tuning values.  
All Simulink models and MATLAB scripts should load this file before running simulations.
