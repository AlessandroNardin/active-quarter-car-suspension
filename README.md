# active-quarter-car-suspension

Project developed for the *Identification, Estimation, and Robust Control* course of the Master's Degree in Automation and Robotics, academic year 2025/2026.

This repository contains a MATLAB/Simulink implementation of an active quarter-car suspension system, including Kalman filtering and robust control design.

## Repository Structure

The repository is organized to keep reusable Simulink libraries, validation models, MATLAB logic, and generated results clearly separated. This structure improves navigation, maintainability, and extensibility as the identification and control stages evolve.

Note: Some folders may be missing because Git does not track empty directories.

```text
project-root/
├── libraries/
├── models/
├── core/
├── scripts/
└── params.m
```

### `libraries/`

Reusable Simulink libraries containing the main building blocks of the project. Each library is stored in its own dedicated subfolder, such as `lib_plant`, `lib_sensors`, or `lib_filters`.

Each subfolder typically includes all files required to use that library, such as the Simulink model itself and any supporting `.slblock` or configuration files.

Typical contents:
- `lib_plant.slx`: Quarter-car plant model
- `lib_sensors.slx`: Sensor models
- `lib_filters.slx`: Filter-related blocks (if implemented in Simulink)
- Additional reusable components used across multiple models

### `models/`

Standalone Simulink models used for validation, integration, and testing.

Typical contents:
- Plant validation models
- Single-sensor validation models
- Multi-sensor validation models
- Estimation test benches (EKF, RTS smoother, and alternative filtering methods)

### `core/`

MATLAB `.m` files containing the main numerical and algorithmic logic. This folder is intended for functions shared across scripts and Simulink components to keep the implementation modular and reusable.

Some functions currently defined inside scripts or MATLAB Function blocks may be moved here in the future to improve structure and reduce duplication.

Typical contents:
- Plant dynamics functions
- Sensor models implemented in MATLAB
- EKF prediction and update routines
- Jacobian computation functions
- RTS smoother implementation
- Shared utilities for scripts and Simulink blocks

### `scripts/`

Entry-point MATLAB scripts used to run simulations, post-process results, and generate plots.

Typical contents:
- Parameter initialization
- Plant and sensor validation runs
- EKF and RTS execution scripts
- Comparison, analysis, and plotting scripts

### `params.m`

Single source of truth for:
- Nominal plant parameters
- Sensor settings
- Filter tuning values

All Simulink models and MATLAB scripts should load this file before running simulations to ensure consistency across the project.

## Git Setup

- Install MATLAB
- Install Git (Git Bash recommended)

### Configure MATLAB

- On Windows, enable long paths:
  https://it.mathworks.com/help/matlab/matlab_prog/set-up-git-source-control.html#mw_4e40b00a-d620-4cda-97f1-01589bd41cc6

- Run the following command in MATLAB to enable automatic merge configuration:
```matlab
comparisons.ExternalSCMLink.setupGitConfig();
```

## Git Workflow

### First-time project setup

- In MATLAB: **New → Git Clone**
- Enter the repository HTTP URL
- If the project does not open automatically:
  - Double-click `active-quarter-car-suspension`
  - Close the project setup prompt if it appears

### Working on the project

- Open the project in MATLAB
- Pull the latest changes:
  - Right-click → Version Control → Pull
- Make your changes
- Ensure all new files are tracked:
  - Right-click → Version Control → Add (if needed)
- Commit your changes:
  - Right-click → Version Control → Commit
  - Write a clear and descriptive commit message
  - Note: Avoid committing temporary files
- Push your changes:
  - Right-click → Version Control → Push

## Notes

- On your first push, you will be asked for a GitHub token
- You can generate one here:
  https://github.com/settings/tokens