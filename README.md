# 26381 - Valkyrie

## Overview
This repository contains the codebase for our FTC robot, designed to participate in the FIRST Tech Challenge "INTO THE DEEP". 
The project implements advanced autonomous and teleoperated control mechanisms, modular subsystems, and optimized commands for 
efficient robot operation.


## Project Structure

The project is divided into the following key packages and directories:

### 1. **Autonomous**
Contains all the autonomous routines and related functionality.
- **AutonomousIndex**:  
  Includes predefined paths and commands for autonomous behavior.
    - `ChassisPaths`: Defines chassis movement paths for navigation.
    - `RamsetteCommand`: Implements path-following logic using Ramsete.
    - `TurnToAngle`: Provides functionality to turn the robot to a specific angle.
    - `GrabThreeSpecimensTwo`: Routine for grabbing multiple game elements.
    - `HighBasketAndPark`: Routine for scoring in the high basket and parking.
    - `HighChamber2HP`: Strategy for handling high chamber interactions.

### 2. **Commands**
Contains modular command implementations for various robot actions.
- **Arm**: Commands to control the robot's arm.
- **Baskets**: Commands for interacting with baskets or scoring elements.
- **Chambers**: Commands for handling the robot's chambers.
- **Climber**: Commands for climbing operations (if applicable).
- **Elevator**: Commands to control the elevator subsystem.
- **GroundGrab**: Commands for picking up elements from the ground.
- **Intake**: Commands for the intake mechanism.
- **Wrist**: Commands for precise wrist movements.
- **Drive**: Commands for driving mechanisms and maneuvers.
- **StowAll**: Command to stow all active subsystems.
- **WaitForButton**: Command to wait for user input.

### 3. **Subsystems**
Encapsulates the hardware and software logic for each robot subsystem.
- **Arm**: Logic and controls for the arm.
- **Chassis**: Movement and drivetrain logic.
- **Constants**: Defines constants used throughout the project (e.g., PID values, dimensions).
- **Elevator**: Logic for the elevator subsystem.
- **Intake**: Functionality for the intake mechanism.
- **Wrist**: Logic for wrist control.
- **MainSystem**: Centralized system integration and coordination.

## Key Features

- **Autonomous Navigation**:  
  Utilizes Ramsete path-following and trajectory generation for smooth and precise movement.

- **Modular Design**:  
  The code is designed with modularity in mind, allowing for easy testing and integration of individual components.

- **Subsystem-Based Architecture**:  
  Implements the command-based programming paradigm, adhering to FTC and WPILib standards.

- **Reusable Constants**:  
  Centralized constants make tuning and updates more efficient.

- **Flexible Commands**:  
  Commands are designed to handle specific tasks, ensuring clarity and reusability.

## Requirements

- **Hardware**:
    - FTC Control Hub or Expansion Hub
    - Motors, Servos, Sensors (specific hardware based on robot design)

- **Software**:
    - Android Studio
    - FTC SDK
    - Java 8+

## Usage
- **Autonomous**:
    - Select the desired autonomous routine from the driver station during initialization.

- **TeleOp**:
    - Operate the robot using the assigned gamepad controls.

## License
This project is licensed under the [MIT License](https://mit-license.org/).