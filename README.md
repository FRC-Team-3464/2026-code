# FRC Team 3464 "Sim-City" 2026 Robot Code
This repository contains all the robot code used by Team 3464 "Sim-City" during the 2026 FRC season "REBUILT."
<br> <br>
The code has been extensively documented to be used as reference in training and for future seasons.

## Structure

### Root Repository Structure
- `.github` - CI/CD workflows for GitHub
- `.vscode/` and `.wpilib` - Configurations and settings for VSCode and WPILib respectively
- `gradle/` - Gradle configuration
- `src/` -  All code for controlling the robot
- `vendordeps/` - Third-party software imports (REV Robotics/CTRE Hardware APIs, AdvantageKit, PathPlanner, etc.)
- Everything else - misc. configuration files, licenses, and other

### Robot Code Structure
After traveling down the `src/main/java/frc/robot` rabbit hole, you will find the main robot code. It is organized as follows:
- `commands/` - Robot-wide commands or commands so large they deserve their own files
- `control/` - Interfaces and classes for handling driver input and other bindings
- `subsystems/` - Pretty self-explanatory: contains all IO implementations, constants, and classes for each subsystem
- `util/` - Any misc. files that contain utility classes to make programming the robot a little easier
- `Constants.java` - Robot-wide constants
- `Main.java` - Program entry point (don't modify)
- `Robot.java` - Main robot code class
- `RobotContainer` - Holds all subsystem instances and some periodic functions
- `RobotState` - The single source of truth for robot pose and anything that depends on it
- `RobotVisualizer` - Logs mechanism poses so they can be visualized in AdvantageScope