# Helicopter-Simulation #

[![Codacy Badge](https://api.codacy.com/project/badge/Grade/dd8d6dd64aee43eda773c641c96de2d1)](https://www.codacy.com/app/DanIsraelMalta/Helicopter-Simulation?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=DanIsraelMalta/Helicopter-Simulation&amp;utm_campaign=Badge_Grade)

## Description ##

A complete rigid body six degrees of freedom helicopter simulation (ANSI-C) including:
* Main rotor model (including a linear rotor flapping model accounting for coupled servo
  rotor dynamics and mechanical feedback by the gyroscopic effect of the
  stabilizer bar, e.g. - fly-bar)
* Both main & tail rotor include a combined finite element and blade momentum theory model for the
  calculation of a rotor thrust, power and torque
* Vertical & horizontal stabilizer simulation (full aerodynamics)
* ground contact/collision simulation
* Opengl based "outside world" and "inertial navigation system" windows
* Second order (non linear) servo model
* WGS-84 gravity model
* Dryden (linearized) wind model
* Standard atmosphere model (ISA)
* Equations of motion integration using 4th order Runge-Kuta
* IMU emulator
* A place to write the flight control code...

![Alt text](https://cloud.githubusercontent.com/assets/5231886/6562224/f2eb1b14-c69f-11e4-8234-10785c2cbba2.png)

## Project setup ##

To compile the simulator *CMake* (https://www.cmake.org/) is required.
*CMake* is a tool to create a compiler-dependant project file and it is cross platform.

Basic usage:
1. Open *CMakeLists.txt* file in the project folder with a text editor and tune the "Configuration section" based on your specific needing and configuration.
2. Download and install *CMake*.
3. Open a command prompt or a terminal and go to the project source directory.
4. Run "cmake ." without quotes. The *period* is important!
5. Check for any error message.
6. If no errors, a compiler-dependant project file will be available on project folder (e.g. .sln file for Visual Studio or Makefile for linux).
7. *Visual studio only*: Open HelicopterSimulation.sln and set "HelicopterSimulationSim" as startup project. If you disabled the "HelicopterSimulationSim" project from the CMakeLists.txt file skip this step.

Alternatively you can use [CMake GUI](https://www.google.com/search?q=cmake+gui) to avoid using command prompt or the terminal.

**Note:** Even if *CMake* is cross platform and can create a Makefile on linux, the source code for this project has not been tested on platforms different than Windows. Compilation errors may (and probably will) occurs.

## Running the simulator ##
*Empty section*
