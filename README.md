# Helicopter-Simulation

A complete rigid body six degrees of freedom helicopter simulation (ANSI-C) including:
* Main rotor model (including a linear rotor flapping model accounting for coupled servo
  rotor dynamics and mechanical feedback by the gyroscopic effect of the stabilizer bar ("fly-bar"))
* Tail rotor model
* Vertical & horizontal stabilizer simulation (full aerodynamics)
* ground contact/collision simulation
* Opengl based "outside world" and "inertial navigation system" windows.
* Second order (non linear) servo model
* WGS-84 gravity model
* Dryden (linearized) wind model
* Equations of motion integration using 4th order Runge-Kuta
* IMU emulator
* A place holder to write the flight control code.