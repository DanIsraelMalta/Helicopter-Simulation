# Helicopter-Simulation

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
