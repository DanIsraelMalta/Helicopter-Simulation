# Helicopter-Simulation

[![Codacy Badge](https://api.codacy.com/project/badge/Grade/dd8d6dd64aee43eda773c641c96de2d1)](https://www.codacy.com/app/DanIsraelMalta/Helicopter-Simulation?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=DanIsraelMalta/Helicopter-Simulation&amp;utm_campaign=Badge_Grade)

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
