UAV-Project
===========
Small UAV project

Creating a flight controller from scratch.

Learning goals
- PCB layouting basics
- microcontroller programming
- source code config control
- interfacing with sensors
- data aqcuisition
- analyzing flight dynamics
- digging into control theory
- A/C controller design

Overview of components
- Multiplex Fun Cub R/C airplane
- ATMEGA32
- tbc

Accomplished
- sensor interfacing
  - imu (I2C)
  - pitot tube (ADC)
  - barometric pressure sensor (I2C)
  - GPS (Serial)

- A/C control
  - remote control signals into MCU
  - servo control output
  - manual flight mode
  - partly autonomous control (damped flight, auto flight)

- A/C flight dynamics
  - logged open loop system responses
  - data analysis in Scilab
  - linearized A/C model simulation in Scilab
  - closed loop control simulation
  - control parameters identified for stabilized flight
  
Outlook
- complete autonomous control of A/C attitude
- GPS waypoint autonomous flight
