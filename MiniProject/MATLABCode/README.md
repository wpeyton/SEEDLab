# MATLAB Code

**This folder contains:**

**PIcontrol.slx**

  Simulink block diagram for the closed loop step response
    - Allows PI controller to be tuned to desired overshoot and rise time

**StepReponse.xlsx**

  Excel file containing data from open and closed step responses
  
**openLoop.slx**

  Simulink block diagram for the open loop step response
    - Helps to determine if estimated transfer function matches experimental data

**stepResponse.m**

  MATLAB script to compare experimental and simulation data
    - Graphs open loop time vs velocity data to determine if transfer function is accurate
    - Graphs closed loop time vs position data to determine if PI controller is tuned correctly
