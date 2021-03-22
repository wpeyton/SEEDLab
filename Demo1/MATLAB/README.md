# MATLAB Code

**This folder contains:**

**discreteSystem.slx**

    Simulink block diagram for the entire system
        - Tests the controller code and sample rate

**innerLoopPhi.slx**

    Simulink block diagram for the rotational velocity closed loop step response
        - Allows PI controller to be tuned to desired overshoot and rise time
  
**innerLoopRho.slx**

    Simulink block diagram for the translational velocity closed loop step response
        - Allows PI controller to be tuned to desired overshoot and rise time

**outerLoop.slx**

    Simulink block diagram for the rotational position closed loop step response
        - Allows PD controller to be tuned to desired overshoot and rise time

**stepResponse.m**

    MATLAB script to compare experimental and simulation data
        - Graphs open loop time vs translational velocity data to determine if transfer function is accurate
        - Graphs open loop time vs rotational velocity data to determine if transfer function is accurate
        - Graphs root locus of phi_dot transfer function to help design phi controller

