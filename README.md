# Soft Robotics MATLAB 


Soft robotics is a relatively new field of robotics with promising future applications. Yet, regarding traditional robotics, the field of soft robotics is still in the state of inception, 
especially regarding modeling and control. Here, we (freely) offer code from MATLAB to model and simulate a soft robot manipulator. To use the provided MATLAB code, follow the steps below

* run *computeDynamics.m* to obtain the dynamics model, which will generate MATLAB code which can be copy-pasted from the command window.
* update the ODE in *simulateDynamics.m* to the desired dynamic model (line 107 - 130). If you use a 2D model, be aware of the state dimension!
* enable **ShowSimulation** in *simulateDynamics.m*, to preview a real-time deformation render.
