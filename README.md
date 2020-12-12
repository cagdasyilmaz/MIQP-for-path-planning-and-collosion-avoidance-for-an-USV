# MIQP-for-path-planning-and-collosion-avoidance-for-an-USV

In these repository, Mixed-Integer-Quadratic-Programming (MIQP) is utilized as an high level controller. This controller produces reference yaw angle and reference surge speed values for low level Linear Model Predictive Controller.

At the end of the simulation, graphs obtained from logs are automatically plotted. 

_USSVPlantMPC.m_ file should be run to start the simulation.

# Dependencies

**Simulation via MATLAB & YALMIP (For this simulation MATLAB should be installed on your computer!)**
1. Find a suitable folder for yalmip installation and create a folder, go to the folder in the MATLAB command window
2. Download YALMIP from website: https://yalmip.github.io/download/ and copy this .zip file to the folder above
3. Copy and past the following 3 lines MATLAB command window

>> unzip('YALMIP-master.zip','yalmip')

>> addpath(genpath([pwd filesep 'yalmip']));

>> savepath

4. Test YALMIP: run the following command in MATLAB command window, check available optimization tools among lists
> yalmiptest
