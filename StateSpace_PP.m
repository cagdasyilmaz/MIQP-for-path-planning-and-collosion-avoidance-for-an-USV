function [ Ad, Bd, Cd] = StateSpace_PP(Ts_PP)

%% Explanation
% This function sets initial parameters for statespace, constraints of MILP
% MPC.

% Inputs:
% Ts_PP: path planner sampling time

% Outputs:
% Discrete time state space matrices: Ad, Bd, Cd

%% Step-1: Initialize StateSpace Model For Path Planner
% state x = [ position_x, velocity_x, position_y, velocity_y]; in Ned Frame
% input u = [ acceleration_x, acceleration_y]; in body frame

% Declare continuous time state space model of path planner
Ad = [1 Ts_PP 0 0;
      0 1 0 0;
      0 0 1 Ts_PP;
      0 0 0 1]; 
  % Discrete time state transition matrix for path planner
 
Bd = [0.5*Ts_PP*Ts_PP 0;
      Ts_PP 0;
      0  0.5*Ts_PP*Ts_PP;
      0 Ts_PP]; 
  % Discrete time input matrix for path planner
 
Cd = [1 0 0 0 ;
      0 0 1 0]; % Output matrix for path planner.


end