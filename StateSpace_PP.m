function [ Ad, Bd, Cd] = StateSpace_PP( Ts_PP)

%% Explanation
% This function sets initial parameters for statespace, constraints of MILP
% MPC.

% Inputs:
% Ts_PP: path planner sampling time
% Yaw angle of the vehicle

% Outputs:
% Discrete time state space matrices: Ad, Bd, Cd

%% Step-1: Initialize StateSpace Model For Path Planner
% state x = [ position_x, velocity_x, position_y, velocity_y]; in Ned Frame
% input u = [ velocity_x, velocity_y]; in body frame

% Declare continuous time state space model of path planner
Ac = [0 0;
      0 0]; % Continuous time state transition matrix for path planner
 
Bc = [1 0;
      0 1]; % Continuous time input matrix for path planner
 
Cc = [1 0 ;
     0 1]; % Output matrix for path planner.
 
D = zeros(2, 2);  

% Discretize state space model for path planner
[ Ad , Bd, Cd, D] = c2dm(Ac,Bc,Cc,D,Ts_PP,'zoh');

end