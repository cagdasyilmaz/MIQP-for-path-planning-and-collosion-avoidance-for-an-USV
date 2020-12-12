function [A_Dis, B_Dis, C_Dis] = ContinousToDiscrete(A,B,C,D,TsPlant)

% This function converts Continous State Space to Discrete State Space

% Creates a state-space model object 
% representing the continuous-time state-space model
Continuous_SYS = ss(double(A),double(B),double(C),double(D));

% Discretizes the continuous-time dynamic system model 
% sys using zero-order hold on the inputs and a sample time of Ts seconds.
Discrete_SYS = c2d(Continuous_SYS,TsPlant);

%extracts the matrix (or multidimensional array) data A, B, C, D 
%from the state-space model (LTI array) sys. If sys is a transfer function 
%or zero-pole-gain model (LTI array), it is first converted to state space

[A_Dis, B_Dis, C_Dis] = ssdata(Discrete_SYS);

end