function [States,StatesInE, Tot_Forces, Dist_Forces] = VehicleMotion_MPC...
                                   (LThrusterForceX, RThrusterForceX,...
                                    PrevStates,...
                                    Ts,...
                                    SimTime)
                                
%% Explanation
% This function loads parameters, simulates motion according to
% motionConfig and logs states to vehiclemotion.mat

% Declare global variables
global MInv

PlantSamplingTime = Ts;
EnableCCForces = true;
EnableDampingForces = true;
EnableAirDragForces = true;
EnableThrusterForces = true;
EnableDisturbanceForces = false;
States = PrevStates;

...fixed coordinates and surge,sway,heave,roll,pitch,yaw speeds in Body coor.
% StatesInE = [States(1:6); B2E(States)*States(7:end)]; %Columns: X,Y,Z,
% ...theta,phi,psi and its derivatives in Earth fixed coordinates.
% StatesInB = [zeros(6,1); States(7:end)];  %Columns: first 6 columns
% ...are NOTHING(only zeros),surge,sway,heave,roll,pitch,yaw speeds in Body Coord.  
   
%Generate Thruster Forces
Ftt = MPC_ThrusterForces(LThrusterForceX,...
                         RThrusterForceX,...
                         EnableThrusterForces);

% Generate Velocity Transformation Matrix
Jn = MPC_B2E(States); 

%Generate Coriolis and Centripetal Forces Matrix
Fcc = MPC_CCForces(States, EnableCCForces);

%Generate Damping Forces
Fwd = MPC_DampingForces(States, EnableDampingForces);

%Generate Air Drag Forces
Fad = MPC_AirDragForces(States, EnableAirDragForces);

%Generate Disturbance Forces
Fdis = MPC_DisturbanceForces(States,SimTime,EnableDisturbanceForces);

% Generate StateDependentUpdate
StateDependentUpdate = [Jn*States(4:end); zeros(3,1)];

% Generate ForceDependentUpdate
ForceDependentUpdate = [zeros(3,1); MInv*(Ftt+Fwd+Fad-Fcc+Fdis)];

Dist_Forces = Fdis';

Tot_Forces = (Ftt+Fwd+Fad+Fdis-Fcc)';

% Update States
States = States + PlantSamplingTime * ForceDependentUpdate + ...
                  PlantSamplingTime * StateDependentUpdate;   
%StatesInB = [zeros(6,1); States(7:end)];
StatesInE = [States(1:3); Jn*States(4:end)];

end



