function VehicleMotion(MotionConfig)
%% Explanation
% This function loads parameters, simulates motion according to
% motionConfig and logs states to vehiclemotion.mat

% Declare global variables
global MInv DisCMLThruster DisCMRThruster

SimTime = MotionConfig.SimTime;
PlantSamplingTime = MotionConfig.PlantSamplingTime;
LThrusterForce = MotionConfig.LThrusterForce;
RThrusterForce = MotionConfig.RThrusterForce;
EnableCCForces = MotionConfig.EnableCCForces;
EnableRestoringForces = MotionConfig.EnableRestoringForces;
EnableDampingForces = MotionConfig.EnableDampingForces;
EnableAirDragForces = MotionConfig.EnableAirDragForces;

Time = 0;
States = MotionConfig.InitialCondition; %Columns: X,Y,Z,phi,theta,psi in Earth 
...fixed coordinates and surge,sway,heave,roll,pitch,yaw speeds in Body coor.
StatesInE = [States(1:6); B2E(States)*States(7:end)]; %Columns: X,Y,Z,
...theta,phi,psi and its derivatives in Earth fixed coordinates.
StatesInB = [zeros(6,1); States(7:end)];  %Columns: first 6 columns
...are NOTHING(only zeros),surge,sway,heave,roll,pitch,yaw speeds in Body Coord.  

% For debug purposes only
StateLog = [Time States']; % First entry of a row represents time stamp,
...rest of the row consists of state information (i.e., States')
StateLoginE = [Time StatesInE'];          % debugTool
StateLoginB = [Time StatesInB'];          % debugTool
FttLog = [Time zeros(6,1)'];
FccLog = [Time zeros(6,1)'];
FwdLog = [Time zeros(6,1)'];
FadLog = [Time zeros(6,1)'];
FgLog = [Time zeros(6,1)'];

%% Simulate Motion
while (Time<SimTime)
    
    % Generate Thuster Forces
    LThrusterTorque = cross(DisCMLThruster,LThrusterForce)'; % Torque applied by left thruster around CM
    RThrusterTorque = cross(DisCMRThruster,RThrusterForce)'; % Torque applied by right thruster around CM
        
    LThrusterTotalForce = [LThrusterForce; LThrusterTorque]; % ThrusterForce= (Fx,Fy,Fz,Tx,Ty,Tz)'
    RThrusterTotalForce = [RThrusterForce; RThrusterTorque]; % ThrusterForce= (Fx,Fy,Fz,Tx,Ty,Tz)'
    Ftt = LThrusterTotalForce+RThrusterTotalForce;
    
    % Generate Velocity Transformation Matrix
    Jn = B2E(States); 
    
    %Generate Coriolis and Centripetal Forces Matrix
    Fcc = CCForces(States, EnableCCForces);
    
    %Generate Restoring Forces
    Fg = RestoringForces(States, EnableRestoringForces);
    
    %Generate Damping Forces
    Fwd = DampingForces(States, EnableDampingForces);
    
    %Generate Air Drag Forces
    Fad = AirDragForces(States, EnableAirDragForces);
    
    % Generate StateDependentUpdate
    StateDependentUpdate = [Jn*States(7:end); zeros(6,1)];
    
    % Generate ForceDependentUpdate
    ForceDependentUpdate = [zeros(6,1); MInv*(Ftt+Fwd+Fad+Fg-Fcc)];

    % Update States
    States = States+PlantSamplingTime*ForceDependentUpdate+PlantSamplingTime*StateDependentUpdate;   
    StatesInB = [zeros(6,1); States(7:end)];
    StatesInE = [States(1:6); Jn*States(7:end)];
        
    % Update Time
    Time = Time+PlantSamplingTime;
    
    % Log States
    StateLog = [StateLog; Time States'];
    StateLoginE = [StateLoginE; Time StatesInE'];
    StateLoginB = [StateLoginB; Time StatesInB'];
    FttLog = [FttLog; Time Ftt'];
    FccLog = [FccLog; Time (-Fcc)'];
    FwdLog = [FwdLog; Time Fwd'];
    FadLog = [FadLog; Time Fad'];
    FgLog = [FgLog; Time Fg'];
    
end

save('VehicleMotion.mat','StateLog','StateLoginE','StateLoginB',...
    'FttLog', 'FccLog', 'FwdLog', 'FadLog', 'FgLog');

end

