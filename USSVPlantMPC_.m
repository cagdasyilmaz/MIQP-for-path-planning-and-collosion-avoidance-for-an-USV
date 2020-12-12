% Initially clear workspace, clear all variable, close all windows
clc;
clear all;
close all;


% Declare global variables ------------------------------------------------
global m  l wi h g roW  MaxThrusterTorque DisCMLThruster...
       DisCMRThruster DistanceCMCOB WindVelocity CDW CDA States Inputs ...
       States_Dot Iz
% -------------------------------------------------------------------------

% Components of state and input vector ------------------------------------
syms  x y psiy u v r  ...
      LThrusterForceX RThrusterForceX real

States = [x y psiy u v r]';
Inputs = [LThrusterForceX; RThrusterForceX];
% -------------------------------------------------------------------------

%% Generate Basic Constant Parameters
% Note1: The body is assumed to be in the shape of rectengular prism
% Note2: Body fixedframe is located at the symmetry center of the 
% rectengular prism.

m = 10; % in kg, total mass of the body with peripherals
l = 0.87; % length in x direction
wi = 0.27; % width in y direction
h = 0.23; % height in z direction
g = 9.81;  % in m/sec2
roW = 1.1*10^3; % in kg/m3
MaxThrusterTorque = [40; 40]; % in per unit, MaxTorque that can be applied
...by thrusters Rows: LeftThruster, RightThruster
DisCMLThruster = [-0.431; -0.055; 0.043]; % in meter, Columns: X, Y, Z
DisCMRThruster = [-0.431;  0.055; 0.043]; % in meter, Columns: X, Y, Z
DistanceCMCOB = [0; 0; 0]'; % in meter, Columns: X, Y, Z
...Representation of CM in Body Fixed Frame
    
WindVelocity = [0; 0];

%For rectangular object l/d==1, Re>10^4
CDA = 1.2466;  % Drag Coeff of Air
CDW = 1.18;    % Drag Coeff of Water
% -------------------------------------------------------------------------

% Decide wheter state reduction in state or not
Reduced = false;

%Define Motion Config
MotionConfig.EnableCCForces = true;
MotionConfig.EnableRestoringForces = true;
MotionConfig.EnableDampingForces = true;
MotionConfig.EnableAirDragForces = true;
MotionConfig.EnableThrusterForces = true;
MotionConfig.ThrusterForces = [0; 0];

% Derive some constant parameters
DeriveParameters();


%% Initial State Variables 
%  which are initially all state variables taken zero
InitStates = zeros(6,1);

% Previous States are used for loop purpose
InitStates(1) = 0;
InitStates(2) = 0;
InitStates(3) = 0;
InitStates(4) = 1;
CurrentStates = InitStates;

%% Configuration for Mixed-Integer Quadratic Programming
InitialStates_PP = [InitStates(1) InitStates(2)]';

% Sampling Time for Path Planning .
Ts_PP = 0.5; 

% Set Parameters for constraints

%[ObstaclePosition_X, ObstaclePosition_Y, LengthOfObstacle, WidthOfObstacle]
Array_Of_Obstacles = [ 10, 0, 4, 1, 0, 0, 0, 0
                       %14.5, -0.5, 5, 1, 0, 0, 0, 0;
                       %19, 0.5, 4, 1, 0, 0, 0, 0
                       ]; 

Number_Of_Obstacles = length( Array_Of_Obstacles(:,1) );    


SafetyMarginInTime = 2; %[s]
% -------------------------------------------------------------------------

[A_PP, B_PP, C_PP] = StateSpace_PP(Ts_PP);

% Define sizes of state, control and outputs
NumberOfStates = size(A_PP, 1); % n = number of lines of the matrix Ad
NumberOfControls = size(B_PP, 2); % p = number of lines of the matrix Bd
NumberOfOutputs = size(C_PP, 1); % q = number of lines of the matrix Cd
% -------------------------------------------------------------------------

% Determine Constraints for MIQP
OutputConstraints = [-inf inf;  % for position_X output
                     -inf inf]; % for position_Y output 

ControlIncrementConstraints = 0.5*[-0.1 0.1;  % delta_vx_min and delta_vx_max
                               -0.1 0.1]; % delta_vy_min and delta_vy_max                 

ControlConstraints = [ -1.0 1.0;
                       -1.0 1.0]; % constraints on U.                         

NumberOfDecisions = 4 * Number_Of_Obstacles; % number of binary constraints.

% Define MIQP Parametes (before integer constraints)
Horizon_Vector_PP = [15 5];
Prediction_Horizon_PP = Horizon_Vector_PP(1);
Control_Horizon_PP = Horizon_Vector_PP(2);
% Weight Term terms for Traking (Q_PP) and Control Effort (R_PP)
Q_PP = diag([1; 1]);
R_PP = 0*diag([1e2; 1e2]);
% -------------------------------------------------------------------------

% Sampling Time for Plant 
TsPlant = 0.1;
% Start Time of The Simulation  
SimTime = 0;
% Time Limit for The Simulation
SimTimeLimit = 20;

% profile on;
% tic;
%% Start MIQP for Path Generation and Model Predictive Algorithm
finished = false;

States_PP = InitialStates_PP;
ControlInput_PP = [ cos( CurrentStates(3) ) -sin( CurrentStates(3) );
                   sin( CurrentStates(3) ) cos( CurrentStates(3) ) ] * ...
                  [ CurrentStates(4); CurrentStates(5) ];
DecisionInput_PP = ones( 4 * Number_Of_Obstacles, 1 );
Output_PP = C_PP*InitialStates_PP;

StatesLog_PP = [SimTime States_PP' ControlInput_PP' DecisionInput_PP'];

PrevStates = CurrentStates;

ReferenceOutput_PP = zeros(Prediction_Horizon_PP, 2);

x_y_final = [18; 0];
CoA = 0.5; % Cicle of acceptance to finalize the motion

ops = sdpsettings('solver','cplex','verbose',0,'debug',0);

%%
while ( ~finished && SimTime < SimTimeLimit )
%while ( ~finished )    
    SimTime

    % Construct Yalmip problem for MIQP - Path Planner
    yalmip('clear');
    du_PP = sdpvar(NumberOfControls, Control_Horizon_PP, 'full');
    binary_PP = binvar(NumberOfDecisions, Prediction_Horizon_PP, 'full');
    
    Constraints_PP = [];
    objective_PP = 0;
    
    PredictedState_PP = States_PP;
    
    for k = 1:Prediction_Horizon_PP
        
        %ReferenceOutput_PP(k,1) = States_PP(1) + Ts_PP * k; 
        ReferenceOutput_PP(k,1) = x_y_final(1);
        %ReferenceOutput_PP(k,2) = x_y_final(2);
        
    end
    
    u_PP = ControlInput_PP;
     
    for k = 1:Number_Of_Obstacles
        % X_min
        Array_Of_Obstacles(k, 5) = Array_Of_Obstacles(k, 1) - ...
                                    ( Array_Of_Obstacles(k, 3)/2 + wi);
                                    
        if u_PP(1)>0
            Array_Of_Obstacles(k, 5) = Array_Of_Obstacles(k, 5) - ... 
                                         u_PP(1) * SafetyMarginInTime;
        end
        
        % X_max
        Array_Of_Obstacles(k, 6) = Array_Of_Obstacles(k, 1) + ...
                                    ( Array_Of_Obstacles(k, 3)/2 + wi);
                                    
        % Y_min
        Array_Of_Obstacles(k, 7) = Array_Of_Obstacles(k, 2) - ...
                                    ( Array_Of_Obstacles(k, 4)/2 + wi);
                                
        % Y_max
        Array_Of_Obstacles(k, 8) = Array_Of_Obstacles(k, 2) + ...
                                    ( Array_Of_Obstacles(k, 4)/2 + wi);                        
    end
    
    
    for k = 1:Control_Horizon_PP
        % Calculate u
        u_PP = u_PP + du_PP(:,k);
        
        % Provide formula for u{k}: state update
        PredictedState_PP = A_PP * PredictedState_PP + B_PP * u_PP;
        
        % Calculate objective
        objective_PP = objective_PP + ...
            (PredictedState_PP - ReferenceOutput_PP(k,:)' )' * Q_PP * ...
                (PredictedState_PP - ReferenceOutput_PP(k,:)' ) + ...
                du_PP(:,k)' * R_PP * du_PP(:,k);
        
        % Add Constraints on Control Increment
        Constraints_PP = [Constraints_PP, ControlIncrementConstraints(:,1) <= ...
                            du_PP(:,k) <= ControlIncrementConstraints(:,2)];
        
        % Add constraints on Control Input
        Constraints_PP = [Constraints_PP, ControlConstraints(:,1) <= ...
                            u_PP <= ControlConstraints(:,2)];
        
        for j = 1: Number_Of_Obstacles                
            
            % Add conststaints for collision avoidance
        Constraints_PP = [Constraints_PP, implies( Array_Of_Obstacles(j,5) <= ...
                          PredictedState_PP(1), binary_PP( (j-1)*4 + 1,k) )];
        
        Constraints_PP = [Constraints_PP, implies( PredictedState_PP(1) <= ...
                            Array_Of_Obstacles(j,6), binary_PP( (j-1)*4 + 2,k) )];
        
        Constraints_PP = [Constraints_PP, implies( Array_Of_Obstacles(j,7) <= ...
                            PredictedState_PP(2), binary_PP( (j-1)*4 + 3,k) )];
        
        Constraints_PP = [Constraints_PP, implies( PredictedState_PP(2) <= ...
                            Array_Of_Obstacles(j,8), binary_PP( (j-1)*4 + 4,k) )];
                        
        % Add constraints on binary variables
        Constraints_PP = [Constraints_PP, ...
            sum( binary_PP( ( (j-1)*4 + 1) : ( (j-1)*4 + 4),k))<=3];
        
        end             
                        
        % Update ReferenceOuput
        % ReferenceOutput_PP = x_y_final; %ReferenceOutput_PP + Ts_PP * [1.5; 0];
    end
    
    for k = Control_Horizon_PP + 1:Prediction_Horizon_PP
        % Provide formula for u{k}: state update
        PredictedState_PP = A_PP *PredictedState_PP + B_PP*u_PP;
        
        % Calculate objective
        objective_PP = objective_PP + ...
            (PredictedState_PP - ReferenceOutput_PP(k,:)' )' * Q_PP * ...
                (PredictedState_PP - ReferenceOutput_PP(k,:)' );
                        
        % Add constraints on OutputState
        Constraints_PP = [Constraints_PP, OutputConstraints(:,1) <= ...
                            PredictedState_PP <= OutputConstraints(:,2)];
        
       for j = 1: Number_Of_Obstacles                
            
            % Add conststaints for collision avoidance
        Constraints_PP = [Constraints_PP, implies( Array_Of_Obstacles(j,5) <= ...
                          PredictedState_PP(1), binary_PP( (j-1)*4 + 1,k) )];
        
        Constraints_PP = [Constraints_PP, implies( PredictedState_PP(1) <= ...
                            Array_Of_Obstacles(j,6), binary_PP( (j-1)*4 + 2,k) )];
        
        Constraints_PP = [Constraints_PP, implies( Array_Of_Obstacles(j,7) <= ...
                            PredictedState_PP(2), binary_PP( (j-1)*4 + 3,k) )];
        
        Constraints_PP = [Constraints_PP, implies( PredictedState_PP(2) <= ...
                            Array_Of_Obstacles(j,8), binary_PP( (j-1)*4 + 4,k) )];
                        
        % Add constraints on binary variables
        Constraints_PP = [Constraints_PP, ...
            sum( binary_PP( ( (j-1)*4 + 1) : ( (j-1)*4 + 4),k))<=3];
        
        end                 
                        
        % Update ReferenceOuput
        %ReferenceOutput_PP = x_y_final; %ReferenceOutput_PP + Ts_PP * [1.5; 0];
    end
    
    OPT = optimize( Constraints_PP, objective_PP, ops );
    ControlIncrement_PP = value(du_PP(:,1));
    
    % Calculate ControlInput and DecisionInput
    ControlInput_PP = ControlInput_PP + ControlIncrement_PP; 
    DecisionInput_PP = value(binary_PP(:,1));
    
    % Calculation of the References for Low Level MPC Controller
    PredictedState_x_y = A_PP * States_PP + B_PP * ControlInput_PP; 
    
    States_PP = [ PredictedState_x_y(1); PredictedState_x_y(2)];
    

    StatesLog_PP = [StatesLog_PP; SimTime States_PP' ...
                    ControlInput_PP' DecisionInput_PP'];
    
     
    SimTime = SimTime + TsPlant;
end

dummylabel=strcat('MIQP_and_MPC','.mat');
save(dummylabel, 'StatesLog_PP', ...
     'Array_Of_Obstacles', 'Number_Of_Obstacles');  

Plotter_
