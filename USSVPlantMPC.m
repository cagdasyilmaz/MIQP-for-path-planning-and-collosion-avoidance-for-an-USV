% Initially clear workspace, clear all variable, close all windows
clc;
clear all;
close all;

% Declare global variables ------------------------------------------------
global m  l wi h g roW  MaxThrusterTorque DisCMLThruster...
       DisCMRThruster DistanceCMCOB WindVelocity CDW CDA States Inputs ...
       States_Dot 
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

% Take StatesDot Equation In The Symbolic Form
States_Dot = vpa(StatesDot(MotionConfig),4);

global Jacobian_StatesDot_States Jacobian_StatesDot_Inputs

Jacobian_StatesDot_States = jacobian(States_Dot,States);
Jacobian_StatesDot_Inputs = jacobian(States_Dot,Inputs);

%% Initial State and Input Variables 
%  which are initially all state variables taken zero
InitStates = zeros(6,1);

% Previous States are used for loop purpose -------------------------------
InitStates(1) = 0;
InitStates(2) = 0;
InitStates(3) = 0;
InitStates(4) = 1;
CurrentStates = InitStates;
% -------------------------------------------------------------------------

% Initial Left and Right Thruster Forces ----------------------------------
LThrusterForceX = 0;
RThrusterForceX = 0;
% -------------------------------------------------------------------------

%% Sampling Time for Path Planning & Sampling Time For Lower Controller
   % and Plant and Simulation Times

% Sampling Time for Path Planning -----------------------------------------
Ts_PP = 0.5;

% Sampling Time for MPC ---------------------------------------------------
Ts_MPC = 0.1;

% Sampling Time for Plant -------------------------------------------------
TsPlant = 0.1;

% Start Time of The Simulation --------------------------------------------
SimTime = 0;

% Time Limit for The Simulation
SimTimeLimit = 30;

%% Configuration for Mixed-Integer Quadratic Programming
InitialStates_PP = [InitStates(1) InitStates(4) InitStates(2) InitStates(5)]';

% Set Parameters for constraints

%[ObstaclePosition_X, ObstaclePosition_Y, LengthOfObstacle, WidthOfObstacle]
Array_Of_Obstacles = [ 10, 0, 4, 1, 0, 0, 0, 0;
                       14.5, -1, 5, 1, 0, 0, 0, 0;
                       19, 0, 4, 1, 0, 0, 0, 0
                     ]; 

Number_Of_Obstacles = length( Array_Of_Obstacles(:,1) );    

SafetyMarginInTime = 2; %[sec]
TangentQuadrant = 1; 
% -------------------------------------------------------------------------

[A_PP, B_PP, C_PP] = StateSpace_PP(Ts_PP);

% Define sizes of state, control and outputs
NumberOfStates = size(A_PP, 1); % n = number of lines of the matrix Ad
NumberOfControls = size(B_PP, 2); % p = number of lines of the matrix Bd
NumberOfOutputs = size(C_PP, 1); % q = number of lines of the matrix Cd
% -------------------------------------------------------------------------

% Determine Constraints for MIQP
OutputConstraints = [-inf inf;  % for position_X output
                     -1 1;      % for velocity_X output
                     -inf inf;  % for position_Y output 
                     -1 1];     % for velocity_Y output

ControlIncrementConstraints = 0.1*[-0.5 0.5; % delta_vx_min and delta_vx_max
                               -0.5 0.5];% delta_vy_min and delta_vy_max                 

ControlConstraints = 0.2*[-1.0 1.0;
                          -1.0 1.0]; % constraints on U.                         

NumberOfDecisions = 4 * Number_Of_Obstacles; % number of binary constraints.

% Define MIQP Parametes (before integer constraints)
% Weight Term terms for Traking (Q_PP) and Control Effort (R_PP)
Horizon_Vector_PP = [40 5];
Prediction_Horizon_PP = Horizon_Vector_PP(1);
Control_Horizon_PP = Horizon_Vector_PP(2);
Q_PP = 1e4*diag([1; 1]);
R_PP = 1e-1*diag([1; 1]);
% -------------------------------------------------------------------------

%% Configuration for Model Predictive Control

Horizon_Vector = [20 5];
Prediction_Horizon = Horizon_Vector(1);
Control_Horizon = Horizon_Vector(2);

Surge_Weight_Outer_Park = 1*1e2;              
Yaw_Weight_Outer_Park = 5*1e1;              

Surge_Weight_Inner_Park = 1*1e2;            
Yaw_Weight_Inner_Park = 5*1e1;  

Num_Of_Output = 2;
Num_Of_Input = 2;

Future_Predicted_Surge = zeros( Prediction_Horizon, 1);
Future_Predicted_Yaw = zeros( Prediction_Horizon, 1);

% Define Weight Matrix
% Q1: Outside of the parking region (Weight on surge speed higher)
% Q2: In the neighboorhod of the parking region (Weight on yaw angle higher)
Q1 = zeros( Num_Of_Output * Prediction_Horizon,...
            Num_Of_Output * Prediction_Horizon);
Q2 = zeros( Num_Of_Output * Prediction_Horizon,...
            Num_Of_Output * Prediction_Horizon);
           
% Construction of the diagonal Q1 and Q2 matricies
for i = 1:1:Num_Of_Output*Prediction_Horizon
    if rem(i,2) == 1
            Q1(i,i) = Surge_Weight_Outer_Park;
            Q2(i,i) = Surge_Weight_Inner_Park;
    elseif rem(i,2) == 0  
            Q1(i,i) = Yaw_Weight_Outer_Park;
            Q2(i,i) = Yaw_Weight_Inner_Park;
    end
end

Input_Weight = 1e-4;
R = Input_Weight*eye(Control_Horizon*Num_Of_Input); 

% Maximum and Minimum Applied Torques
U_Max =  40;
U_Min = -40;

% Determine Constraints for MIQP
OutputConstraints_MPC = [-2 2     % for surge speed
                         -pi pi]; % for yaw angle

ControlIncrementConstraints_MPC = [U_Min*2 U_Max*2; % delta_LThrust_min and delta_LThrust_max
                                   U_Min*2 U_Max*2];% delta_RThrust_min and delta_RThrust_max                 

ControlConstraints_MPC = [U_Min U_Max;
                          U_Min U_Max]; % constraints on Thrusters.                      
                      
Future_Predicted_Output = zeros(2*Prediction_Horizon,1);
%% Initialize Log Arrays
% -------------------------------------------------------------------------
StatesLog = [];

LThrusterForceXLog = [];
RThrusterForceXLog = [];

Desired_Outputs = [];

Tot_F = zeros(1,3);
Tot_Forces = [];

Dist_F = zeros(1,3);
Dist_Forces = [];

Energy_Vector = 0;
Total_Energy_Consumption = []; 
% -------------------------------------------------------------------------

%% Low Pass Filter Configuration
    
% Yaw Angle ---------------------------------------------------------------
% FILTER_CONSTANT_YAW = 0.05;
filtered_yaw_previous = InitStates(3);
% yaw_filter_coefficient = FILTER_CONSTANT_YAW / ...
%                                 ( FILTER_CONSTANT_YAW + Ts_PP)
yaw_filter_coefficient = 0.2;

% Surge Speed -------------------------------------------------------------
% FILTER_CONSTANT_SURGE = 0.05;
filtered_surge_previous = InitStates(4);
% surge_filter_coefficient = FILTER_CONSTANT_SURGE / ...
%                                 ( FILTER_CONSTANT_SURGE + Ts_PP)
surge_filter_coefficient = 0.25;

%% Start MIQP for Path Generation and Model Predictive Algorithm

States_PP = InitialStates_PP;
ControlInput_PP = [0; 0];
DecisionInput_PP = ones( 4 * Number_Of_Obstacles, 1 );
Output_PP = C_PP*InitialStates_PP;

PrevStates = CurrentStates;

ReferenceOutput_PP = zeros(Prediction_Horizon_PP, 2);
        
%%  Write Logs With Initial Values at Time = 0

% -------------------------------------------------------------------------
StatesLog_PP = [SimTime States_PP' ControlInput_PP' DecisionInput_PP' ...
                TangentQuadrant ];
LThrusterForceXLog = [LThrusterForceXLog; SimTime LThrusterForceX];
RThrusterForceXLog = [RThrusterForceXLog; SimTime RThrusterForceX];
StatesLog = [StatesLog; SimTime CurrentStates'];
Desired_Outputs = [Desired_Outputs; SimTime Future_Predicted_Output'];
Tot_Forces = [Tot_Forces; SimTime Tot_F];
Dist_Forces = [Dist_Forces; SimTime Dist_F];
% -------------------------------------------------------------------------

%% Mixed-Integer Programming - YALMIP Configuration 

%ops = sdpsettings('solver', 'mosek', 'verbose', 0, 'debug', 0);
ops = sdpsettings('solver', 'cplex', 'verbose', 0, 'debug', 0);
%ops = sdpsettings('solver', 'gurobi', 'verbose', 0, 'debug', 0);

x_y_final = [19.5; 1.0];
CoA = 0.25; % Cicle of acceptance to finalize the motion

finished = false;
while ( ~finished && SimTime < SimTimeLimit )
%while ( ~finished )    
    
    SimTime
    
    vehicle_velocity = sqrt(States_PP(2)^2 + States_PP(4)^2);
    angle_PP = atan2((x_y_final(2) - States_PP(3)), ...
                    (x_y_final(1) - States_PP(1)));
    distance_between_references = vehicle_velocity * Ts_PP;
   
    distance_between_references_x = distance_between_references * cos(angle_PP);
    distance_between_references_y = distance_between_references * sin(angle_PP);
    
    for k = 1:Prediction_Horizon_PP
        
         ReferenceOutput_PP(k,1) = States_PP(1) + k * distance_between_references_x;
         ReferenceOutput_PP(k,2) = States_PP(3) + k * distance_between_references_y;
        
    end
%% Solution of Mixed Integer Quadratic Programming For Path Planning
% -------------------------------------------------------------------------

    [PredictedState, DecisionInput_PP, States_PP, ControlInput_PP] = ...
                            Solve_MIQP(NumberOfControls,...
                            Control_Horizon_PP, Prediction_Horizon_PP, ...
                            Q_PP, R_PP, States_PP, ControlInput_PP, ...
                            A_PP, B_PP, C_PP, ReferenceOutput_PP, ...
                            NumberOfDecisions, Number_Of_Obstacles, ...
                            SafetyMarginInTime,ControlIncrementConstraints,...
                            ControlConstraints, OutputConstraints, ...
                            Array_Of_Obstacles, ops);
% -------------------------------------------------------------------------
                        
    delta_y = PredictedState(3) - CurrentStates(2);
    delta_x = PredictedState(1) - CurrentStates(1);
    
    Yaw_PP =  atan2( delta_y, delta_x );

    Surge_Sway_PP = [cos( Yaw_PP ) sin( Yaw_PP ); ...
                     -sin( Yaw_PP ) cos( Yaw_PP )] * ...
                     [PredictedState(2); PredictedState(4)];
    
    % Determine Quadrant for Atan2 Function
    % ---------------------------------------------------------------------               
    if( ( sign( delta_x ) == 1 || sign( delta_x ) == 0 ) && ...
            ( sign( delta_y ) == 1 || sign( delta_y ) == 0 )  )
        TangentQuadrant = 1;
    elseif( ( sign( delta_x ) == -1 ) && ...
            ( sign( delta_y ) == 1 || sign( delta_y ) == 0 )  )
        TangentQuadrant = 2;
    elseif( ( sign( delta_x ) == -1 || sign( delta_x ) == 0 ) && ...
            ( sign( delta_y ) == -1 )  )    
        TangentQuadrant = 3;
    else    
        TangentQuadrant = 4;
    end        
    % ---------------------------------------------------------------------
    
    % Write Logs for States_PP, Control_Input_PP and DecisionInput_PP
    % ---------------------------------------------------------------------
    StatesLog_PP = [StatesLog_PP; SimTime PredictedState' ...
                    ControlInput_PP' DecisionInput_PP' TangentQuadrant];
    %----------------------------------------------------------------------
                
    [filtered_yaw, filtered_surge] = LowPassFilter_For_References( ...
                                          filtered_yaw_previous, ...
                                          Yaw_PP, ...
                                          yaw_filter_coefficient, ...
                                          filtered_surge_previous, ...
                                          Surge_Sway_PP(1),...
                                          surge_filter_coefficient );
    
    filtered_yaw_previous = filtered_yaw;                                   
    filtered_surge_previous = filtered_surge;
    
    for k = 1:Prediction_Horizon
        
        Future_Predicted_Output(2*k-1) = filtered_surge; %Desired_Surge; 
        Future_Predicted_Output(2*k) = filtered_yaw; %Desired_Yaw
        
    end
    
%% Solution of Model Predictive Control 
% -------------------------------------------------------------------------
    ControlIncrement_MPC = Solve_MPC(CurrentStates, LThrusterForceX, ...
                                RThrusterForceX, NumberOfControls, ...
                                Control_Horizon, ...
                                Prediction_Horizon, ...
                                Ts_MPC, Q1, Input_Weight, ...
                                Future_Predicted_Output, ...
                                OutputConstraints_MPC, ...
                                ControlIncrementConstraints_MPC, ...
                                ControlConstraints_MPC, ops);
    
    LThrusterForceX = LThrusterForceX +  ControlIncrement_MPC(1);
    RThrusterForceX = RThrusterForceX +  ControlIncrement_MPC(2);
%     
    %LThrusterForceX
    %RThrusterForceX
% -------------------------------------------------------------------------    
    
%% Apply Thruster Forces which is found by using MPC to Plant 
% -------------------------------------------------------------------------
    [States_MPC, StatesInE, Tot_F, Dist_F] = VehicleMotion_MPC(...
                                               LThrusterForceX,...
                                               RThrusterForceX,...
                                               CurrentStates,...
                                               TsPlant,...
                                               SimTime);
% -------------------------------------------------------------------------
    
    PrevStates = CurrentStates;
    CurrentStates = States_MPC ;
%     CurrentStates = [States_PP(1); States_PP(3); 0; Surge_Sway_PP(1); Surge_Sway_PP(2); 0];
    States_PP = [CurrentStates(1); States_PP(2); CurrentStates(2); States_PP(4)];
    
    if( norm(x_y_final - [CurrentStates(1); CurrentStates(2)]) < CoA )
        finished = true;
    end
    
    if( CurrentStates(1) >= 20 )
        finished = true;
    end
    
    SimTime = SimTime + TsPlant;
%     SimTime = SimTime + Ts_PP;

    % Write Logs
    % --------------------------------------------------------------------
    LThrusterForceXLog = [LThrusterForceXLog; SimTime LThrusterForceX];
    RThrusterForceXLog = [RThrusterForceXLog; SimTime RThrusterForceX];
    StatesLog = [StatesLog; SimTime CurrentStates'];
    Desired_Outputs = [Desired_Outputs; SimTime Future_Predicted_Output'];
    Tot_Forces = [Tot_Forces; SimTime Tot_F];
    Dist_Forces = [Dist_Forces; SimTime Dist_F];
    % ---------------------------------------------------------------------
    
end

dummylabel=strcat('MIQP_and_MPC','.mat');
              
save(dummylabel, 'StatesLog','Desired_Outputs','Tot_Forces', 'Dist_Forces',...
     'LThrusterForceXLog','RThrusterForceXLog', 'StatesLog_PP', ...
     'Array_Of_Obstacles','Number_Of_Obstacles');  

Plotter
