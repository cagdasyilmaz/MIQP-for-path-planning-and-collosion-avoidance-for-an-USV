function ControlIncrement_MPC = Solve_MPC(CurrentStates, LThrusterForceX,...
                                RThrusterForceX, NumberOfControls, ...
                                Control_Horizon, ...
                                Prediction_Horizon, ...
                                Ts_MPC, Q1, Input_Weight, ...
                                Future_Predicted_Output, ...
                                OutputConstraints_MPC, ...
                                ControlIncrementConstraints_MPC, ...
                                ControlConstraints_MPC, ops)
                                                                                           
% ---------------------------------------------------------------------
   
% Current States
% MotionConfig.LinearizationPoint = InitStates;
MotionConfig.LinearizationPoint = CurrentStates;
MotionConfig.ThrusterForces = [LThrusterForceX;RThrusterForceX];
    
% Linearization to find State Stape Equation
[A,B,C,D] = SystemMatrixCalculator(MotionConfig);
 
% Convert Discrete State Space Model
% [A_Dis, B_Dis, C_Dis] = ContinousToDiscrete(A,B,C,D,TsPlant);
[A_Dis, B_Dis, C_Dis] = c2dm(A,B,C,D,Ts_MPC,'zoh');
%A_Dis = TsPlant * A + eye(7);
%B_Dis = TsPlant * B;
%C_Dis = C;
        
% Construct Yalmip problem for MIQP - Path Planner
yalmip('clear');
delta_Thrusts_MPC = sdpvar(NumberOfControls, Control_Horizon, 'full');
Constraints_MPC = [];
objective_MPC = 0;
    
Thruster_Vector = [LThrusterForceX; RThrusterForceX];
PredictedState_MPC = [CurrentStates; 0];
Constraints_MPC = [];

for k = 1:Control_Horizon
        
    % Calculate Thrust
    Thruster_Vector = Thruster_Vector + delta_Thrusts_MPC(:,k);
        
    % Provide formula for u{k}: state update
    PredictedState_MPC = A_Dis * PredictedState_MPC + B_Dis * Thruster_Vector;
        
    Future_Surge_Yaw = [Future_Predicted_Output(2*k-1); ...
                        Future_Predicted_Output(2*k)];
                        
    Surge_Yaw = [PredictedState_MPC(4); ...
                 PredictedState_MPC(3)];                
        
    % Calculate objective
    objective_MPC = objective_MPC + (Surge_Yaw - Future_Surge_Yaw)' *...
              [Q1(2*k-1,2*k-1) 0; 0 Q1(2*k,2*k)] * ...
              (Surge_Yaw - Future_Surge_Yaw) + ...
              delta_Thrusts_MPC(:,k)' * [Input_Weight 0; 0 Input_Weight]...
              * delta_Thrusts_MPC(:,k);
        
    % Add constraints on OutputState for MPC
    Constraints_MPC = [Constraints_MPC, OutputConstraints_MPC(:,1) <= ...
                            Surge_Yaw <= OutputConstraints_MPC(:,2)];    
            
    % Add Constraints on Control Increment
    Constraints_MPC = [Constraints_MPC, ControlIncrementConstraints_MPC(:,1) <= ...
           delta_Thrusts_MPC(:,k) <= ControlIncrementConstraints_MPC(:,2)];
        
    % Add constraints on Control Input
    Constraints_MPC = [Constraints_MPC, ControlConstraints_MPC(:,1) <= ...
                            Thruster_Vector <= ControlConstraints_MPC(:,2)];
        
end
    
for k = Control_Horizon:Prediction_Horizon
        
    % Provide formula for u{k}: state update
    PredictedState_MPC = A_Dis * PredictedState_MPC + B_Dis * Thruster_Vector;
        
    Future_Surge_Yaw = [Future_Predicted_Output(2*k-1); ...
                        Future_Predicted_Output(2*k)];
                        
    Surge_Yaw = [PredictedState_MPC(4); ...
                 PredictedState_MPC(3)];                
        
    % Calculate objective
    objective_MPC = objective_MPC + (Surge_Yaw - Future_Surge_Yaw)' *...
                [Q1(2*k-1,2*k-1) 0; 0 Q1(2*k,2*k)] * ...
                (Surge_Yaw - Future_Surge_Yaw);  
        
    % Add constraints on OutputState for MPC
    Constraints_MPC = [Constraints_MPC, OutputConstraints_MPC(:,1) <= ...
                            Surge_Yaw <= OutputConstraints_MPC(:,2)];
                        
end
    
OPT = optimize(Constraints_MPC, objective_MPC, ops); 
ControlIncrement_MPC = value(delta_Thrusts_MPC(:,1));

end