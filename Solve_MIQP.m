function [PredictedState, DecisionInput_PP, States_PP, ControlInput_PP] = ...
                            Solve_MIQP(NumberOfControls,...
                            Control_Horizon_PP, Prediction_Horizon_PP, ...
                            Q_PP, R_PP, States_PP, ControlInput_PP, ...
                            A_PP, B_PP, C_PP, ReferenceOutput_PP, ...
                            NumberOfDecisions, Number_Of_Obstacles, ...
                            SafetyMarginInTime,ControlIncrementConstraints,...
                            ControlConstraints, OutputConstraints, ...
                            Array_Of_Obstacles, ops)
     
global wi

% Construct Yalmip problem for MIQP - Path Planner
yalmip('clear');
du_PP = sdpvar(NumberOfControls, Control_Horizon_PP, 'full');
binary_PP = binvar(NumberOfDecisions, Prediction_Horizon_PP, 'full');
    
Constraints_PP = [];
objective_PP = 0;
    
PredictedState_PP = States_PP;
    
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
                               (Array_Of_Obstacles(k, 3)/2 + wi);
                                    
    % Y_min
    Array_Of_Obstacles(k, 7) = Array_Of_Obstacles(k, 2) - ...
                               (Array_Of_Obstacles(k, 4)/2 + wi);
                                
    % Y_max
    Array_Of_Obstacles(k, 8) = Array_Of_Obstacles(k, 2) + ...
                               (Array_Of_Obstacles(k, 4)/2 + wi);                        
end
    
for k = 1:Control_Horizon_PP
    % Calculate u
    u_PP = u_PP + du_PP(:,k);
        
    % Provide formula for u{k}: state update
    PredictedState_PP = A_PP * PredictedState_PP + B_PP * u_PP;
    PredictedOutput_PP = C_PP * PredictedState_PP;    
    
    % Calculate objective
    objective_PP = objective_PP + ...
        (PredictedOutput_PP - ReferenceOutput_PP(k,:)' )' * Q_PP * ...
        (PredictedOutput_PP - ReferenceOutput_PP(k,:)' ) + ...
        du_PP(:,k)' * R_PP * du_PP(:,k);
    
    % Add constraints on OutputState
    Constraints_PP = [Constraints_PP, OutputConstraints(:,1) <= ...
        PredictedState_PP <= OutputConstraints(:,2)];
    
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
                            PredictedState_PP(3), binary_PP( (j-1)*4 + 3,k) )];
        
        Constraints_PP = [Constraints_PP, implies( PredictedState_PP(3) <= ...
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
    PredictedState_PP = A_PP * PredictedState_PP + B_PP*u_PP;
    PredictedOutput_PP = C_PP * PredictedState_PP;
    % Calculate objective
    objective_PP = objective_PP + ...
        (PredictedOutput_PP - ReferenceOutput_PP(k,:)' )' * Q_PP * ...
        (PredictedOutput_PP - ReferenceOutput_PP(k,:)' );
                        
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
                            PredictedState_PP(3), binary_PP( (j-1)*4 + 3,k) )];
        
        Constraints_PP = [Constraints_PP, implies( PredictedState_PP(3) <= ...
                            Array_Of_Obstacles(j,8), binary_PP( (j-1)*4 + 4,k) )];
                        
        % Add constraints on binary variables
        Constraints_PP = [Constraints_PP, ...
            sum( binary_PP( ( (j-1)*4 + 1) : ( (j-1)*4 + 4),k))<=3];
        
    end                 
                        
    % Update ReferenceOuput
    %ReferenceOutput_PP = x_y_final; %ReferenceOutput_PP + Ts_PP * [1.5; 0];
end
    
OPT = optimize(Constraints_PP ,objective_PP, ops); %, sdpsettings('verbose', 0));
ControlIncrement_PP = value(du_PP(:,1));
    
% Calculate ControlInput and DecisionInput
ControlInput_PP = ControlInput_PP + ControlIncrement_PP; 
DecisionInput_PP = value(binary_PP(:,1));
    
% Calculation of the References for Low Level MPC Controller
PredictedState = A_PP * States_PP + B_PP * ControlInput_PP; 
    
States_PP = PredictedState;
        
% for k = 2:Control_Horizon_PP  
% 
%     ControlIncrement_PP = value(du_PP(:,k));
%     ControlInput_PP = ControlInput_PP + ControlIncrement_PP;
%         
%     PredictedState_x_y = A_PP * PredictedState_x_y + B_PP * ControlInput_PP;  
% 
% end
%     
% for k = Control_Horizon_PP + 1:Prediction_Horizon_PP
%         
%     PredictedState_x_y = A_PP * PredictedState_x_y + B_PP * ControlInput_PP; 
%         
% end

end