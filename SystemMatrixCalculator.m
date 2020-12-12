function [A,B,C,D] = SystemMatrixCalculator(MotionConfig)
% Calculates state matrices by using linearization method

global Jacobian_StatesDot_States Jacobian_StatesDot_Inputs States Inputs ...
       States_Dot 

States_Value = MotionConfig.LinearizationPoint(1:6,:)';   
   

   
Aprime = subs(Jacobian_StatesDot_States,{States(1) States(2) States(3) ...
               States(4) States(5) States(6)  Inputs(1) Inputs(2)}, ...
               double([States_Value ...
               MotionConfig.ThrusterForces']));

Bprime = subs(Jacobian_StatesDot_Inputs,{States(1) States(2) States(3) ...
               States(4) States(5) States(6)  Inputs(1) Inputs(2)}, ...
               double([States_Value ...
               MotionConfig.ThrusterForces']));

StatesDotInit = subs(States_Dot,{States(1) States(2) States(3) ...
               States(4) States(5) States(6) Inputs(1) Inputs(2)}, ...
               double([States_Value ...
               MotionConfig.ThrusterForces']));

A = [Aprime StatesDotInit; zeros(1,6) 0];

B = [Bprime; zeros(1,2)];

% C = [zeros(1,2) 1 zeros(1,4); zeros(1,3) 1 zeros(1,3)];

C = [zeros(1,3) 1 zeros(1,3);
     zeros(1,2) 1 zeros(1,4)]; % surge speed and yaw angle

D = zeros(2,2);

A = double(A); B = double(B); C = double(C); D = double(D);
end