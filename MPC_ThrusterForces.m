function Ftt = MPC_ThrusterForces(LThrusterForceX,RThrusterForceX,Enable)
%This function calculates the thruster forces

global DisCMLThruster DisCMRThruster

if (Enable)

LThrusterForce = [LThrusterForceX;0;0];
RThrusterForce = [RThrusterForceX;0;0];

% Generate Thuster Forces
% Torque applied by left thruster around CM
LThrusterTorque = cross(DisCMLThruster,LThrusterForce); 
% Torque applied by right thruster around CM
RThrusterTorque = cross(DisCMRThruster,RThrusterForce); 

% ThrusterForce= (Fx,Fy,Fz,Tx,Ty,Tz)'
LThrusterTotalForce = [LThrusterForce; LThrusterTorque]; 
% ThrusterForce= (Fx,Fy,Fz,Tx,Ty,Tz)'
RThrusterTotalForce = [RThrusterForce; RThrusterTorque]; 
  
Ftt = LThrusterTotalForce + RThrusterTotalForce;

Ftt = [Ftt(1,1); Ftt(2,1); Ftt(6,1)];

else
    
Ftt = zeros(3,1);

end

end