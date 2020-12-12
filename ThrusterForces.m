function Ftt = ThrusterForces(Enable)
%This function calculates the thruster forces

global DisCMLThruster DisCMRThruster

syms LThrusterForceX RThrusterForceX real

if (Enable)

LThrusterForce = [LThrusterForceX;0;0];
RThrusterForce = [RThrusterForceX;0;0];

% Generate Thuster Forces
LThrusterTorque = cross(DisCMLThruster,LThrusterForce); % Torque applied by left thruster around CM
RThrusterTorque = cross(DisCMRThruster,RThrusterForce); % Torque applied by right thruster around CM
        
LThrusterTotalForce = [LThrusterForce; LThrusterTorque]; % ThrusterForce= (Fx,Fy,Fz,Tx,Ty,Tz)'
RThrusterTotalForce = [RThrusterForce; RThrusterTorque]; % ThrusterForce= (Fx,Fy,Fz,Tx,Ty,Tz)'
  
Ftt = LThrusterTotalForce+RThrusterTotalForce;

Ftt = [Ftt(1,1); Ftt(2,1); Ftt(6,1)];

else
    
Ftt = zeros(3,1);

end

end