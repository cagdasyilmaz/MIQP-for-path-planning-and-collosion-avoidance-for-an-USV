function [Fcc] = MPC_CCForces(States,Enable)
% This function generates coriolis and centripental forces

global m Yrdot Xudot Yvdot Nvdot xG yG 

if (Enable)
    
u = States(4);      % Surge Speed in Body-fixed frame
v = States(5);      % Sway Speed in Body-fixed frame
r = States(6);     % Yaw Speed in Body-fixed frame
 
% Rigid Body Coriolis and Centripental Forces Matrix
Crb = [0 0 -m*(xG*r+v);
       0 0 -m*(yG*r-u);
       m*(xG*r+v) m*(yG*r-u) 0];

% Added Mass Coriolis and Centripetal Forces Matrix  
Ca = -[0 0  -Yvdot*v-(Yrdot+Nvdot)/2*r;
      0 0  Xudot*u;
      Yvdot*v+(Yrdot+Nvdot)/2*r -Xudot*u  0];

Ct = Ca+Crb;

Fcc = Ct*States(4:end);

else
    
Fcc = zeros(3,1);

end

end

