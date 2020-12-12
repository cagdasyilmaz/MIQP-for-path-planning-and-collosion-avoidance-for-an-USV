function [Jn] = MPC_B2E(States)
%Tihs function produces the matrix for velocity transformation, from
...body to earth fixed frame
    
psiy = States(3);  % Earth-fixed orientation, euler angle

% Linear velocity transformation
J1 = [cos(psiy), -sin(psiy);
      sin(psiy), cos(psiy)];

% Angular velocity transformation
J2 = 1;

Jn = [J1, zeros(2,1);
      zeros(1,2), J2];
  
end

