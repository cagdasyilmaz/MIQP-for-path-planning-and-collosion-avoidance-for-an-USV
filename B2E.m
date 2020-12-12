function [Jn] = B2E()
%Tihs function produces the matrix for velocity transformation, from
...body to earth fixed frame
    
syms States x y psiy u v r real
%States = [x y z phi theta psiy u v w p q r]';

% Linear velocity transformation
J1 = [cos(psiy), -sin(psiy);
      sin(psiy), cos(psiy)];

% Angular velocity transformation
J2 = 1;

Jn = [J1, zeros(2,1);
      zeros(1,2), J2];
  
end

