function [Fg] = RestoringForces(Enable)
% This function generates restoring forces for a given set of states

global m l wi g roW GMT GML

syms States x y z phi theta psiy u v w p q r real
States = [x y z phi theta psiy u v w p q r]';

if (Enable)

% ro is density of the fluid, g is acceleration   
% of gravity downwards, Awp is the area of    
% vehicle bottom : z0=ro*g*Awp                      

Awp = l*wi;
z0 = roW*g*Awp;

Fg = [0; 0; -z0*z; -m*g*GMT*phi; -m*g*GML*theta; 0];

else
    
Fg = zeros(6,1);

end

end

