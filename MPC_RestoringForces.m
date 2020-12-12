function [Fg] = RestoringForces(States,Enable)
% This function generates restoring forces for a given set of states

global m l wi g roW GMT GML

if (Enable)

% ro is density of the fluid, g is acceleration   
% of gravity downwards, Awp is the area of    
% vehicle bottom : z0=ro*g*Awp

z = States(3);            % z position in Earth-fixed orientation
phi = States(4);          % phi position in Earth-fixed orientation
theta = States(5);        % theta in Earth-fixed orientation

Awp = l*wi;
z0 = roW*g*Awp;

Fg = [0; 0; -z0*z; -m*g*GMT*phi; -m*g*GML*theta; 0];

else
    
Fg = zeros(6,1);

end

end

