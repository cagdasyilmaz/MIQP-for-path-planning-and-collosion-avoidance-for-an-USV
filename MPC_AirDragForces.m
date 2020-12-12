function Fad = MPC_AirDragForces(States,Enable)
% This function calculates the air drag forces

%Declare global variables
global WindVelocity CDA l wi h d

if (Enable)
    
u = States(4); %Surge Lin. Vel.
v = States(5); %Sway Lin. Vel.

h0 = h-d; %Height subject to air

% Forces calculated based on relative velocities
BodyLinearVelocity = [u; v];
RelativeVelocity = BodyLinearVelocity-WindVelocity;

ur = RelativeVelocity(1);
vr = RelativeVelocity(2);

%Area subject to air is utilized
Fax = h0*wi*2.56*abs(ur)*CDA;
Fay = h0*l*2.56*abs(vr)*CDA;

%It is assumed that air moments are negilicible due to symmetry
Fad = -[Fax; Fay; 0];

else
    
Fad = zeros(3,1);

end

end