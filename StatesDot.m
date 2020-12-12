function StatesDot = StatesDot(MotionConfig)
%This function Calulates StatesDot

global MInv

syms States x y psiy u v r real
States = [x y psiy u v r]';

EnableThrusterForces = MotionConfig.EnableThrusterForces;
EnableCCForces = MotionConfig.EnableCCForces;
EnableDampingForces = MotionConfig.EnableDampingForces;
EnableAirDragForces = MotionConfig.EnableAirDragForces;

Ftt = ThrusterForces(EnableThrusterForces);

%Generate Coriolis and Centripetal Forces Matrix
Fcc = CCForces(EnableCCForces);
        
%Generate Damping Forces
Fwd = DampingForces(EnableDampingForces);
    
%Generate Air Drag Forces
Fad = AirDragForces(EnableAirDragForces);

viDot = MInv*(Ftt+Fwd+Fad-Fcc);

Jn = B2E();

nuDot = Jn*States(4:end);

%StatesDot = simple([nuDot; viDot]);
StatesDot = [nuDot; viDot];
end


