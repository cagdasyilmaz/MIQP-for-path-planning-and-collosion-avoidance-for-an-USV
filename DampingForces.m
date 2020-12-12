function Fwd = DampingForces(Enable)
% This function calculates the water damping forces

global CDW wi l d roW

syms States x y psiy u v r real
States = [x y psiy u v r]';

if (Enable)

% Extract Velocities
h0 = d; % Height Subject to Water

%Linear Damping Forces
Xu = -5.6128; %-5.6128; % 5;
Yv = -3.5059; %-3.5059; % 5;
Yr = -0.0001; % Yr;
Nv = Yr; % 5;
%Nv = %-1.0005; % Yr;
Nr = -0.1039; % 5;
%Zw = 150;
%Kp = 5;
%Mq = 5;

%Linear Damping Matrix
Dl = -[Xu 0  0;...
      0 Yv  Yr;...
      0 Nv  Nr];
 
%Nonlinear Damping Forces
%Cross-sectional areas
AUT = wi*h0;
AUL = h0*l;
AWP = l*wi;

Xuu = -2.3136; % 1/2*roW*CDW*AUT;
Yvv = -3.9310; % 1/2*roW*CDW*AUL;
%Zww = 1/2*roW*CDW*AWP;
%Kpp = 1/2*roW*CDW*((AUT+AUL)*2+AWP);
%Mqq = 1/2*roW*CDW*((AUT+AUL)*2+AWP);
Nrr = -0.2605; % 1/2*roW*CDW*((AUT+AUL)*2+AWP);

%Nonlinear damping matrix
Dn = -[ Xuu*abs(u) 0  0;...
        0 Yvv*abs(v)  0;...
        0 0  Nrr*abs(r)];

%Total damping matrix
D = -(Dl+Dn);

Fwd = D*States(4:end);

else
    
Fwd = zeros(3,1);

end

end