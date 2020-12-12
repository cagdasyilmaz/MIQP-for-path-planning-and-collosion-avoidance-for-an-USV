function DeriveParameters()
% This function derives some parameters to be utilized model

%Declare global variables
global m  l wi h roW DistanceCMCOB Ix Iy Iz Ixy Iyz Ixz d...
     GMT GML Xvdot Xwdot Xpdot Xqdot Xrdot Yudot Ywdot Ypdot Yqdot Yrdot...
     Zudot Zvdot Zpdot Zqdot Zrdot Kudot Kvdot Kwdot Kqdot Krdot...
     Mudot Mvdot Mwdot Mpdot Mrdot Nudot Nvdot Nwdot Npdot Nqdot...
     Xudot Yvdot Zwdot Kpdot Mqdot Nrdot xG yG zG I0 Mrb Ma M MInv
 
% If ellipsoid assumption is used
a = l/2; % semi major axis of the elipsoid
b = wi/2; % semi minor axis of the elipsoid
c = h/2; % semi vertical axis of the elipsoid

% Moment of Inertia Terms
Ix = 0.0894; % -2.5;    % m/12*(wi^2+h^2);
Iy = 2.1442;       % m/12*(l^2+h^2);
Iz = 0.6487; % 0.0894;  % m/12*(wi^2+l^2);
Ixy = 0;      % 0;
Iyz = 0;      % 0.0894; % 0;
Ixz = 0.02280; % 2.1493; % 0;

% Moment Arm Calculation For Buoyancy and Gravitational Forces
% Approximations for GMT and GML
% Assumption: Rectangular water Plane (Awp), Box Shape Assumption
IT = wi^3*l/12;
IL = l^3*wi/12;

%Draft calculation
d = m/(roW*l*wi); % in meter, the height in water
DV = l*wi*d; % volume of ship under water
BMT = IT/DV; % in meter
BML = IL/DV; % in meter
BG = h/2-d/2; % in meter, distance between CG and BC (buoyancy center)

%Moment arms
GMT = BMT-BG;
GML = BML-BG;

% Added Mass, Coriolis Terms
% Assumption: The vehicle has 3 plane symmetry and neglect off-diagonal terms in Ma
% Ellipsoid assumption

Xvdot=0; Xwdot=0; Xpdot=0; Xqdot=0; Xrdot=0;
Yudot=0; Ywdot=0; Ypdot=0; Yqdot=0; Yrdot=0;
Zudot=0; Zvdot=0; Zpdot=0; Zqdot=0; Zrdot=0;
Kudot=0; Kvdot=0; Kwdot=0; Kqdot=0; Krdot=0;
Mudot=0; Mvdot=0; Mwdot=0; Mpdot=0; Mrdot=0;
Nudot=0; Nvdot=0; Nwdot=0; Npdot=0; Nqdot=0;

e = sqrt(1-(b/a)^2);
alphaZero = 2*(1-e^2)/e^3*(1/2*log((1+e)/(1-e))-e);
betaZero= 1/e^2-(1-e^2)/(2*e^3)*log((1+e)/(1-e));
 
Xudot = -1.0946; %-1.0946; %-alphaZero/(2-alphaZero)*(4/3)*pi*roW*a*b^2;
Yvdot = -1.0536; %-1.0536; %-betaZero/(2-betaZero)*(4/3)*pi*roW*a*b^2;
Yrdot = -1.4353; %-1.4353;
Nvdot = Yrdot;
Zwdot = 0;
Kpdot = 0;
Mqdot = 0; % -(1/5)*(b^2-a^2)^2*(alphaZero-betaZero)/(2*(b^2-a^2)+(b^2+a^2)*...
           %(betaZero-alphaZero))*(4/3)*pi*roW*a*(b^2);
% Nrdot = Mqdot;
Nrdot = -0.0474;

% Distances between CM and COB 
xG = DistanceCMCOB(1);
yG = DistanceCMCOB(2);
zG = DistanceCMCOB(3);

%Cross product operator
%Src = [0 -zG yG; zG 0 -xG; -yG xG 0];

% Inertia Matrix
I0 = Iz; 

% Rigid body inertia matrix 
Mrb = [ m*eye(2)     zeros(2,1);                      
        zeros(1,2)  I0 ];

%Added mass inertia matrix
Ma = -[Xudot Xvdot  Xrdot;
       Yudot Yvdot  Yrdot;
       Nudot Nvdot  Nrdot];

M = Mrb+Ma;                               
MInv = inv(M); 

end