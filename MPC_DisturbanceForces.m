function Fdis = MPC_DisturbanceForces(States,SimTime,Enable)

global roW g l wi d

if (Enable)

psiy = States(3); % Yaw position
u = States(4);     % Speed in x

h0 = d; % Height subject to water

% Dimensions of wetted part
L = l;
B = wi;
T = h0;

% Disturbance Parameters
w = [1 3 5 7 9];
%w = [1 3];
wN = length(w);
%dw = 2;
Hs = 0.06;
Ad = 8.1*(10^-3)*g^2;
Bd = 3.11/(Hs^2);
Beta = 5*pi/4;

% Encounter Angle
we = w-(w.^2)/g*u*cos(Beta-psiy);

Sw = Ad*(w.^-5).*exp(-Bd*(w.^-4));
%A = sqrt(2*Sw*dw);
A = Sw;
k = w.^2/g;

St = A.*k.*sin(we*SimTime);

Xwave = 0;
Ywave = 0;
Nwave = 0;

for i=1:wN
    
    Xwave = Xwave+roW*g*B*L*T*cos(Beta-psiy)*St(i);
    Ywave = Ywave+roW*g*B*L*T*sin(Beta-psiy)*St(i);
    Nwave = Nwave+(1/24)*roW*g*B*L*(L^2-B^2)*sin(2*(Beta-psiy))*St(i)^2;
    
end

Fdis = [Xwave; Ywave; Nwave];

else
    
Fdis = zeros(3,1);
    
end

end
