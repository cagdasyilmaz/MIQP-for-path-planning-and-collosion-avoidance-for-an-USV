function [Ac,Bc,Cc,Dc] = CtrlblSystemMatrixCalculator(A,B,C,D)
% Calculates controllable state space matrices from state space matrices

Co = ctrb(A,B);

SizeCtrl=rank(Co);

[Abar, Bbar, Cbar, ~, ~] = ctrbf(A,B,C);

Dbar = D;

Ac = Abar(size(Abar,1)-SizeCtrl+1:size(Abar,1),size(Abar,2)-SizeCtrl+1:size(Abar,2));
Bc = Bbar(size(Bbar,1)-SizeCtrl+1:size(Bbar,1),:);
Cc = Cbar(:,size(Cbar,2)-SizeCtrl+1:size(Cbar,2));
Dc = Dbar;

end

