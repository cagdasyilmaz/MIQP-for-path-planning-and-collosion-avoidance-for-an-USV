function [Phi_Matrix, F_Matrix] = mpcgain(Ap,Bp,Cp,Nc,Np)

[m1,n1]=size(Cp);
[n1,n_in]=size(Bp);

A_e=eye(n1+m1,n1+m1);
A_e(1:n1,1:n1)=Ap;
A_e(n1+1:n1+m1,1:n1)=Cp*Ap;

B_e=zeros(n1+m1,n_in);
B_e(1:n1,:)=Bp;
B_e(n1+1:n1+m1,:)=Cp*Bp;

C_e=zeros(m1,n1+m1);
C_e(:,n1+1:n1+m1)=eye(m1,m1);

% Find F and Phi Matrix
hh(1:m1,:) = C_e;
F_Matrix(1:m1,:)  = C_e * A_e;
    
for kk = (m1+1):m1:m1*Np

hh(kk:(kk+m1-1),:) = hh(kk-m1:kk-1,:)*A_e;

F_Matrix(kk:(kk+m1-1),:) = ...
                                F_Matrix(kk-m1:kk-1,:)*A_e;
end


% the dimension of Phi_Matrix
Phi_Matrix = zeros(m1*Np,...
                   n_in*Nc); 
vv = hh*B_e;
Phi_Matrix(:,1:n_in) = vv;  % first two column of Phi_Matrix
    
i = m1;
for kk = (n_in+1):n_in:n_in*Nc
Phi_Matrix(:,kk:kk+n_in-1) = [zeros(i,n_in); ...
                         vv(1:m1*Np-i,:)];
i = i + m1;
end


end