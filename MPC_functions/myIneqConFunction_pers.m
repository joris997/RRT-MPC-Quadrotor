%% RRT-MPC-Quadcopter
% Quadcopter global and local path planning with Rapidly-Exploring Random
% Tree search and nonlinear Model Predictive Control. 
%
% Created by:
%   Christos Vasileio
%   Cristian Meo
%   Francesco Stella
%   Joris Verhagen
%
% MIT License
%
% Created: April 2020

%% Start
function [cineq] = myIneqConFunction(X,U,e,data,params)
global timestep t 
%persistent  RX1 RX2 RX3 RX4 RY1 RY2 RY3 RY4 RZ1 RZ2 RZ3 RZ4 Theta1 Theta2 Theta3 Theta4 Q
persistent  RX1 RX2 RX3 RY1 RY2 RY3 RZ1 RZ2 RZ3 Theta1 Theta2 Theta3 Theta_pi1 Theta_pi2 Theta_pi3 Q cp n r_boundary
p = data.PredictionHorizon;

X1 = X(2:p+1,1);
X2 = X(2:p+1,1);
X3 = X(2:p+1,1);


if timestep == t
    
%little wall at the end
cp = [10.5 11;
       5 11;
       0 8];
           
Ts=0.12;
theta_eps = [0.15 0.15 0.15];% 
theta_pi = [0.1];

%IneqParams.movingObs.position = rm_o;
IneqParams.angles.psi_o=[0 0 0];
IneqParams.angles.phi_o=[0 0 0];
IneqParams.angles.theta_o=[0 0 0];

IneqParams.dim.rx=[1 1 1];
IneqParams.dim.ry=[1 1 1];
IneqParams.dim.rz=[1 1 1]*3;

IneqParams.sensitivity.theta_eps=theta_eps;
IneqParams.sensitivity.theta_pi=theta_pi;
IneqParams.Nobs = 3;
IneqParams.ObsPoisitions.static=[0 10 3.5];
IneqParams.ObsPoisitions.moving= [5 13 3.5; -7 7 3.5]'; 

% pre-allocations
R_OW = zeros(3,3,IneqParams.Nobs);
M = zeros(3,3,IneqParams.Nobs);
Q = zeros(3,3,IneqParams.Nobs);

cineq = [];
n = [1 0 0];
   % 0 0 1;
    %0 0 -1];

r_boundary = ones(p,1)*[11 5 8]; 

 
X1 = X(2:p+1,1);
X2 = X(2:p+1,1);
X3 = X(2:p+1,1);


RX1  = IneqParams.ObsPoisitions.static(1)*ones(length(X1),1); 
RY1  = IneqParams.ObsPoisitions.static(2)*ones(length(X1),1);  
RZ1  = IneqParams.ObsPoisitions.static(3)*ones(length(X1),1);  
RX2  = IneqParams.ObsPoisitions.moving(1,1)*ones(length(X1),1);  
RY2  = IneqParams.ObsPoisitions.moving(2,1)*ones(length(X1),1);  
RZ2  = IneqParams.ObsPoisitions.moving(3,1)*ones(length(X1),1);  
RX3  = IneqParams.ObsPoisitions.moving(1,2)*ones(length(X1),1);  
RY3  = IneqParams.ObsPoisitions.moving(2,2)*ones(length(X1),1); 
RZ3  = IneqParams.ObsPoisitions.moving(3,2)*ones(length(X1),1); 

frequency = 20/4;
            
for i=1:IneqParams.Nobs               
R_OW(:,:,i) = R_ow(IneqParams.angles.psi_o(i),IneqParams.angles.theta_o(i),IneqParams.angles.phi_o(i));
M(:,:,i) = diag([IneqParams.dim.rx(i)^-2 IneqParams.dim.ry(i)^-2 IneqParams.dim.rz(i)^-2]);
Q(:,:,i) = R_OW(:,:,i)'*M(:,:,i)*R_OW(:,:,i);
end

for i=1:length(X1)
RY2(i)  = 10 + 3*sin((i-1)*Ts*frequency+pi + timestep*Ts*frequency);
RY3(i)  = 10 + 3*sin((i-1)*Ts*frequency + timestep*Ts*frequency);
end 

Theta1 = ones(length(X1),1)*IneqParams.sensitivity.theta_eps(1);
Theta2 = ones(length(X2),1)*IneqParams.sensitivity.theta_eps(2);
Theta3 = ones(length(X3),1)*IneqParams.sensitivity.theta_eps(3);

Theta_pi1 = ones(length(X1),1)*IneqParams.sensitivity.theta_pi;
%Theta_pi2 = ones(length(X1),1)*IneqParams.sensitivity.theta_pi(2);
%Theta_pi3 = ones(length(X1),1)*IneqParams.sensitivity.theta_pi(3);
 
end
U1 = U(1:p,data.MVIndex(1));
U2 = U(1:p,data.MVIndex(2));
U3 = U(1:p,data.MVIndex(3));
U4 = U(1:p,data.MVIndex(4));


Xd1=X(2:p+1,7);
Xd2=X(2:p+1,8);
Xd3=X(2:p+1,9);


t=t+1;

cube_ineq = zeros(size(X,1)-1,1);    
for i = 1:size(X,1)-1   
    ps = 10;
    a = 1; b = 0; c = 0;
    eq1 = a*(X1(i)- cp(1,1)) + b*(X2(i) - 0) + c*(X3(i) - 0);
    a = -1;
    eq2 = a*(X1(i)- cp(1,2)) + b*(X2(i) - 0) + c*(X3(i)- 0);
    a = 0; b = 1; c = 0;
    eq3 = a*(X1(i) - 0) + b*(X2(i) - cp(2,1)) + c*(X3(i) - 0);
    b = -1;
    eq4 = a*(X1(i) - 0) + b*(X2(i) - cp(2,2)) + c*(X3(i) - 0);
    a = 0; b = 0; c = 1;
    eq5 = a*(X1(i)- 0) + b*(X2(i) - 0) + c*(X3(i) - cp(3,1));
    c = -1;
    eq6 = a*(X1(i)- 0) + b*(X2(i) - 0) + c*(X3(i) - cp(3,2));
    new_ineq = -log((exp(-eq1*ps)+exp(-eq2*ps)+exp(-eq3*ps)+ ...
                exp(-eq4*ps)+exp(-eq5*ps)+exp(-eq6*ps))^(1/ps));
    new_ineq(~isfinite(new_ineq)) = -1;
    cube_ineq(i) = new_ineq;
end   

cineq = [-sqrt(abs(X1.^2*Q(1,1,1) + X2.^2*Q(2,2,1) + X3.^2*Q(3,3,1) + X1.*X2*(Q(1,2,1)+Q(2,1,1)) + ...
         X1.*X3*(Q(1,3,1) + Q(3,1,1)) + X2.*X3*(Q(2,3,1) + Q(3,2,1)) - ...
         (RX1.^2*Q(1,1,1) + RY1.^2*Q(2,2,1) + RZ1.^2*Q(3,3,1) + RX1.*RY1*(Q(1,2,1)+Q(2,1,1)) + ...
         RX1.*RZ1*(Q(1,3,1) + Q(3,1,1)) + RY1.*RZ1*(Q(2,3,1) + Q(3,2,1)))))   + 2*ones(length(X1),1) - Theta1*e; %
         
         -sqrt(abs(X1.^2*Q(1,1,2) + X2.^2*Q(2,2,2) + X3.^2*Q(3,3,2) + X1.*X2*(Q(1,2,2)+Q(2,1,2)) + ...
         X1.*X3*(Q(1,3,2) + Q(3,1,2)) + X2.*X3*(Q(2,3,2) + Q(3,2,2)) - ...
         (RX2.^2*Q(1,1,2) + RY2.^2*Q(2,2,2) + RZ2.^2*Q(3,3,2) + RX2.*RY2*(Q(1,2,2)+Q(2,1,2)) + ...
         RX2.*RZ2*(Q(1,3,2) + Q(3,1,2)) + RY2.*RZ2*(Q(2,3,2) + Q(3,2,2)))))  + 2*ones(length(X2),1) - Theta2*e; 
         
     
           -sqrt(abs(X1.^2*Q(1,1,3) + X2.^2*Q(2,2,3) + X3.^2*Q(3,3,3) + X1.*X2*(Q(1,2,3)+Q(2,1,3)) + ...
          X1.*X3*(Q(1,3,3) + Q(3,1,3)) + X2.*X3*(Q(2,3,3) + Q(3,2,3)) - ...
          (RX3.^2*Q(1,1,3) + RY3.^2*Q(2,2,3) + RZ3.^2*Q(3,3,3) + RX3.*RY3*(Q(1,2,3)+Q(2,1,3)) + ... %
          RX3.*RZ3*(Q(1,3,3) + Q(3,1,3)) + RY3.*RZ1*(Q(2,3,3) + Q(3,2,3))))) + 2*ones(length(X3),1) - Theta3*e; 
         

         -(n(1,:)*(X(1:3,2:p+1)'-r_boundary)')'-Theta_pi1*e;
        % -(n(2,:)*(X(1:3,2:p+1)'-ones(p,1)*[0 0 1])')'-Theta_pi2*e;
        % -(n(3,:)*(X(1:3,2:p+1)'-ones(p,1)*[0 0 8])')'-Theta_pi3*e;
         
         
         Xd1 - 10;
         Xd2 - 10;
         Xd3 - 10;
         

         U1 - 40;
         -U1 ;
         U2 - 40;
         -U2 ; 
         U3 - 40;
         -U3 ;
         U4 - 40; 
         -U4;
         cube_ineq ];
     



end
