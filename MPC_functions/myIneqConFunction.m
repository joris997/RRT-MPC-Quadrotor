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
global timestep 
global r_0


Ts=0.1;
Nobs = 4; %number of obstacles
psi_o = [pi/6 pi/5 pi/3 pi/2];    % Obstacles orientations 
theta_o = [pi/2 pi/4 pi/7 pi/2];
phi_o = [pi/2 pi/3 pi/8 pi/2];

rx = [1 10 1 10];
ry = [1 10 1 10];
rz = [1 10 1 10];

r_o = [ 5.003 0.49 0.150;
        -1.486 -5.95 1.133;
      -1.761 5.5 1.304 ;
      -1 -2 0];

rm_o = [-1 -2 0];  
  
rd = [-0.7086 -1.6021 1.0843];
  
rm_o = rm_o + + (timestep-1)*Ts*rd;

theta_eps = [0.15 0.15 0.15 0.15];

IneqParams.movingObs.position = rm_o;
IneqParams.angles.psi_o=psi_o;
IneqParams.angles.phi_o=phi_o;
IneqParams.angles.theta_o=theta_o;

IneqParams.dim.rx=rx;
IneqParams.dim.ry=ry;
IneqParams.dim.rz=rz;

IneqParams.sensitivity.theta_eps=theta_eps;
IneqParams.Nobs = Nobs;
IneqParams.ObsPoisitions=r_o;

% pre-allocations
R_OW = zeros(3,3,IneqParams.Nobs);
M = zeros(3,3,IneqParams.Nobs);
Q = zeros(3,3,IneqParams.Nobs);

% prediction horizon 
p = data.PredictionHorizon;
cineq = [];
 
X1 = X(2:p+1,1);
X2 = X(2:p+1,1);
X3 = X(2:p+1,1);
X4 = X(2:p+1,1);

RX1  = IneqParams.ObsPoisitions(1,1)*ones(length(X1),1); 
RY1  = IneqParams.ObsPoisitions(2,1)*ones(length(X1),1);  
RZ1  = IneqParams.ObsPoisitions(3,1)*ones(length(X1),1);  
RX2  = IneqParams.ObsPoisitions(1,2)*ones(length(X1),1);  
RY2  = IneqParams.ObsPoisitions(2,2)*ones(length(X1),1);  
RZ2  = IneqParams.ObsPoisitions(3,2)*ones(length(X1),1);  
RX3  = IneqParams.ObsPoisitions(1,3)*ones(length(X1),1);  
RY3  = IneqParams.ObsPoisitions(2,3)*ones(length(X1),1); 
RZ3  = IneqParams.ObsPoisitions(3,3)*ones(length(X1),1); 

RX4 = zeros(length(X4),1);
RY4 = zeros(length(X4),1);
RZ4 = zeros(length(X4),1);
           
for i=1:IneqParams.Nobs               
R_OW(:,:,i) = R_ow(IneqParams.angles.psi_o(i),IneqParams.angles.theta_o(i),IneqParams.angles.phi_o(i));
M(:,:,i) = diag([IneqParams.dim.rx(i)^-2 IneqParams.dim.ry(i)^-2 IneqParams.dim.rz(i)^-2]);
Q(:,:,i) = R_OW(:,:,i)'*M(:,:,i)*R_OW(:,:,i);
end

for i=1:length(X1)
RX4(i)  = IneqParams.movingObs.position(1) + (i-1)*Ts*rd(1);
RY4(i)  = IneqParams.movingObs.position(2) + (i-1)*Ts*rd(2);
RZ4(i)  = IneqParams.movingObs.position(3) + (i-1)*Ts*rd(3);
end 

Theta1 = ones(length(X1),1)*IneqParams.sensitivity.theta_eps(1);
Theta2 = ones(length(X2),1)*IneqParams.sensitivity.theta_eps(2);
Theta3 = ones(length(X3),1)*IneqParams.sensitivity.theta_eps(3);
Theta4 = ones(length(X4),1)*IneqParams.sensitivity.theta_eps(4);

U1 = U(1:p,data.MVIndex(1));
U2 = U(1:p,data.MVIndex(2));
U3 = U(1:p,data.MVIndex(3));
U4 = U(1:p,data.MVIndex(4));
       
cineq = [-sqrt(abs(X1.^2*Q(1,1,1) + X2.^2*Q(2,2,1) + X3.^2*Q(3,3,1) + X1.*X2*(Q(1,2,1)+Q(2,1,1)) + ...
         X1.*X3*(Q(1,3,1) + Q(3,1,1)) + X2.*X3*(Q(2,3,1) + Q(3,2,1)) - ...
         (RX1.^2*Q(1,1,1) + RY1.^2*Q(2,2,1) + RZ1.^2*Q(3,3,1) + RX1.*RY1*(Q(1,2,1)+Q(2,1,1)) + ...
         RX1.*RZ1*(Q(1,3,1) + Q(3,1,1)) + RY1.*RZ1*(Q(2,3,1) + Q(3,2,1)))))   + ones(length(X1),1) - Theta1*e; %
         
         -sqrt(abs(X1.^2*Q(1,1,2) + X2.^2*Q(2,2,2) + X3.^2*Q(3,3,2) + X1.*X2*(Q(1,2,2)+Q(2,1,2)) + ...
         X1.*X3*(Q(1,3,2) + Q(3,1,2)) + X2.*X3*(Q(2,3,2) + Q(3,2,2)) - ...
         (RX2.^2*Q(1,1,2) + RY2.^2*Q(2,2,2) + RZ2.^2*Q(3,3,2) + RX2.*RY2*(Q(1,2,2)+Q(2,1,2)) + ...
         RX2.*RZ2*(Q(1,3,2) + Q(3,1,2)) + RY2.*RZ2*(Q(2,3,2) + Q(3,2,2)))))  + ones(length(X2),1) - Theta2*e; 
         %
     
          -sqrt(abs(X1.^2*Q(1,1,3) + X2.^2*Q(2,2,3) + X3.^2*Q(3,3,3) + X1.*X2*(Q(1,2,3)+Q(2,1,3)) + ...
         X1.*X3*(Q(1,3,3) + Q(3,1,3)) + X2.*X3*(Q(2,3,3) + Q(3,2,3)) - ...
         (RX3.^2*Q(1,1,3) + RY3.^2*Q(2,2,3) + RZ3.^2*Q(3,3,3) + RX3.*RY3*(Q(1,2,3)+Q(2,1,3)) + ... %
         RX3.*RZ3*(Q(1,3,3) + Q(3,1,3)) + RY3.*RZ1*(Q(2,3,3) + Q(3,2,3))))) + ones(length(X3),1) - Theta3*e; 
         
          -sqrt(abs(X1.^2*Q(1,1,4) + X2.^2*Q(2,2,4) + X3.^2*Q(3,3,4) + X1.*X2*(Q(1,2,4)+Q(2,1,4)) + ...
         X1.*X3*(Q(1,3,4) + Q(3,1,4)) + X2.*X3*(Q(2,3,4) + Q(3,2,4)) - ...
         (RX3.^2*Q(1,1,4) + RY3.^2*Q(2,2,4) + RZ3.^2*Q(3,3,4) + RX3.*RY3*(Q(1,2,4)+Q(2,1,4)) + ... %
         RX3.*RZ3*(Q(1,3,4) + Q(3,1,4)) + RY3.*RZ1*(Q(2,3,4) + Q(3,2,4))))) + ones(length(X4),1) - Theta4*e; 
         
         U1 - 40;
         -U1 ;
         U2 - 40;
         -U2 ; 
         U3 - 40;
         -U3 ;
         U4 - 40; 
         -U4 ];


%cineq = [cineq_boundaries ; cineq_obstacles; cineq_input ] ; 
end
