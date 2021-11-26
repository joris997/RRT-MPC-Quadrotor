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

global r_0 


Nobs = 3; %number of obstacles
psi_o = [pi/6 pi/5 pi/3];    % Obstacles orientations 
theta_o = [pi/2 pi/4 pi/7];
phi_o = [pi/2 pi/3 pi/8];

rx = [0.5 0.3 0.5];
ry = [0.5 0.3 0.5];
rz = [0.5 0.3 0.5];


    
theta_eps = [0.1 0.1 0.1];



IneqParams.angles.psi_o=psi_o;
IneqParams.angles.phi_o=phi_o;
IneqParams.angles.theta_o=theta_o;

IneqParams.dim.rx=rx;
IneqParams.dim.ry=ry;
IneqParams.dim.rz=rz;

IneqParams.sensitivity.theta_eps=theta_eps;

IneqParams.Nobs = Nobs;

IneqParams.ObsPoisitions=r_0;



% pre-allocations
R_OW = zeros(3,3,3);
M = zeros(3,3,3);
Q = zeros(3,3,3);

% prediction horizon 
p = data.PredictionHorizon;
cineq = [];
cineq_obstacles = zeros(IneqParams.Nobs*p,1);

         
for i=1:IneqParams.Nobs               
R_OW(:,:,i) = R_ow(IneqParams.angles.psi_o(i),IneqParams.angles.theta_o(i),IneqParams.angles.phi_o(i));
M(:,:,i) = diag([IneqParams.dim.rx(i)^-2 IneqParams.dim.ry(i)^-2 IneqParams.dim.rz(i)^-2]);
Q(:,:,i) = R_OW(:,:,i)'*M(:,:,i)*R_OW(:,:,i);
end

X1 = X(2:p+1,1);
X2 = X(2:p+1,2);
X3 = X(2:p+1,3);

Xd1 = X(2:p+1,7);
Xd2 = X(2:p+1,8);
Xd3 = X(2:p+1,9);

RX1 = ones(length(X1),1)*IneqParams.ObsPoisitions(1,1);
RY1 = ones(length(X1),1)*IneqParams.ObsPoisitions(2,1);
RZ1 = ones(length(X1),1)*IneqParams.ObsPoisitions(3,1);
RX2 = ones(length(X2),1)*IneqParams.ObsPoisitions(1,2);
RY2 = ones(length(X2),1)*IneqParams.ObsPoisitions(2,2);
RZ2 = ones(length(X2),1)*IneqParams.ObsPoisitions(3,2);
RX3 = ones(length(X3),1)*IneqParams.ObsPoisitions(1,3);
RY3 = ones(length(X3),1)*IneqParams.ObsPoisitions(2,3);
RZ3 = ones(length(X3),1)*IneqParams.ObsPoisitions(3,3);


Theta1 = ones(length(X1),1)*IneqParams.sensitivity.theta_eps(1);
Theta2 = ones(length(X2),1)*IneqParams.sensitivity.theta_eps(2);
Theta3 = ones(length(X3),1)*IneqParams.sensitivity.theta_eps(3);


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
         
     
          -sqrt(abs(X1.^2*Q(1,1,3) + X2.^2*Q(2,2,3) + X3.^2*Q(3,3,3) + X1.*X2*(Q(1,2,3)+Q(2,1,3)) + ...
         X1.*X3*(Q(1,3,3) + Q(3,1,3)) + X2.*X3*(Q(2,3,3) + Q(3,2,3)) - ...
         (RX3.^2*Q(1,1,3) + RY3.^2*Q(2,2,3) + RZ3.^2*Q(3,3,3) + RX3.*RY3*(Q(1,2,3)+Q(2,1,3)) + ... %
         RX3.*RZ3*(Q(1,3,3) + Q(3,1,3)) + RY3.*RZ1*(Q(2,3,3) + Q(3,2,3))))) + ones(length(X2),1) - Theta3*e; 
         
         
         Xd1-5;
         Xd2-5;
         Xd3-5;
         
         U1 - 30;
         -U1 ;
         U2 - 30;
         -U2 ; 
         U3 - 30;
         -U3 ;
         U4 - 30; 
         -U4 ];
     

end
