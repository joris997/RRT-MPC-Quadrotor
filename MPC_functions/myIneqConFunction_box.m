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
psi_o = [pi/6 pi/5 pi/3];    % Obstacles orientations 
theta_o = [pi/2 pi/4 pi/7];
phi_o = [pi/2 pi/3 pi/8];

rx = [2 0.1 1];
ry = [2 0.1 1];
rz = [2 0.1 1];
Nobs = 3;

% theta_eps = [0.4 0.6 0.8];
% theta_pi = [0.1 0.2 0.4 0.6 0.7 0.9];
theta_eps = [0.1 0.1 0.1];
% theta_pi = [500 500 500 500 500 500];

p=data.PredictionHorizon ;

angles.psi_o=psi_o;
angles.phi_o=phi_o;
angles.theta_o=theta_o;

dim.rx=rx;
dim.ry=ry;
dim.rz=rz;

sensitivity.theta_eps=theta_eps;
% sensitivity.theta_pi=theta_pi;
% 
ObsPoisitions=r_0;

maxPropForce = 40;
% 
% RoomDimensions.w=30;
% RoomDimensions.l=30;
% RoomDimensions.h=30;
% 
% 
% w=RoomDimensions.w;
% l=RoomDimensions.l;
% h=RoomDimensions.h;

% pre-allocations
R_OW = zeros(3,3,3);
M = zeros(3,3,3);
Qi = zeros(3,3,3);
Eps = zeros(Nobs,1);
Slack_eps = zeros(Nobs,1);
Slack_pi = zeros(6,1);
% prediction horizon 
cineq_obstacles = zeros(Nobs*p,1);
% cineq_boundaries = zeros(6*p,1); 
% 
% n = [1 0 0;         % [x y z]
%      0 1 0;
%      0 0 1;
%      -1 0 0;
%      0 -1 0;
%      0 0 -1];
%  
% r_boundary = [0 w/2 h/2;  % position walls, floor and roof 
%               l/2 0 h/2;
%               l/2 w/2 0;
%               l w/2 h/2;
%               l/2 w h/2;
%               l/2 w/2 h];
           
U1 = U(1:p,data.MVIndex(1));
U2 = U(1:p,data.MVIndex(2));
U3 = U(1:p,data.MVIndex(3));
U4 = U(1:p,data.MVIndex(4));

mPF = maxPropForce;
cineq_input = [U1 - mPF; -U1 ;  U2 - mPF; -U2 ; ...
               U3 - mPF; -U3 ; U4 - mPF; -U4 ]';    
         
for i=1:Nobs               
    R_OW(:,:,i) = R_ow(angles.psi_o(i),angles.theta_o(i),angles.phi_o(i));
    M(:,:,i) = diag([dim.rx(i)^-2 dim.ry(i)^-2 dim.rz(i)^-2]);
    Qi(:,:,i) = R_OW(:,:,i)'*M(:,:,i)*R_OW(:,:,i);
end
% l=1;
% for t=2:p+1
%     Pi = zeros(1,6);
%     for j=1:6
%         l=l+1;
%         Pi(j) = n(j,:)*(X(t,1:3)-r_boundary(j,:))';
%         Slack_pi(j) = sensitivity.theta_pi(j)*e;
%     end
%     cineq_boundaries(6*(t-2)+1:6*(t-2)+6) = -Pi' - Slack_pi;
% end
for t=2:p+1
    for i=1:Nobs
        Eps(i) = (X(t,1:3)' - ObsPoisitions(:,i))'*Qi(:,:,i)*(X(t,1:3)' - ObsPoisitions(:,i));
        Slack_eps(i) = sensitivity.theta_eps(i) *e;
    end 
    cineq_obstacles(3*(t-2)+1:3*(t-2)+3) =  -Eps - Slack_eps + ones(Nobs,1);
end
% cineq = [cineq_boundaries; cineq_obstacles; cineq_input'];
cineq = [cineq_obstacles; cineq_input'];
end

