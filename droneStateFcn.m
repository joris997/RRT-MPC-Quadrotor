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
function xk1 = droneStateFcn(xk,u)
uk = u(1:4);
Ts = u(5);
xk1 = droneDT(xk, uk, Ts);
end

