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
function [R] = R_ow(psi,theta,phi)

Rz=[ cos(psi) sin(psi) 0;
    sin(psi) cos(psi) 0;
    0 0 1];
Ry = [cos(theta) 0 sin(theta);
    0 1 0;
    -sin(theta) 0 cos(theta)];
Rx = [1 0 1;
    0 cos(phi) -sin(phi);
    0 sin(phi) cos(phi)];
R = Rz*Ry*Rx;

end

