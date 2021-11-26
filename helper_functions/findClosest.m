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
function idx = findClosest(xk,x,y,z)
    [val1,idx1] = min((xk(1)-x).^2 + (xk(2)-y).^2 + (xk(3)-z).^2);
    [~,idx2] = min(val1);
    idx = [idx1 idx2];
end