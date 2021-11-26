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
function [ceq] = myEqConFunction(X,U,data,params)

p = data.PredictionHorizon;
Ts = 0.01;
X1 = X(2:p+1,1);
for i = 1:length(X1)
    R(i,:) = droneDT(X(i+1,:)',U(i+1,:)',Ts);
end


ceq = [ reshape(X(2:p+1,:)' - R' , [120,1]) ];
end

