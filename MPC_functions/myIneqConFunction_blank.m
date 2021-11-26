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
function [cineq] = myIneqConFunction_blank(X,U,e,data,params)

p = data.PredictionHorizon;

Xd1=X(2:p+1,6);
Xd2=X(2:p+1,7);
Xd3=X(2:p+1,8);

U1 = U(1:p,data.MVIndex(1));
U2 = U(1:p,data.MVIndex(2));
U3 = U(1:p,data.MVIndex(3));
U4 = U(1:p,data.MVIndex(4));

cineq = [U1-30; -U1 ;  U2-30; -U2 ; ...
               U3-30; -U3 ; U4-30; -U4; Xd1-5; Xd2-5; Xd3-5];
     
   
end


