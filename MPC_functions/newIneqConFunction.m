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
function [cineq] = newIneqConFunction(X,U,e,data,params)
con_horizon = 5;

sphere_offset = [0 -7.5 0];

constraints_Npred = zeros(size(sphere_offset,1),1);
constraints_Nobs  = zeros(size(sphere_offset,1)*con_horizon,1);
for i = 1:size(sphere_offset,1)
    % radius is 1, discretized in 100 squares
    [x_s, y_s, z_s] = sphere(100);
    % displace the sphere
    x_s = x_s + sphere_offset(i,1);
    y_s = y_s + sphere_offset(i,2);
    z_s = z_s + sphere_offset(i,3);
    for ii = 1:con_horizon
        xk = X(ii,1:3);
        distance_mesh = sqrt((xk(1)-x_s).^2 + (xk(2)-y_s).^2 + (xk(3)-z_s).^2);
        [r,c] = find(distance_mesh == min(min(distance_mesh)));
        r = min(r); c = min(c);
        vector = [x_s(r,c) - xk(1);
                  y_s(r,c) - xk(2);
                  z_s(r,c) - xk(3)];
        normal = vector./norm(vector);

        constraint = (normal(1)*(xk(1) - x_s(r,c)) + ...
                      normal(2)*(xk(2) - y_s(r,c)) + ...
                      normal(3)*(xk(3) - z_s(r,c))) +0.1;
%         if constraint > 0
%             disp('constraint larger than zero')
%         end
        constraints_Npred(ii) = constraint;
    end
    constraints_Nobs(con_horizon*(i-1)+1:i*con_horizon) = constraints_Npred;
end
cineq = constraints_Nobs;
end
