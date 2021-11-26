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
close all; clc;
addpath('MPC_functions')

global r_0 t timestep

%% constants
Ts = 0.12;
 %number of obstacles
Nobs=3;

%% create controller

numStates = 12;
numOutputs = 6;
numControl = 4;

nlobj = nlmpc(numStates,numOutputs,numControl);
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 8;
nlobj.ControlHorizon = 3;

nlobj.Model.StateFcn = "droneDT";
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 1;

nlobj.Model.OutputFcn = @(x,u,Ts) x(1:numOutputs);

%% define constraints
nlobj.Weights.OutputVariables = [1 1 1 1 1 1]*5;
nlobj.Weights.ManipulatedVariablesRate = [1 1 1 1]*0.1;


%% initialization
x0 = [-13 -12 0.5 0 0 0 0 0 0 0 0 0]';
u0 = zeros(numControl,1);

EKF = extendedKalmanFilter(@droneStateFcn,@droneMeasurementFcn);
EKF.State = x0;
uk = u0;

nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};

%% run on simulated data
duration = round(length(x_n)/10); %changed by Fra to make it always consistent with trajectory.
yref = [x_n' y_n' z_n' x_n'-x_n' x_n'-x_n' x_n'-x_n'];
        
y = x0(1:6);

ukHistory = zeros(numControl,round(duration/Ts));
xHistory = zeros(numStates,round(duration/Ts+1));
xHistory(:,1) = x0;

xktotal=[];
c=1;
for timestep = 1:(length(x_n))
    t=timestep
    xk = correct(EKF,y)
    xktotal=[xktotal,xk];  
   %compute optimal control actions
   % for ii = 1:Nobs
        if  timestep>288 %sum(abs(xk(1:3) - r_0(ii,:)')) < 3 &&
            if c==1
            nlobj.Weights.OutputVariables = [1 1 1 1 1 1]*1.7;
            nlobj.Weights.ManipulatedVariablesRate = [1 1 1 1]*0.1;
            nlobj.PredictionHorizon = 6;
            nlobj.ControlHorizon = 1;
            c=c+1;
            end
            nlobj.Optimization.CustomIneqConFcn = "myIneqConFunction_pers";
            %break
        else
           nlobj.Optimization.CustomIneqConFcn = "myIneqConFunction_blank";
        end
  % end
  
    [uk,nloptions,info] = nlmpcmove(nlobj,xk,uk,yref(timestep:min(timestep+9,(length(x_n))),:),[],nloptions);
    info.ExitFlag
    ukHistory(:,timestep) = uk;
    % Predict prediction model states for the next iteration
    predict(EKF,[uk; Ts]);
    % Implement first optimal control move
    x = droneDT(xk,uk,Ts);
    % Generate sensor data
    y = x(1:numOutputs) + randn(numOutputs,1)*0.01;
    % Save plant states
    xHistory(:,timestep+1) = x;
    timestep
    
end


figure
subplot(1,2,1)
plot3(xHistory(1,:),xHistory(2,:),xHistory(3,:),'-*')
title('drone location')
hold on
plot3(x_n,y_n,z_n)
xlim([-10 10])
ylim([-10 10])
zlim([-10 10])
grid on

subplot(1,2,2)
plot(ukHistory(1,:))
hold on;
plot(ukHistory(2,:))
plot(ukHistory(3,:))
plot(ukHistory(4,:))
title('control input')
grid on

