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
% clear; close; clc
addpath('MPC_functions')

%% constants
Ts = 0.1;
 %number of obstacles
Nobs=3;
rx = [2 0.1 1];
ry = [2 0.1 1];
rz = [2 0.1 1];

%% create controller

numStates = 12;
numOutputs = 6;
numControl = 4;

nlobj = nlmpc(numStates,numOutputs,numControl);
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 6;
nlobj.ControlHorizon = 3;

nlobj.Model.StateFcn = "droneDT";
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 1;

nlobj.Model.OutputFcn = @(x,u,Ts) x(1:numOutputs);

%% define constraints
nlobj.Weights.OutputVariables = [1 1 1 1 1 1];
nlobj.Weights.ManipulatedVariablesRate = [1 1 1 1];


%% initialization
x0 = [8 -6.4 1 0 0 0 0 0 0 0 0 0]';
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

ukHistory = zeros(numControl,duration/Ts);
xHistory = zeros(numStates,duration/Ts+1);
xHistory(:,1) = x0;

xktotal=[];
for i = 1:(duration/Ts)
    xk = correct(EKF,y)
    xktotal=[xktotal,xk];  
  %  compute optimal control actions
    for ii = 1:Nobs
        if norm(xk(1:3) - r_o(ii,:)) < 7
            nlobj.Optimization.CustomIneqConFcn = "myIneqConFunction";
            break
        else
            nlobj.Optimization.CustomIneqConFcn = "myIneqConFunction";
        end
    end
    
    %nlobj.Optimization.CustomEqConFcn = "myEqConFunction";
    [uk,nloptions,info] = nlmpcmove(nlobj,xk,uk,yref(i:min(i+9,(duration/Ts)),:),[],nloptions);
    info.ExitFlag
    ukHistory(:,i) = uk;
    % Predict prediction model states for the next iteration
    predict(EKF,[uk; Ts]);
    % Implement first optimal control move
    x = droneDT(xk,uk,Ts);
    % Generate sensor data
    y = x(1:numOutputs);% + randn(numOutputs,1)*0.01;
    % Save plant states
    xHistory(:,i+1) = EKF.State;
    i
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



%% Creation of video with "drone"


figure(11)
plot3(x_n,y_n,z_n,'b');

for i =1:length(xktotal) 
      figure(11)
      view(40,60)
      hold on
       xlim([-10 10]) %the room is actually for now -10 10
       ylim([-10 10])
      zlim([-10,10])
      
      x=xktotal(1,i);
      y=xktotal(2,i);
      z=xktotal(3,i);
      phi=xktotal(4,i);
      theta=xktotal(5,i);
      psi=xktotal(6,i);
      xd=xktotal(7,i);
      yd=xktotal(8,i);
      zd=xktotal(9,i);
      
      rot=[cos(phi)*cos(theta),cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi),cos(phi)*sin(theta)*cos(psi)-sin(phi)*sin(psi);...
                 sin(phi)*cos(theta),sin(phi)*sin(theta)*sin(psi)-cos(phi)*cos(psi),sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);...
                 -sin(theta),cos(theta)*sin(psi),cos(theta)*cos(psi)];
      
      l=0.15;
      edge1=rot*[-l;-l;-l/8];
      edge2=rot*[l;-l;-l/8];
      edge3=rot*[l;l;-l/8];
      edge4=rot*[-l;l;-l/8];
      
      edgetop1=rot*[-l;-l;l/8];
      edgetop2=rot*[l;-l;l/8];
      edgetop3=rot*[l;l;l/8];
      edgetop4=rot*[-l;l;l/8];
      
      xdronebase=[x+edge1(1),x+edge2(1),x+edge3(1),x+edge4(1)];
      ydronebase=[y+edge1(2),y+edge2(2),y+edge3(2),y+edge4(2)];
      zdronebase=[z+edge1(3),z+edge2(3),z+edge3(3),z+edge4(3)];
      
      xdronewall1=[x+edge1(1),x+edge2(1),x+edgetop2(1),x+edgetop1(1)];
      ydronewall1=[y+edge1(2),y+edge2(2),y+edgetop2(2),y+edgetop1(2)];
      zdronewall1=[z+edge1(3),z+edge2(3),z+edgetop2(3),z+edgetop1(3)];
      
      xdronewall2=[x+edge3(1),x+edge4(1),x+edgetop4(1),x+edgetop3(1)];
      ydronewall2=[y+edge3(2),y+edge4(2),y+edgetop4(2),y+edgetop3(2)];
      zdronewall2=[z+edge3(3),z+edge4(3),z+edgetop4(3),z+edgetop3(3)];
      
      xdronewall3=[x+edge1(1),x+edge4(1),x+edgetop4(1),x+edgetop1(1)];
      ydronewall3=[y+edge1(2),y+edge4(2),y+edgetop4(2),y+edgetop1(2)];
      zdronewall3=[z+edge1(3),z+edge4(3),z+edgetop4(3),z+edgetop1(3)];
      
      xdronewall4=[x+edge3(1),x+edge2(1),x+edgetop2(1),x+edgetop3(1)];
      ydronewall4=[y+edge3(2),y+edge2(2),y+edgetop2(2),y+edgetop3(2)];
      zdronewall4=[z+edge3(3),z+edge2(3),z+edgetop2(3),z+edgetop3(3)];
      
      xdroneceil=[x+edgetop1(1),x+edgetop2(1),x+edgetop3(1),x+edgetop4(1)];
      ydroneceil=[y+edgetop1(2),y+edgetop2(2),y+edgetop3(2),y+edgetop4(2)];
      zdroneceil=[z+edgetop1(3),z+edgetop2(3),z+edgetop3(3),z+edgetop4(3)];
      
      patch(xdronebase,ydronebase,zdronebase,'red')
      patch(xdronewall1,ydronewall1,zdronewall1,'green')
      patch(xdronewall2,ydronewall2,zdronewall2,'green')
      patch(xdronewall3,ydronewall3,zdronewall3,[0.9100 0.4100 0.1700])
      patch(xdronewall4,ydronewall4,zdronewall4,[0.9100 0.4100 0.1700])
      patch(xdroneceil,ydroneceil,zdroneceil,'y')
      
      vscalefactor=7;
      
      patch([x,x+xd/vscalefactor],[y,y+yd/vscalefactor],[z,z+zd/vscalefactor],'b');
       
      hold off
      view(120,20)
      grid on 
      drawnow limitrate
end

% figure(11)
% hold on
% for i = 1:Nobs
%     [x, y, z] = ellipsoid(r_o(i,1),r_o(i,2),r_o(i,3),rx(i),ry(i),rz(i),30);
%     surf(x,y,z)
% end

cp = [ -1,  2;
       1, 2.5;
       -2 2];

ax = (cp(1,1)+cp(1,2))/2;  
ay = (cp(2,1)+cp(2,2))/2;  
az = (cp(3,1)+cp(3,2))/2;

hx = (cp(1,1)-cp(1,2))/2;
hy = (cp(2,1)-cp(2,2))/2;  
hz = (cp(3,1)-cp(3,2))/2;
patch_args = { 'FaceColor', 'b', 'FaceAlpha', 0.3 };

patch( 'XData', ax+[-hx -hx  hx  hx], 'YData', ay+[-hy  hy  hy -hy], 'ZData', az+[-hz -hz -hz -hz], patch_args{:} )
daspect( [1 1 1] )  % 1:1:1 aspect ratio.
hold on
patch( 'XData', ax+[-hx -hx  hx  hx], 'YData', ay+[-hy  hy  hy -hy], 'ZData', az+[ hz  hz  hz  hz], patch_args{:} )
patch( 'XData', ax+[-hx -hx  hx  hx], 'YData', ay+[ hy  hy  hy  hy], 'ZData', az+[-hz  hz  hz -hz], patch_args{:} )
patch( 'XData', ax+[-hx -hx  hx  hx], 'YData', ay+[-hy -hy -hy -hy], 'ZData', az+[-hz  hz  hz -hz], patch_args{:} )
patch( 'XData', ax+[ hx  hx  hx  hx], 'YData', ay+[-hy -hy  hy  hy], 'ZData', az+[-hz  hz  hz -hz], patch_args{:} )
patch( 'XData', ax+[-hx -hx -hx -hx], 'YData', ay+[-hy -hy  hy  hy], 'ZData', az+[-hz  hz  hz -hz], patch_args{:} )