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
close
%% Drone animation with moving obstacles
l=0.4;
velocityofreproduction=0.005; %0.07 is good
lobs=0.6;

figure('units','normalized','outerposition',[0 0 1 1])

%plot trajectory

subplot(2,4,[1 2 5 6])
plot3(x_n,y_n,z_n,'b'); 

subplot(2,4,[3 4 7 8])
plot3(x_n,y_n,z_n,'b'); 

% subplot(3,4,[9]);
% plot3(x_n,y_n,z_n,'b'); 
% 
% subplot(3,4,[11]);
% plot3(x_n,y_n,z_n,'b'); 
%first wall
x1=-13-1.9;
x2=-13+1.9;
y1=-12+1+lobs*sqrt(2);
y2=-12-1+lobs*sqrt(2);
z1=0;
z2=2.5;
subplot(2,4,[1 2 5 6])
hold on 
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])

subplot(2,4,[3 4 7 8])
hold on 
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])

% subplot(3,4,[9]);
% patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
% patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
% patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
% patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
% 
% % 
% % subplot(3,4,[10]);
% % patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
% % patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
% % patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
% % patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
% % patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
% 
% 
% subplot(3,4,[11]);
% patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
% patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
% patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
% patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])


%second small wall
x1=-15;
x2=15;
y1=-8;
y2=-9;
z1=0;
z2=5-0.5;

subplot(2,4,[1 2 5 6])
hold on 
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])

subplot(2,4,[3 4 7 8])
hold on 
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])
% subplot(3,4,[9]);
% patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
% patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
% patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
% patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])
% 
% % subplot(3,4,[10]);
% % patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
% % patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
% % patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
% % patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
% % patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
% % patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])
% subplot(3,4,[11]);
% patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
% patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
% patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
% patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])

%third wall

x1=-15;
x2=8-lobs*sqrt(2);
y1=-1-0.5;
y2=-1.5-0.5;
z1=0;
z2=8;

subplot(2,4,[1 2 5 6])
hold on 
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])


subplot(2,4,[3 4 7 8])
hold on 
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])
% subplot(3,4,[9]);
% patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
% patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
% patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
% patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])
% subplot(3,4,[10]);
% patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
% patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
% patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
% patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])
% subplot(3,4,[11]);
% patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
% patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
% patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
% patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])
%fourth wall
x1=-10+0.5;
x2=15;
y1=4.5;
y2=5;
z1=0;
z2=8;

subplot(2,4,[1 2 5 6])
hold on 
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.35 0 0])


subplot(2,4,[3 4 7 8])
hold on 
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.35 0 0])

% subplot(3,4,[9]);
% patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
% patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
% patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
% patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.35 0 0])

% subplot(3,4,[10]);
% patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
% patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
% patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
% patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.35 0 0])
% subplot(3,4,[11]);
% patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
% patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
% patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
% patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
% patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.35 0 0])

%small final wall
x1=10.5;
x2=11;
y1=10;
y2=5;
z1=0;
z2=8;

subplot(2,4,[1 2 5 6])
hold on 
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.35 0 0])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.35 0 0])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.35 0 0])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.35 0 0])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.35 0 0])
patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.35 0 0])

subplot(2,4,[3 4 7 8])
hold on 
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.35 0 0])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.35 0 0])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.35 0 0])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.35 0 0])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.35 0 0])
patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.35 0 0])

% subplot(3,4,[9]);
% patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.35 0 0])
% patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.35 0 0])
% patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.35 0 0])
% patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.35 0 0])
% patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.35 0 0])
% patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.35 0 0])

% subplot(3,4,[10]);
% patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.35 0 0])
% patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.35 0 0])
% patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.35 0 0])
% patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.35 0 0])
% patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.35 0 0])
% patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.35 0 0])
% subplot(3,4,[11]);
% patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.35 0 0])
% patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.35 0 0])
% patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.35 0 0])
% patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.35 0 0])
% patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.35 0 0])
% patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.35 0 0])


%initialization sphere
[Xs,Ys,Zs] = sphere;  
Xs=Xs*1;
Ys=Ys*1;
Zs=Zs*3+3.5;

%man3
      vx3=0;
      vy3=0;
      xi3=0;
      yi3=10;
      hold on 
% subplot(3,4,[11]);     
%  m33=surf(Xs+xi3+i*vx3,Ys+yi3+i*vy3,Zs);
% subplot(3,4,[9]); 
% hold on 
%  m33=surf(Xs+xi3+i*vx3,Ys+yi3+i*vy3,Zs);
subplot(2,4,[3 4 7 8]); 
hold on 
m33=surf(Xs+xi3+vx3,Ys+yi3+vy3,Zs);
subplot(2,4,[1 2 5 6]); 
hold on 
m33=surf(Xs+xi3+vx3,Ys+yi3+vy3,Zs);

length_video = length(xktotal);
 
for i =1:length_video
     
      subplot(2,4,[1 2 5 6])
      xlim([-15 15]) 
      ylim([-15 15])
      zlim([0,8])
      xlabel('x')
      ylabel('y')
      zlabel('z')
      
%       subplot(3,4,[9]);
%       xlim([-15 15]) 
%       ylim([-15 15])
%       zlim([0,8])
%       xlabel('x')
%       ylabel('y')
%       zlabel('z')
%       
%       subplot(3,4,[10]);
%       xlim([-15 15]) 
%       ylim([-15 15])
%       zlim([0,8])
%       xlabel('x')
%       ylabel('y')
%       zlabel('z')
%       subplot(3,4,[11]);
%       xlim([-15 15]) 
%       ylim([-15 15])
%       zlim([0,8])
%       xlabel('x')
%       ylabel('y')
%       zlabel('z')
%       
      subplot(2,4,[3 4 7 8])
      xlim([-15 15]) 
      ylim([-15 15])
      zlim([0,8])
      xlabel('x')
      ylabel('y')
      zlabel('z')      
     
      x=xktotal(1,i);
      y=xktotal(2,i);
      z=xktotal(3,i);
      phi=xktotal(4,i);
      theta=xktotal(5,i);
      psi=xktotal(6,i);
      xd=xktotal(7,i);
      yd=xktotal(8,i);
      zd=xktotal(9,i);
      
      rot=[cos(psi)*cos(theta),cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi),cos(psi)*sin(theta)*cos(phi)-sin(psi)*sin(phi);...
           sin(psi)*cos(theta),sin(phi)*sin(theta)*sin(psi)-cos(phi)*cos(psi),sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);...
          -sin(theta),cos(theta)*sin(phi),cos(theta)*cos(phi)];
      
      l=0.4;
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
      
      subplot(2,4,[1 2 5 6])
          
      if i>1
       delete(m11)
       delete(m21)
      
       delete(d11)
       delete(d21)
       delete(d31)
       delete(d41)
       delete(d51)
       delete(d61)
       delete(v11) 
      end
      
      subplot(2,4,[3 4 7 8])
      
      if i>1
       delete(m13)
       delete(m23)
   
       delete(d13)
       delete(d23)
       delete(d33)
       delete(d43)
       delete(d53)
       delete(d63)
       delete(v13) 
      end
       
%       subplot(3,4,[9]);
%           
%       if i>1
%        delete(m12)
%        delete(m22)
%     
%        delete(d12)
%        delete(d22)
%        delete(d32)
%        delete(d42)
%        delete(d52)
%        delete(d62)
%        delete(v12) 
%       end
%       
%       subplot(3,4,[10]);
%           
%       if i>1
%        delete(m14)
%        delete(m24)

%        delete(d14)
%        delete(d24)
%        delete(d34)
%        delete(d44)
%        delete(d54)
%        delete(d64)
%        delete(v14) 
%       end
% 
%       subplot(3,4,[11]);
%           
%       if i>1
%        delete(m15)
%        delete(m25)
%       
%        delete(d15)
%        delete(d25)
%        delete(d35)
%        delete(d45)
%        delete(d55)
%        delete(d65)
%        delete(v15) 
%       end
%       
       subplot(2,4,[1 2 5 6])
      %representation drone
      d11=patch(xdronebase,ydronebase,zdronebase,'red');
      d21=patch(xdronewall1,ydronewall1,zdronewall1,'green');
      d31=patch(xdronewall2,ydronewall2,zdronewall2,'green');
      d41=patch(xdronewall3,ydronewall3,zdronewall3,[0.9100 0.4100 0.1700]);
      d51=patch(xdronewall4,ydronewall4,zdronewall4,[0.9100 0.4100 0.1700]);
      d61=patch(xdroneceil,ydroneceil,zdroneceil,'y');
      
      %velocity vector
      vscalefactor=7;
      v11=patch([x,x+xd/vscalefactor],[y,y+yd/vscalefactor],[z,z+zd/vscalefactor],'b');
      
      Ts=0.01;
      frequency=20/4;
      %man1
      vx1=0;
      vy1=3*sin((i-1)*Ts*frequency+pi)
      xi1=-7;
      yi1=10;
      %man2
      vx2=0;
      vy2=3*sin((i-1)*Ts*frequency)
      xi2=5;
      yi2=10;
      
      %moving obstacles
      hold on 
      m11=surf(Xs+xi1+vx1,Ys+yi1+vy1,Zs);
      m21=surf(Xs+xi2+vx2,Ys+yi2+vy2,Zs);
      
      hold off
      
      view(80,30)
      grid on 
      
      
%       subplot(3,4,[9]);
%       
%       %representation drone
%       d12=patch(xdronebase,ydronebase,zdronebase,'red');
%       d22=patch(xdronewall1,ydronewall1,zdronewall1,'green');
%       d32=patch(xdronewall2,ydronewall2,zdronewall2,'green');
%       d42=patch(xdronewall3,ydronewall3,zdronewall3,[0.9100 0.4100 0.1700]);
%       d52=patch(xdronewall4,ydronewall4,zdronewall4,[0.9100 0.4100 0.1700]);
%       d62=patch(xdroneceil,ydroneceil,zdroneceil,'y');
%       
%       %velocity vector
%       vscalefactor=3;
%       v12=patch([x,x+xd/vscalefactor],[y,y+yd/vscalefactor],[z,z+zd/vscalefactor],'b');
%        
%       %moving obstacles
%       hold on 
%       m12=surf(Xs+xi1+i*vx1,Ys+yi1+i*vy1,Zs);
%       m22=surf(Xs+xi2+i*vx2,Ys+yi2+i*vy2,Zs);
%       
%       hold off
%             
%       view(0,90)
%       grid on 
%       
      
%       subplot(3,4,[10]);
%       
%       %representation drone
%       d14=patch(xdronebase,ydronebase,zdronebase,'red');
%       d24=patch(xdronewall1,ydronewall1,zdronewall1,'green');
%       d34=patch(xdronewall2,ydronewall2,zdronewall2,'green');
%       d44=patch(xdronewall3,ydronewall3,zdronewall3,[0.9100 0.4100 0.1700]);
%       d54=patch(xdronewall4,ydronewall4,zdronewall4,[0.9100 0.4100 0.1700]);
%       d64=patch(xdroneceil,ydroneceil,zdroneceil,'y');
%       
%       %velocity vector
%       vscalefactor=3;
%       v14=patch([x,x+xd/vscalefactor],[y,y+yd/vscalefactor],[z,z+zd/vscalefactor],'b');
%        
%       %moving obstacles
%       hold on 
%       m14=surf(Xs+xi1+i*vx1,Ys+yi1+i*vy1,Zs);
%       m24=surf(Xs+xi2+i*vx2,Ys+yi2+i*vy2,Zs);
%       m34=surf(Xs+xi3+i*vx3,Ys+yi3+i*vy3,Zs);
%       hold off
%             
%       view(90,0)
%       grid on 
%       
%       subplot(3,4,[11]);
%       
%       %representation drone
%       d15=patch(xdronebase,ydronebase,zdronebase,'red');
%       d25=patch(xdronewall1,ydronewall1,zdronewall1,'green');
%       d35=patch(xdronewall2,ydronewall2,zdronewall2,'green');
%       d45=patch(xdronewall3,ydronewall3,zdronewall3,[0.9100 0.4100 0.1700]);
%       d55=patch(xdronewall4,ydronewall4,zdronewall4,[0.9100 0.4100 0.1700]);
%       d65=patch(xdroneceil,ydroneceil,zdroneceil,'y');
%       
%       %velocity vector
%       vscalefactor=3;
%       v15=patch([x,x+xd/vscalefactor],[y,y+yd/vscalefactor],[z,z+zd/vscalefactor],'b');
%        
%       %moving obstacles
%       hold on 
%       m15=surf(Xs+xi1+i*vx1,Ys+yi1+i*vy1,Zs);
%       m25=surf(Xs+xi2+i*vx2,Ys+yi2+i*vy2,Zs);
%      
%       hold off
%             
%       view(90,00)
%       grid on 
%       
           
      subplot(2,4,[3 4 7 8])
      %representation drone
      d13=patch(xdronebase,ydronebase,zdronebase,'red');
      d23=patch(xdronewall1,ydronewall1,zdronewall1,'green');
      d33=patch(xdronewall2,ydronewall2,zdronewall2,'green');
      d43=patch(xdronewall3,ydronewall3,zdronewall3,[0.9100 0.4100 0.1700]);
      d53=patch(xdronewall4,ydronewall4,zdronewall4,[0.9100 0.4100 0.1700]);
      d63=patch(xdroneceil,ydroneceil,zdroneceil,'y');
      
      %velocity vector
      vscalefactor=7;
      v13=patch([x,x+xd/vscalefactor],[y,y+yd/vscalefactor],[z,z+zd/vscalefactor],'b');
       
      %moving obstacles
      hold on 
      m13=surf(Xs+xi1+vx1,Ys+yi1+vy1,Zs);
      m23=surf(Xs+xi2+vx2,Ys+yi2+vy2,Zs);
      
      hold off
      
      view(0,90)
      grid on 
      
      pause(velocityofreproduction)
end

