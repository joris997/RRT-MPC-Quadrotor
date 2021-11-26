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
function x = droneDT(x,u,Ts)
    g = 9.81;
    m = 1;
    b_1 = 1;
    b_2 = 1;
    b_3 = 1;
    
    l = 0.3;
    w = 0.3; 
    h = 0.1;
    
    Ixx = 1/12*m*(h^2+w^2); 
    Iyy = 1/12*m*(h^2+l^2); 
    Izz = 1/12*m*(l^2+w^2);

    l = 0.3;
    Km=20;
    Kf=1;
    
    phi = x(4);
    theta = x(5);
    psi = x(6);
    xd = x(7);
    yd = x(8);
    zd = x(9);
    phid = x(10);
    thetad = x(11);
    psid = x(12);
    
    T1 = u(1);
    T2 = u(2);
    T3 = u(3);
    T4 = u(4);
%     A = [zeros(6,6) eye(6);
%         0,0,0,0,0,0,0,0,0,g,0,0;
%         0,0,0,-g,0,0,0,0,0,0,0,0;
%         0,0,0,0,0,0,0,0,0,0,0,0;
%         0,0,0,0,0,0,0,0,0,0,0,0;
%         0,0,0,0,0,0,0,0,0,0,0,0;
%         0,0,0,0,0,0,0,0,0,0,0,0];
% 
%     B = [zeros(8,4);
%         diag([1/m, b_1, b_2, b_3])];
% 
%     C = [eye(3),zeros(3,9)];
% 
%     D = zeros(3,4);
% 
%     sysc=ss(A,B,C,D);
%     sysd = c2d(sysc,Ts);
%     x = sysd.A*x+sysd.B*u;
    
    Xdot=[xd;
          yd;
          zd;
          phid;
          thetad;
          psid;
          (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*(T1+T2+T3+T4)/m;
          (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*(T1+T2+T3+T4)/m;
          ((cos(phi)*cos(theta)*(T1+T2+T3+T4)-m*g))/m;
          (l*(T2-T4)-(Izz-Iyy)*thetad*psid)/Ixx;
          (l*(T3-T1)-(Ixx-Izz)*phid*psid)/Iyy;
          (((T1+T3)-(T2+T4))*Km/Kf-(Iyy-Ixx)*phid*thetad)/Izz];
    x = x+Ts*Xdot;
end