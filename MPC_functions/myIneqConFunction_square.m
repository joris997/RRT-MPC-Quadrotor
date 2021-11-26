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
function [cineq] = myIneqConFunction(X,U,e,data,params)
%% qube in environment
cp = [ -1,  2;   % x=-1 x=2 
       1, 2.5;   %y=1 
       -2 2];
cube_ineq = zeros(size(X,1),1);
for i = 1:size(X,1)
    xs = X(i,1); 
    ys = X(i,2);
    zs = X(i,3);
    p = 10;
    a = 1; b = 0; c = 0;
    eq1 = a*(xs- cp(1,1)) + b*(ys - 0) + c*(zs - 0);
    a = -1;
    eq2 = a*(xs- cp(1,2)) + b*(ys - 0) + c*(zs - 0);
    a = 0; b = 1; c = 0;
    eq3 = a*(xs- 0) + b*(ys - cp(2,1)) + c*(zs - 0);
    b = -1;
    eq4 = a*(xs- 0) + b*(ys - cp(2,2)) + c*(zs - 0);
    a = 0; b = 0; c = 1;
    eq5 = a*(xs- 0) + b*(ys - 0) + c*(zs - cp(3,1));
    c = -1;
    eq6 = a*(xs- 0) + b*(ys - 0) + c*(zs - cp(3,2));
    new_ineq = -log((exp(-eq1*p)+exp(-eq2*p)+exp(-eq3*p)+ ...
                exp(-eq4*p)+exp(-eq5*p)+exp(-eq6*p))^(1/p));
    new_ineq(~isfinite(new_ineq)) = -1;
    
    cube_ineq(i) = new_ineq;
end

%% ellipsoid
if 0
    global r_o

    Nobs = 3; %number of obstacles
    psi_o   = [pi/6 pi/5 pi/3];    % Obstacles orientations 
    theta_o = [pi/2 pi/4 pi/7];
    phi_o   = [pi/2 pi/3 pi/8];

    rx = [0.5 10 1];
    ry = [0.5 10 1];
    rz = [0.5 10 1];

    % r_o = [ x_n(20) y_n(20) z_n(20);
    %         x_n(5) y_n(5) z_n(5);
    %         x_n(45) y_n(45) z_n(45)];


    theta_eps = [0.15 0.15 0.15];
    theta_pi = [0.1 0.2 0.4 0.6 0.7 0.9];


    IneqParams.angles.psi_o=psi_o;
    IneqParams.angles.phi_o=phi_o;
    IneqParams.angles.theta_o=theta_o;

    IneqParams.dim.rx=rx;
    IneqParams.dim.ry=ry;
    IneqParams.dim.rz=rz;

    IneqParams.sensitivity.theta_eps=theta_eps;
    IneqParams.sensitivity.theta_pi=theta_pi;

    IneqParams.Nobs = Nobs;

    IneqParams.ObsPoisitions=r_o;

    %Rommsize

    w=30;
    l=30;
    h=30;

    % pre-allocations
    R_OW = zeros(3,3,3);
    M = zeros(3,3,3);
    Q = zeros(3,3,3);
    Eps = zeros(IneqParams.Nobs,1);
    Slack_eps = zeros(IneqParams.Nobs,1);
    Slack_pi = zeros(6,1);
    % prediction horizon 
    p = data.PredictionHorizon;
    cineq = [];
    cineq_boundaries = zeros(6*p,1); 
    cineq_obstacles = zeros(IneqParams.Nobs*p,1);

    n = [1 0 0;         % [x y z]
         0 1 0;
         0 0 1;
         -1 0 0;
         0 -1 0;
         0 0 -1];

    r_boundary = [0 w/2 h/2;  % position walls, floor and roof 
                   l/2 0 h/2;
                   l/2 w/2 0;
                   l w/2 h/2;
                   l/2 w h/2;
                   l/2 w/2 h];


    for i=1:IneqParams.Nobs               
    R_OW(:,:,i) = R_ow(IneqParams.angles.psi_o(i),IneqParams.angles.theta_o(i),IneqParams.angles.phi_o(i));
    M(:,:,i) = diag([IneqParams.dim.rx(i)^-2 IneqParams.dim.ry(i)^-2 IneqParams.dim.rz(i)^-2]);
    Q(:,:,i) = R_OW(:,:,i)'*M(:,:,i)*R_OW(:,:,i);
    end
    l=1;


    X1 = X(2:p+1,1);
    X2 = X(2:p+1,2);
    X3 = X(2:p+1,3);

    RX1 = ones(length(X1),1)*IneqParams.ObsPoisitions(1,1);
    RY1 = ones(length(X1),1)*IneqParams.ObsPoisitions(1,2);
    RZ1 = ones(length(X1),1)*IneqParams.ObsPoisitions(1,3);
    RX2 = ones(length(X2),1)*IneqParams.ObsPoisitions(2,1);
    RY2 = ones(length(X2),1)*IneqParams.ObsPoisitions(2,2);
    RZ2 = ones(length(X2),1)*IneqParams.ObsPoisitions(2,3);
    RX3 = ones(length(X3),1)*IneqParams.ObsPoisitions(3,1);
    RY3 = ones(length(X3),1)*IneqParams.ObsPoisitions(3,2);
    RZ3 = ones(length(X3),1)*IneqParams.ObsPoisitions(3,3);


    Theta1 = ones(length(X1),1)*IneqParams.sensitivity.theta_eps(1);
    Theta2 = ones(length(X2),1)*IneqParams.sensitivity.theta_eps(2);
    Theta3 = ones(length(X3),1)*IneqParams.sensitivity.theta_eps(3);

    % for t=2:p+1
    %  
    %     for j=1:6
    %         l=l+1;
    %         Pi(j) = n(j,:)*(X(t,1:3)-r_boundary(j,:))';
    %         Slack_pi(j) =   IneqParams.sensitivity.theta_pi(j)*e;
    %        
    %     end
    %     
    %  cineq_boundaries(6*(t-2)+1:6*(t-2)+6) = -Pi' - Slack_pi;
    %         
    %  
    % end

    % for t=2:p+1
    %     for i=1:IneqParams.Nobs
    %       
    %         Eps(i) = (X(t,1:3)' - IneqParams.ObsPoisitions(:,i))'*Qi(:,:,i)*(X(t,1:3)' - IneqParams.ObsPoisitions(:,i));
    %         Slack_eps(i) = IneqParams.sensitivity.theta_eps(i) *e;
    %     end    
    % cineq_obstacles(3*(t-2)+1:3*(t-2)+3) =  - Eps' - Slack_eps + 3*ones(IneqParams.Nobs,1) ;
    % end

    U1 = U(1:p,data.MVIndex(1));
    U2 = U(1:p,data.MVIndex(2));
    U3 = U(1:p,data.MVIndex(3));
    U4 = U(1:p,data.MVIndex(4));

    % cineq_input = [U1 - 30; -U1 ;  U2 - 30; -U2 ; ...
    %                U3 - 30; -U3 ; U4 - 30; -U4 ];

    cineq = [-sqrt(abs(X1.^2*Q(1,1,1) + X2.^2*Q(2,2,1) + X3.^2*Q(3,3,1) + X1.*X2*(Q(1,2,1)+Q(2,1,1)) + ...
             X1.*X3*(Q(1,3,1) + Q(3,1,1)) + X2.*X3*(Q(2,3,1) + Q(3,2,1)) - ...
             (RX1.^2*Q(1,1,1) + RY1.^2*Q(2,2,1) + RZ1.^2*Q(3,3,1) + RX1.*RY1*(Q(1,2,1)+Q(2,1,1)) + ...
             RX1.*RZ1*(Q(1,3,1) + Q(3,1,1)) + RY1.*RZ1*(Q(2,3,1) + Q(3,2,1)))))   + ones(length(X1),1) - Theta1*e; %

             -sqrt(abs(X1.^2*Q(1,1,2) + X2.^2*Q(2,2,2) + X3.^2*Q(3,3,2) + X1.*X2*(Q(1,2,2)+Q(2,1,2)) + ...
             X1.*X3*(Q(1,3,2) + Q(3,1,2)) + X2.*X3*(Q(2,3,2) + Q(3,2,2)) - ...
             (RX2.^2*Q(1,1,2) + RY2.^2*Q(2,2,2) + RZ2.^2*Q(3,3,2) + RX2.*RY2*(Q(1,2,2)+Q(2,1,2)) + ...
             RX2.*RZ2*(Q(1,3,2) + Q(3,1,2)) + RY2.*RZ2*(Q(2,3,2) + Q(3,2,2)))))  + ones(length(X2),1) - Theta2*e; 
             %

              -sqrt(abs(X1.^2*Q(1,1,3) + X2.^2*Q(2,2,3) + X3.^2*Q(3,3,3) + X1.*X2*(Q(1,2,3)+Q(2,1,3)) + ...
             X1.*X3*(Q(1,3,3) + Q(3,1,3)) + X2.*X3*(Q(2,3,3) + Q(3,2,3)) - ...
             (RX3.^2*Q(1,1,3) + RY3.^2*Q(2,2,3) + RZ3.^2*Q(3,3,3) + RX3.*RY3*(Q(1,2,3)+Q(2,1,3)) + ... %
             RX3.*RZ3*(Q(1,3,3) + Q(3,1,3)) + RY3.*RZ3*(Q(2,3,3) + Q(3,2,3))))) + ones(length(X3),1) - Theta3*e; 

             U1 - 40;
             -U1 ;
             U2 - 40;
             -U2 ; 
             U3 - 40;
             -U3 ;
             U4 - 40; 
             -U4 ;
             cube_ineq];


    %cineq = [cineq_boundaries ; cineq_obstacles; cineq_input ] ; 
else
    cineq = cube_ineq;
end
end
