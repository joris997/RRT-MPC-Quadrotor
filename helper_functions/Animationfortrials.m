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
%% Drone animation with moving obstacles
l=0.4

figure(11)

%plot trajectory
plot3(x_n,y_n,z_n,'b'); 
%first wall
x1=-5+0.1;
x2=5+0.9;
y1=-0.40;
y2=0.4;
z1=-3+0.5;
z2=3+0.8;

patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])



%initialization sphere
[Xs,Ys,Zs] = sphere;
Xs=Xs/3;
Ys=Ys/3;
Zs=Zs/3;
%man1
vx1=-0.2086;
vy1=-0.2021;
vz1=0.2843;
xi1=-1;
yi1=-2;


for i =1:length(xktotal) 
      figure(11)
      
      view(40,60)
      xlim([-10 10]) 
      ylim([-10 10])
      zlim([-5,5])
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
      
      
      if i>1
       delete(m1)
       
       delete(d1)
       delete(d2)
       delete(d3)
       delete(d4)
       delete(d5)
       delete(d6)
       delete(v1) 
       end
      
      %representation drone
      d1=patch(xdronebase,ydronebase,zdronebase,'red');
      d2=patch(xdronewall1,ydronewall1,zdronewall1,'green');
      d3=patch(xdronewall2,ydronewall2,zdronewall2,'green');
      d4=patch(xdronewall3,ydronewall3,zdronewall3,[0.9100 0.4100 0.1700]);
      d5=patch(xdronewall4,ydronewall4,zdronewall4,[0.9100 0.4100 0.1700]);
      d6=patch(xdroneceil,ydroneceil,zdroneceil,'y');
      
      %velocity vector
      vscalefactor=7;
      v1=patch([x,x+xd/vscalefactor],[y,y+yd/vscalefactor],[z,z+zd/vscalefactor],'b');
       
      %moving obstacles
      hold on 
      m1=surf(Xs+xi1+i*vx1,Ys+yi1+i*vy1,Zs+vz1);
   
      hold off
      
      view(-10,45)
      grid on 
      
      pause(0.07)
end
