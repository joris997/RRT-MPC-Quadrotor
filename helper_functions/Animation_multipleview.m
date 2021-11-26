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
velocityofreproduction=0.01 %0.07 is good


figure(11)

%plot trajectory
plot3(x_n,y_n,z_n,'b'); 
%first wall
x1=6.1;
x2=9.9;
y1=-5.4++l*sqrt(2);
y2=-7.4+l*sqrt(2);
z1=-1;
z2=2.5;
subplot(1,2,1)
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
subplot(1,2,2)
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])

%second wall
x1=-1.4;
x2=-9.1;
y1=6.4;
y2=4.4;
z1=-4.5;
z2=5;

subplot(1,2,1)
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])
subplot(1,2,2)
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])

%third wall

x1=0.2+l*sqrt(2);
x2=10+0.9;
y1=6.4;
y2=4.4+l*sqrt(2);
z1=0;
z2=5;
subplot(1,2,1)
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])
subplot(1,2,2)
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])

%fourth wall
x1=-5-l*sqrt(2);
x2=10;
y1=0+l*sqrt(2);
y2=0.5+l*sqrt(2);
z1=-1;
z2=3;
subplot(1,2,1)
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])
subplot(1,2,2)
patch([x1 x2 x2 x1],[y1 y1 y2 y2],[z1 z1 z1 z1],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y1 y1 y1 y1],[z1 z2 z2 z1],[0.7 0.7 0.7])
patch([x2 x2 x2 x2],[y1 y2 y2 y1],[z2 z2 z1 z1],[0.7 0.7 0.7])
patch([x2 x2 x1 x1],[y1 y2 y2 y1],[z2 z2 z2 z2],[0.7 0.7 0.7])
patch([x1 x1 x1 x1],[y1 y1 y2 y2],[z2 z1 z1 z2],[0.7 0.7 0.7])
patch([x1 x1 x2 x2],[y2 y2 y2 y2],[z1 z2 z2 z1],[0.7 0.7 0.7])

%initialization sphere
[Xs,Ys,Zs] = sphere;
Xs=Xs/1.5;
Ys=Ys/1.5;
Zs=Zs*1.5+1;
%man1
vx1=0.12;
vy1=0;
xi1=-9;
yi1=4;
%man2
vx2=-0.2;
vy2=0;
xi2=9;
yi2=-2;
%man3
vx3=0.1;
vy3=-0.3;
xi3=-0.5;
yi3=10;

for i =1:length(xktotal) 
     
      subplot(1,2,1)
      xlim([-10 10]) 
      ylim([-10 10])
      zlim([0,5])
      xlabel('x')
      ylabel('y')
      zlabel('z')
      
      subplot(1,2,2)
      xlim([-10 10]) 
      ylim([-10 10])
      zlim([0,5])
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
      
      subplot(1,2,1)
          
      if i>1
       delete(m11)
       delete(m21)
       delete(m31)
       delete(d11)
       delete(d21)
       delete(d31)
       delete(d41)
       delete(d51)
       delete(d61)
       delete(v11) 
      end
       
      subplot(1,2,2)
          
      if i>1
       delete(m12)
       delete(m22)
       delete(m32)
       delete(d12)
       delete(d22)
       delete(d32)
       delete(d42)
       delete(d52)
       delete(d62)
       delete(v12) 
      end
      
       subplot(1,2,1)
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
       
      %moving obstacles
      hold on 
      m11=surf(Xs+xi1+i*vx1,Ys+yi1+i*vy1,Zs);
      m21=surf(Xs+xi2+i*vx2,Ys+yi2+i*vy2,Zs);
      m31=surf(Xs+xi3+i*vx3,Ys+yi3+i*vy3,Zs);
      hold off
      
      view(-10,45)
      grid on 
      
      
      subplot(1,2,2)
      
      %representation drone
      d12=patch(xdronebase,ydronebase,zdronebase,'red');
      d22=patch(xdronewall1,ydronewall1,zdronewall1,'green');
      d32=patch(xdronewall2,ydronewall2,zdronewall2,'green');
      d42=patch(xdronewall3,ydronewall3,zdronewall3,[0.9100 0.4100 0.1700]);
      d52=patch(xdronewall4,ydronewall4,zdronewall4,[0.9100 0.4100 0.1700]);
      d62=patch(xdroneceil,ydroneceil,zdroneceil,'y');
      
      %velocity vector
      vscalefactor=7;
      v12=patch([x,x+xd/vscalefactor],[y,y+yd/vscalefactor],[z,z+zd/vscalefactor],'b');
       
      %moving obstacles
      hold on 
      m12=surf(Xs+xi1+i*vx1,Ys+yi1+i*vy1,Zs);
      m22=surf(Xs+xi2+i*vx2,Ys+yi2+i*vy2,Zs);
      m32=surf(Xs+xi3+i*vx3,Ys+yi3+i*vy3,Zs);
      hold off
      
      view(0,90)
      grid on 
      
      
      pause(velocityofreproduction)
end




