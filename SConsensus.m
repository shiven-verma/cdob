%Standard Consensus based control of three agents
% P-Consensus without delay

N = 3;
T = 30;
time = linspace(0,T,100);
Y = [2 3 4];
X = [0 0 0];

L = [1 -1 0; -1 3 -1; 0 -1 1];
a = 12.5;

K = [0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1;
    -a^2 a^2 0 -2*a 2*a 0;
    a^2 -3*a^2 a^2 2*a -6*a 2*a;
    0 a^2 -a^2 0 2*a -2*a];

B = [0;
    0;
    0;
    0;
    1;
    0];

C = [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0];

D = zeros(3,1);

% [Den,Num] = ss2tf(K,B,C,D);

t = linspace(0,T,T*10);
xd = heaviside(t);

sys = ss(K,B,C,D);


u1 = a^2*(sin(t)-2);
u2 = 2*a*cos(t);
u0 = 2*(u1+u2);
u = 5*a^2;

tspan = [0 T];
X0 = [0 0 0 0 0 0 u];

sol = ode45(@(t,y) MultiSys(t,y,K,B,a),tspan,X0);
Xsol = deval(sol,t);




figure(1)
for k=1:1:T*10
   disp(k)
   
%    dt = time(2)-time(1);
%    dX = [3 2.5 3]/15;
   plot_anim(Xsol(1:3,k),Y)
   hold on;
%    X = X + dX*dt;
   
   set(gcf,'color','w');
   hold off
   pause(0.001);
end

figure(2)
plot(t,Xsol(1,:))
hold on;
plot(t,Xsol(2,:))
hold on;
plot(t,Xsol(3,:))
hold on;

figure(3)
plot(t,Xsol(1,:)-Xsol(2,:))
hold on;
plot(t,-Xsol(2,:)+Xsol(3,:))
hold on;


function [stateDer] = MultiSys(t,states,K,B,a)
u = states(7);
xstate = states(1:6);
formDist = [0;0;0;0.5;0;0.5];
xD = K*xstate + B*u - a^2*formDist;
uD = -5*sin(t);
stateDer = [xD; uD];
end



function plot_anim(X,Y)

x1 = X - 0.2;
x2 = X + 0.2;

y1 = Y - 0.2;
y2 = Y + 0.2;

disp(x1(1)-x2(1))
plot([x1(1),x1(1),x2(1),x2(1),x1(1)],[y1(1),y2(1),y2(1),y1(1),y1(1)],'r','LineWidth',5);
hold on;
plot([x1(2),x1(2),x2(2),x2(2),x1(2)],[y1(2),y2(2),y2(2),y1(2),y1(2)],'y','LineWidth',5);
hold on;
plot([x1(3),x1(3),x2(3),x2(3),x1(3)],[y1(3),y2(3),y2(3),y1(3),y1(3)],'b','LineWidth',5);
hold on;

plot(X,Y,'k','LineWidth',0.5)


xlim([0 10]);
ylim([0 6.5]);


end
