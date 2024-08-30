%%% Pyramid formation with directed and Undirected graphs and do Con and
%%% DSR with that

clc
clear
close all

N = 6;
T = 5;
dt = 0.01;

L = [1 0 0 0 0 0;
    -1 2 -1 0 0 0;
    -1 -1 2 0 0 0;
    0 -1 0 1 0 0;
    0 0 -1 0 1 0;
    0 -1 -1 -1 -1 4]; % Change based on Network Configurations

alpha = 8.5; % weight of the networks

velK = [zeros(N) eye(N)];

DynKp = -alpha^2*L;
DynKd = -2*alpha*L;
K = [velK;DynKp,DynKd];

Bd = L*ones(N,1);
B = [zeros(N,1);Bd];

C = [eye(N) zeros(N)];

D = zeros(N,1);

Y = [2 2.5 1.5 3 1 2]*2;
X = [2 1 1 0 0 0]*2 - 4;
X0 = [X,zeros(1,6)];

t = 0:dt:T; %linspace(0,T,T*10);
tspan = [0 T];

sol = ode45(@(t,y) PyramidFormDer(t,y,K,B,alpha),tspan,X0*0);
Xsol = deval(sol,t);

Xvis = Xsol+X0';

figure(1)
for k=1:10:length(t)
    
    
   plot_config(Xvis(1:6,k),Y,N)
   hold on;
   disp(k)
    
    
    
   set(gcf,'color','w');
   hold off;
   pause(0.1)
end


figure(2)
plot(t,Xsol(1,:),'g')
hold on;
plot(t,Xsol(2,:),'b-.')
hold on;
plot(t,Xsol(3,:),'r-.')
hold on;
plot(t,Xsol(4,:),'y--')
hold on;
plot(t,Xsol(5,:),'c--')
hold on;
plot(t,Xsol(6,:),'m--')
hold on;
plot(t,10*heaviside(t))
xlim([0,4])
ylim([0,11])


function [StateDer] = PyramidFormDer(t,states,A,B,alpha)

u = 10*alpha^2;
% u = 10*alpha^2*sin(t);
formDist = [zeros(6,1);[0;1;1;2;2;2]*2];
XDer = A*states +  B*u;% - alpha^2*formDist;
N = 6;
StateDer = XDer;
disp(t)
end



function plot_config(X,Y,N)

x1 = X - 0.3;
x2 = X + 0.3;

y1 = Y - 0.3;
y2 = Y + 0.3;

for i=1:1:N
   
    plot([x1(i),x1(i),x2(i),x2(i),x1(i)],[y1(i),y2(i),y2(i),y1(i),y1(i)],'b','LineWidth',5);
    hold on;
end
% disp(X([1,3]))

plot(X([1,2]),Y([1,2]),'k','LineWidth',0.5)
plot(X([1,3]),Y([1,3]),'k','LineWidth',0.5)
plot(X([2,3]),Y([2,3]),'r--','LineWidth',0.5)
plot(X([2,4]),Y([2,4]),'k','LineWidth',0.5)
plot(X([3,5]),Y([3,5]),'k','LineWidth',0.5)
plot(X([5,6]),Y([5,6]),'k','LineWidth',0.5)
plot(X([4,6]),Y([4,6]),'k','LineWidth',0.5)
plot(X([3,6]),Y([3,6]),'k','LineWidth',0.5)
plot(X([2,6]),Y([2,6]),'k','LineWidth',0.5)



xlim([-10 15]);
ylim([0 10]);

hold off

end
