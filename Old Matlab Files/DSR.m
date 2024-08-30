%Delay Self-Reinforcement DSR for cohesive decentralized control
close all
clc
clear 


N = 3;

T = 10;
time = linspace(0,T,100);
Y = [2 3 4];
X = [0 0 0];

L = [1 -1 0; -1 3 -1; 0 -1 1];
alpha = 1.5;

%%% Stability Parameters
betac = 10.5;
tau_d = 0.1;

param = [tau_d,alpha, N, betac];
%%% DDE solving
t = linspace(0,T,T*10);
X0 = [1;2;1];
history = [X0;zeros(3,1)];
sol = dde23(@DSR_derivative,tau_d,history,[0 t(end)],[],param,L);
% DSR_derivative(1,history,history,param,L)
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
plot(t,Xsol(1,:),'g')
hold on;
plot(t,Xsol(2,:),'r--')
hold on;
plot(t,Xsol(3,:),'b')




function Xdd = DSR_derivative(t,y,y_delay,param,L)

AXd = [0 0 0 1 0 0;
      0 0 0 0 1 0;
      0 0 0 0 0 1];


Xd0 = AXd*y;

N = param(3);
tau_d = param(1);
alpha = param(2);
betac = param(4);
input = alpha*t;

X_dot_td = y_delay(N+1:2*N);
% disp(y_delay)
B = L*ones(3,1);
formDist = [1;2;1];
Xd1 = (eye(N) - betac*L)*(y(N+1:2*N)-X_dot_td)/tau_d - alpha^2*betac*L*y(1:N)...
- 2*alpha*betac*L*y(N+1:2*N) + alpha*betac*B*input + alpha^2*betac*L*formDist; 

Xdd = [Xd0;Xd1];
disp(t)
end


function plot_anim(X,Y)



x1 = X - 0.2;
x2 = X + 0.2;

y1 = Y - 0.2;
y2 = Y + 0.2;

% disp(x1(1)-x2(1))
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