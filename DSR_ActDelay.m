%%% DSR with actuator delay, which is Udsr(t-taud)
%Delay Self-Reinforcement DSR for cohesive decentralized control
close all
clear 
clc

N = 3;

T = 10;
time = linspace(0,T,100);
Y = [2 3 4];
X = [0 0 0];

L = [1 -1 0; -1 3 -1; 0 -1 1];
alpha = 2;

%%% Stability Parameters
betac = 5;
tau_a = 0.002; % Actuator Delay

tau = 0.5; % Approximation delay
a = 20;  % Damping

param = [tau,alpha,N, betac,a];
%%% DDE solving
t = linspace(0,T,T*10);
history = zeros(2*N,1);
sol = dde23(@DSR_derivative,[tau_a,tau + tau_a],history,[0 t(end)],[],param,L);
% DSR_derivative(1,history,history,param,L)
Xsol = deval(sol,t);

if (1)
    vidObj = VideoWriter('Simul_ObjectTransport8.avi');
    open(vidObj);
end
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
   
   
    if (1)
        F=getframe(gcf);
        writeVideo(vidObj,F);
    end
end

if (1)
    close(vidObj);
end

figure(2)
hold off
plot(t,Xsol(1,:),'g')
hold on;
plot(t,Xsol(2,:),'r--')
hold on;
plot(t,Xsol(3,:),'b')
hold on;
plot(t,(3*sin(2*t)));
hold off


figure(3)
plot(t,Xsol(1,:)-Xsol(2,:))
hold on;
plot(t,-Xsol(2,:)+Xsol(3,:))
hold on;



function Xdd = DSR_derivative(t,y,y_delay,param,L)

AXd = [0 0 0 1 0 0;
      0 0 0 0 1 0;
      0 0 0 0 0 1];


Xd0 = AXd*y_delay(:,1);

N = param(3);
tau = param(1);
alpha = param(2);
betac = param(4);
a = param(5);
input = alpha*(3*sin(2*t));

X_dot_ta = y_delay(N+1:2*N,1);
% disp(y_delay)
B = L*ones(3,1);

Udsr_td = (eye(N) - betac*L)*(y_delay(N+1:2*N,1)-y_delay(N+1:2*N,2))/tau ...
                - alpha^2*betac*L*y_delay(1:N,1) - 2*alpha*betac*L*y_delay(N+1:2*N,1) + alpha*betac*B*input; 
            
Xd1 = -a*(y(N+1:2*N)-X_dot_ta) + Udsr_td;
Xdd = [Xd0;Xd1];
% disp(Xd1)
% disp(t)
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


xlim([-5 5]);
ylim([0 6.5]);


end