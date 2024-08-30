%%% Pyramid Shape DSR with Delay

%%% DSR W/O delay in Pyramid Configuration

clc
clear
close all

N = 6;
T = 10;

L = [1 0 0 0 0 0;
    -1 2 -1 0 0 0;
    -1 -1 2 0 0 0;
    0 -1 0 1 0 0;
    0 0 -1 0 1 0;
    0 -1 -1 -1 -1 4]; % Change based on Network Configurations

alpha = 5.5; % weight of the networks

%%% Stability Depends
betac = 10.5; % gain of laplacian
tau = 0.5;  % DSR delay

tau_a = 0.001; % Actuator Delay 
a = 5.5; % Damping

velK = [zeros(N) eye(N)];

DynKp = -alpha^2*L;
DynKd = -2*alpha*L;
K = [velK;DynKp,DynKd];

Bd = L*ones(N,1);
B = [zeros(N,1);Bd];



Y = [2 2.5 1.5 3 1 2]*2;
X = [2 1 1 0 0 0]*2;
X0 = [X,zeros(1,6)];

param = [alpha,betac,N,tau];
t = linspace(0,T,T*10);
tspan = [0 T];
history = X0;

sol = dde23(@PyramidFormDer,[tau_a,tau+tau_a],history,tspan,[],param,K,L,a,X');
Xsol1 = deval(sol,t);

Xsol = Xsol1-X0';




figure(1)
for k=1:1:T*10
    
    
   plot_config(Xsol1(1:6,k),Y,N)
  
    
    
    
   set(gcf,'color','w');

   pause(0.01)
end


figure(2)
plot(t,Xsol(1,:),'g')
hold on;
plot(t,Xsol(2,:),'b-.')
hold on;
plot(t,Xsol(3,:),'r-.')
hold on;
plot(t,Xsol(4,:),'k--')
hold on;
plot(t,Xsol(5,:),'c--')
hold on;
plot(t,Xsol(6,:),'m--')
hold on;




function [StateDer] = PyramidFormDer(t,states,delayed_states,param,K,L,a,initial_pos)

%%% States is 12x1 vector, A is 12x12 matrix, the upper half 6x12 is just
%%% for the velocity



alpha = param(1);
betac = param(2);
N = param(3);
tau = param(4);


input = 7*alpha;
% input = 4*alpha*sin(t);

Xd0 = K(1:N,:)*states;             % Velocity 6x1


X_dot_ta = delayed_states(N+1:2*N,1);
% disp(delayed_states)
B = L*ones(6,1);

Udsr_td = (eye(N) - betac*L)*(delayed_states(N+1:2*N,1)-delayed_states(N+1:2*N,2))/tau ...
                - alpha^2*betac*L*delayed_states(1:N,1) - 2*alpha*betac*L*delayed_states(N+1:2*N,1) + alpha*betac*B*input + ...
            alpha^2*betac*L*initial_pos(1:N);
            
Xd1 = -a*(states(N+1:2*N)-X_dot_ta) + Udsr_td;
StateDer = [Xd0;Xd1];
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
ylim([0 12]);

hold off

end








