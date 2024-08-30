%%% Pyramid Shape DSR with Delay

%%% DSR W/O delay in Pyramid Configuration

clc
% clear
% close all

N = 6;
T = 15;

L = [1 0 0 0 0 0;
    -1 2 -1 0 0 0;
    -1 -1 2 0 0 0;
    0 -1 0 1 0 0;
    0 0 -1 0 1 0;
    0 -1 -1 -1 -1 4]; % Change based on Network Configurations

alpha = 1.195; % weight of the networks

%%% Stability Depends
betac = 2; % gain of laplacian
tau = 0.5;  % DSR delay

tau_a = 0.0035; % Actuator Delay 
a = 5; % Damping

velK = [zeros(N) eye(N)];

DynKp = -alpha^2*L;
DynKd = -2*alpha*L;
K = [velK;DynKp,DynKd];

Bd = L*ones(N,1);
B = [zeros(N,1);Bd];



Y = [2 2.5 1.5 3 1 2]*2;
X = [2 1 1 0 0 0]*2;
X0 = [X,zeros(1,2*N)];
omega = 200;

param = [alpha,betac,N,tau,omega];
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

deformations1 = [Xsol(1,:)-Xsol(2,:),Xsol(1,:)-Xsol(2,:)];
deformations2 = [Xsol(2,:)-Xsol(4,:),Xsol(2,:)-Xsol(5,:),Xsol(2,:)-Xsol(6,:)];
MaxDef = max(max(deformations1),max(deformations2));
disp('Max deformation is:')
disp(MaxDef)


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
grid on;
xlim([0,15])
ylim([0,1.2])



function [StateDer] = PyramidFormDer(t,states,delayed_states,param,K,L,a,initial_pos)

%%% States is 12x1 vector, A is 12x12 matrix, the upper half 6x12 is just
%%% for the velocity



alpha = param(1);
betac = param(2);
N = param(3);
tau = param(4);
omega = param(5);


input = 1*alpha;
% input = 4*alpha*sin(t);

Xd0 = K(1:N,:)*states(1:2*N);             % Velocity 6x1


X_dot_ta = delayed_states(N+1:2*N,1);


B = L*ones(6,1);


Vdot_f = states(2*N+1:3*N);
Vdiff = (delayed_states(N+1:2*N,1)-delayed_states(N+1:2*N,2))/tau;
dotVdot_f = -omega*Vdot_f + omega*Vdiff;


Udsr_td = (eye(N) - betac*L)*Vdot_f ...
                - alpha^2*betac*L*delayed_states(1:N,1) - 2*alpha*betac*L*delayed_states(N+1:2*N,1) + alpha*betac*B*input + ...
            alpha^2*betac*L*initial_pos(1:N);
            
Xd1 = -a*(states(N+1:2*N)-X_dot_ta) + Udsr_td;

StateDer = [Xd0;Xd1;dotVdot_f];
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








