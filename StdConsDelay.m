%%% Standard Consenus with similar system as DSR
%%% G(s) = a/(s+a)*s

clc
clear
close all

N = 6;
T = 10;
dt = 0.01;
gnet = 100;

L = [1 0 0 0 0 0;
    -1 2 -1 0 0 0;
    -1 -1 2 0 0 0;
    0 -1 0 1 0 0;
    0 0 -1 0 1 0;
    0 -1 -1 -1 -1 4]; % Change based on Network Configurations

L = [1 0 0 0 0 0;
    -1/2 2/2 -1/2 0 0 0;
    -1/2 -1/2 2/2 0 0 0;
    0 -1 0 1 0 0;
    0 0 -1 0 1 0;
    0 -1/4 -1/4 -1/4 -1/4 4/4]; % Change based on Network Configura

gamma = 1.69; % weight of the networks
a = 5.5; % Damping term



DynKp = -gamma^2*L;
K = DynKp;

B = L*ones(N,1);

%%% ACtuator Delay
tau_a = 0.1;
delay = tau_a;

%%%Positioning the robots
Y = [2 2.5 1.5 3 1 2]*2;
X = [2 1 1 0 0 0]*2 - 4;
X0 = [X,zeros(1,6)];

t = 0:dt:T; %linspace(0,T,T*10);
tspan = [0 T];
history = X0*0;
sol = dde23(@PyramidFormDer, tau_a, history,tspan,[],K,B,gamma,a);
Xsol = deval(sol,t);

Xvis = Xsol+X0';

figure(1)
for k=1:5:length(t)
    
    
   plot_config(Xvis(1:6,k),Y,N)
   hold on;
   disp(k)
    
    
    
   set(gcf,'color','w');
   hold off;
   pause(0.01)
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
grid on;
% xlim([0,10])
% ylim([0,12])

deformations1 = abs([Xsol(1,:)-Xsol(2,:),Xsol(1,:)-Xsol(2,:)]);
deformations2 = abs([Xsol(2,:)-Xsol(4,:),Xsol(2,:)-Xsol(5,:),Xsol(2,:)-Xsol(6,:)]);
MaxDef = max(max(deformations1),max(deformations2));
disp('Max deformation is:')
disp([max(deformations1),max(deformations2)])


function [StateDer] = PyramidFormDer(t,states,states_delay,A,B,alpha,a)

u = 10*alpha^2;
% u = 10*alpha^2*sin(t);

N = 6;
V = states(N+1:2*N);
V_delay = states_delay(N+1:2*N);
Ucons = A*states_delay(1:N) +  B*u;
xDamp = -a*(V-V_delay);
XDer = Ucons + xDamp;
StateDer = [V_delay;XDer];
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
