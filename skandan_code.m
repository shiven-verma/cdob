%%%%%Code for simulating input delayed systems%%%%%%%%%%%%%
%for Skandhan
%%%%%%%%%%%% April 12 2024 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc
close all
nfig = 0;
to_save_video = 0;
global rdx
rdx = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts = 50;%7.5
beta = 57.5;     %dsr gain

tend = 100;      %seconds

tau_d = 0.05;    %dsr delay for taking delay based derivative

%input delay dynamics - from actuators
tau = 0.000000000000000000000282;
adamp = 8;
%return
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%network definition%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N = 5;%4;

% K = [1 -1 0 0;
%     -1 2 -1 0;
%     0 -1 2 -1;
%     0 0 -1 2];
K= [ 1 -1  0  0  0;
    -1  2  -1  0  0;
    0  -1  3  -1  0;
    0  0  -1  2  -1;
    0  0  0  -1  1];


%B = [0; 0; 0; 1];
B = [0; 0; 1; 0; 0];
eig(K);
lambda_K = eig(K);
% return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%simulating the response 
tsteps = 12000;
t = linspace(0, tend, tsteps);

%settling times for comparison of DSR and noDSR
alpha = 0.10*4/Ts;
w_R = 3*alpha;
alpha_no_dsr = 0.5*(4/Ts)*sqrt(beta); 

%getting polynomial coeffs C(s) = (s+alpha)^2 = s^2 + 2 alpha s + alpha^2
ahat0 = alpha^2;
ahat1 = 2*alpha;
ahat0_nodsr = alpha_no_dsr^2;
ahat1_nodsr = 2*alpha_no_dsr; 

Nsim = (N)*(4+1); %4*N are for position and velocity states with the No-DSR and DSR approaches

history = zeros(Nsim,1);
c=linspace(1,tsteps,tsteps);
vd = 1;

Param_dde = [alpha tau beta N alpha_no_dsr tau_d vd adamp w_R];
D = zeros(4, 2000);
sol = dde23(@ddefun,[tau (tau+tau_d)],history,[0, t(end)],[],K, B, Param_dde, D, c);
Ydsr = deval(sol,t);

X_nodsr = Ydsr(1:N,:);
V_nodsr = Ydsr(N+1:2*N,:);
X_dsr = Ydsr(2*N+1:3*N,:);
V_dsr = Ydsr(3*N+1:4*N,:);




% sol2 = dde23(@ddefun,[tau (tau+0.0000002)],history,[0, t(end)],[D],K, B, Param_dde);
% Ydsr2 = deval(sol2,t);
% X_dsr = Ydsr2(2*N+1:3*N,:);
% V_dsr = Ydsr2(3*N+1:4*N,:);

nfig=nfig+1; figure(nfig);
plot(nan, nan, 'c:', nan, nan, 'b-', nan, nan, 'r-', nan, nan, 'g-', 'MarkerSize', 3, 'LineWidth', 3);
hold on
plot(t, vd*ones(size(t)), 'c:', 'MarkerSize', 3, 'LineWidth', 3)
hold on
plot(t, V_dsr, 'b-', 'MarkerSize', 3, 'LineWidth', 3)
% hold on
% plot(t, V_dsr, 'r-', 'MarkerSize', 3, 'LineWidth', 3)
% xlabel('time (s)')
ylabel('Network Step Response')
% legend('x_s', 'Consensus', 'DSR', 'Location', 'southeast')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
if (to_save_video)
    vidObj = VideoWriter('Simul_ObjectTransport8.avi');
    open(vidObj);
end
nfig=nfig+1; 
figure(nfig);
for k=500:10:1000

    plot_visualization(7*V_nodsr(:,k), 0);
    set(gcf,'color','w');
    hold off
    pause(0.1);
    
    if (to_save_video)
        F=getframe(gcf);
        writeVideo(vidObj,F);
    end
end

if (to_save_video)
    close(vidObj);
end

function plot_visualization(Y_i, case_)

    d_x = 1.5;
    
    rod_x = [];
    
    for i=1:1:length(Y_i)
        %bigger block
        x1_1=0.3+(i-1)*d_x;
        x2_1=.7+(i-1)*d_x;
        y1_1=0+Y_i(i);
        y2_1=.5+Y_i(i);
        x_1 = [y1_1, y1_1, y2_1, y2_1, y1_1];
        y_1 = [x1_1, x2_1, x2_1, x1_1, x1_1]; 
        %smaller block
        x1_2=0.4+(i-1)*d_x;
        x2_2=.6+(i-1)*d_x;
        y1_2=0.5+Y_i(i);
        y2_2=0.65+Y_i(i);
        x_2 = [y1_2, y1_2, y2_2, y2_2, y1_2];
        y_2 = [x1_2, x2_2, x2_2, x1_2, x1_2]; 
        if (case_ == 0)
            plot(x_1, y_1, 'b-', 'LineWidth', 5);
        elseif (case_==1)
            plot(x_1, y_1, 'y-', 'LineWidth', 5);
        else
            plot(x_1, y_1, 'r-', 'LineWidth', 5);
        end

        hold on
        plot(x_2, y_2, 'k-', 'LineWidth', 4);
        
    end

    rod_x = [0.5:d_x:(0.5+(length(Y_i)-1)*d_x)];
  
   
    plot(0.6*ones(size(rod_x))+Y_i',rod_x, 'k-', 'LineWidth',3)
    
    xlim([6 18]);
    ylim([0 7.5]);
    xticks([]); yticks([]);

end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dydt = ddefun(t, y, ydel, K, B, Param, D, c)
            % Differential equation function
            t;%parameters
            alpha = Param(1); 
            tau = Param(2); 
            beta = Param(3); 
            N = Param(4); 
            alpha_no_dsr = Param(5); 
            tau_d = Param(6);
            vd = Param(7);
            a = Param(8);
            i=floor(t/120*2000)+1;
            Omega = Param(9);
            v_d = vd;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %getting the no DSR states from y and no DSR delayed states
            %from ydel 
            X_nodsr = y(1:N,1); V_nodsr = y(N+1:2*N,1);
            X_nodsr_tau = ydel(1:N,1); V_nodsr_tau = ydel(N+1:2*N,1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %getting the DSR states from y and DSR delayed states
            %from ydel 
            X_dsr = y(2*N+1:3*N,1); V_dsr = y(3*N+1:4*N,1);
            X_dsr_tau = ydel(2*N+1:3*N,1); V_dsr_tau = ydel(3*N+1:4*N,1);
            X_dsr_taud = ydel(2*N+1:3*N,2); V_dsr_taud = ydel(3*N+1:4*N,2);
            
            Vdot_f = y(4*N+1:5*N,1);
            
            
            %using a filtered approach to obtain the derivative on RHS of
            %DSR update law
            dot_Vdot_f = -Omega*Vdot_f + Omega*(V_dsr_tau-V_dsr_taud)/tau_d;
            disp(size(dot_Vdot_f))
            %%%%%%DYNAMICS - State Equations
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % no DSR
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            dot_Xnodsr = V_nodsr;
            %dot_Vnodsr =  (-alpha_no_dsr*K*V_nodsr + alpha_no_dsr*B*v_d  + a*(V_nodsr) ) - a*(V_nodsr); %no input delay
            dot_Vnodsr =  (-alpha_no_dsr*K*V_nodsr_tau+alpha_no_dsr*[1;1;1;1;1]*vd);
            %dot_Vnodsr =(-alpha_no_dsr*K*V_nodsr_tau - alpha_no_dsr*B*v_d );% + a*(V_nodsr_tau) ); - a*(V_nodsr); %added
            %input delay
            

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %DSR
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            dot_Xdsr = eye(N)*V_dsr;
%              dot_Vdsr = (eye(N)-beta*K)*Vdot_f ...
%                  -alpha*beta*K*V_dsr + alpha*beta*B*v_d - a*(V_dsr-V_dsr);   %no input delay
            dot_Vdsr = (eye(N)-beta*K)*Vdot_f ...
                -alpha*beta*K*V_dsr_tau + alpha*beta*B*v_d - a*(V_dsr-V_dsr_tau); 
            %added input delay
            % V_dsr_tau;
            % dot_Vdsr = -K*V_dsr_tau;  %added input delay
            
           %  D1 = dot_Vdsr+10*V_dsr;
           % 
           %  a1 = 10*exp(-10*t);
           %  D(:,i) = a1*D1;
           %  D4 = D(:,i)-10*V_dsr;
           %  C = D4 - Vdot_f/a;
           %  C = C+V_dsr;
           
           
           dot_Vdsr =  (eye(N)*(1+a*tau_d)-beta*K)*Vdot_f ...
           -alpha*beta*K*V_dsr_tau + alpha*beta*(B*v_d) - a*(V_dsr-V_dsr_tau);

            
            %%%%%%%%%%%
            %collating the velocity and position derivatives into single
            %column vectors for the two approaches
            dot_XV_nodsr = [dot_Xnodsr; dot_Vnodsr];
            dot_XV_dsr = [dot_Xdsr; dot_Vdsr];
           
            dydt = [dot_XV_nodsr; dot_XV_dsr; dot_Vdot_f];
end % ddex4de

 
 
