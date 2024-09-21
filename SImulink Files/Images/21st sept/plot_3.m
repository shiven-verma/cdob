clc


%%% Common

omegader = 40;
alpha = 1.192;
a = 0.5;
omega = 5;
delay_a = 0.01;

%%% DSR specific
delay = 0.02;
betac = 5;
betac1 = 3;
c1 = 1;
betacd = 3;

%%% CDOB
gnet = 0;%0.005;

% 
% %%% Simulation Through Script
% mdl = 'DSRCDOB_cohesion';
% 
% open_system(mdl)
% 
% % delays = [0.001,0.005,0.009,0.01,0.012,0.014,0.016,0.018,0.02,0.022,0.023,0.025,0.027,0.03,0.05,0.06,0.07,0.08,0.09,0.1,0.15,0.19,0.25,0.3,0.35,0.4];
% delays = 0.001:0.01:0.09;
% 
% simIn = Simulink.SimulationInput(mdl);
% 
% simIn = setModelParameter(simIn,"StopTime","20");
% 
% blk1 = mdl + "/TD1";
% blk2 = mdl + "/TD2";
% blk3 = mdl + "/TD3";
% blk4 = mdl + "/TD4";
% blk5 = mdl + "/TD5";
% blk6 = mdl + "/TD6";
% 
% delay1 = get_param(blk1,"DelayTime");
% % delay2 = str2double(get_param(blk2,"DelayTime"));
% % delay3 = str2double(get_param(blk3,"DelayTime"));
% % delay4 = str2double(get_param(blk4,"DelayTime"));
% % delay5 = str2double(get_param(blk5,"DelayTime"));
% % delay6 = str2double(get_param(blk6,"DelayTime"));
% 
% disp(delay1)
% 
% set_param(blk1,"DelayTime",delay1)
% set_param(blk2,"DelayTime",delay1)
% set_param(blk3,"DelayTime",delay1)
% set_param(blk4,"DelayTime",delay1)
% set_param(blk5,"DelayTime",delay1)
% set_param(blk6,"DelayTime",delay1)
% 
% 
% 
% for k = length(delays):-1:1
%     simIn(k) = Simulink.SimulationInput(mdl);
%     simIn(k) = setVariable(simIn(k),delay1,delays(k));
% %     simIn(k) = setVariable(simIn(k),"freq",freqs(k));
% end
% 
% out = sim(simIn,"UseFastRestart","off");
% 
% MaxDefCD15 = zeros(length(delays),1);
% for i=1:1:length(delays)
%     MaxDefCD15(i) = out(1,i).yout{1}.Values.Data(end);
% end
% 
% % disp(isnan(MaxDef))
% 
% loglog(delays,MaxDefCD15)
% ylim([0.015,0.25])
% yticks([0.02,0.03,0.04,0.05,0.0673,0.08,0.1,0.16,0.2])
% grid on
% xlabel("delays")
% ylabel("MaxDeform")
% % xlim([0.001,0.4])
% % ylim([0,0.4])
% 
% % simIn = setBlockParameter(simIn,blk1,"DelayTime","0.05");
% % 
% % out = sim(simIn);
