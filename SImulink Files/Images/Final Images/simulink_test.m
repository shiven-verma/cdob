
clc
%%% Common

omegader = 80; % Approximating x_dot LPF
alpha = 1.192; %4/Ts
a = 5; % Unknown System

delay_a = 0.01; % Actuator delay

%%%DSR
omega = 0.8; % LPF for DSR term
delay = 0.015; % DSR intentional delay
betac = 10; % DSr gain
c1 = 1;

%%% CDOB
gnet = 0.0; 
gnetset = 0:0.1:5;


mdl = 'DSR_delay_beta_test'; % Model Name

open_system(mdl) %Open the model

simIn = Simulink.SimulationInput(mdl);
simIn = setModelParameter(simIn,"StopTime","20");


MaxDef = zeros(length(gnetset),1);

for k = 1:1:length(gnetset)
    gnet = gnetset(k);
    out = sim(simIn,"UseFastRestart","off");
    disp(k)
    MaxDef(k) = max(out(1,1).yout{1}.Values.Data);
end
% out = sim(simIn,"UseFastRestart","off");






