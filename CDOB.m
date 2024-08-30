%%% CDOB and Time Delay Compensation function
%%% Inputs would be output of second order function and Input to the system
%%% Output would be the feedback to the controller

System = tf([0 1],[1 0 1]);
DelayedSystem = tf([0 1],[1 0 1],'InputDelay',0.1);
Acceleration = tf(1,[1 0]);
DelayedOutput = DelayedSystem*Acceleration;

CDOBfun(Acceleration,DelayedOutput,System)

function Xt = CDOBfun(Acceleration,DelayedOutput,System)

s = tf('s');
gnet = 40;            % filter gain
OutDer = DelayedOutput*s*gnet; 

DistSig = Acceleration+OutDer;
LPF = gnet/(s+gnet); 

D_estimate = LPF*DistSig;  % D_hat = gnet*(U(t)-U(t-tau))/(gnet+s)

DistOut = System*D_estimate; % X(t) - X(t-tau)
Xt = DistOut + DelayedOutput; % Feedback

end