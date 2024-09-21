clc
K = [1 0 0 0 0 0;
    -1 2 -1 0 0 0;
    -1 -1 2 0 0 0;
    0 -1 0 1 0 0;
    0 0 -1 0 1 0;
    0 -1 -1 -1 -1 4];

beta1 = 20;
taua = 0.017;


%%%%%%%%%%
tau = 0.01;
%%%%%%%%%%
alpha = 1.195;
a = 0; % Damping term

lambda = 1; % 4,3
omega = 20;
w = 0.9;



omegalist = 0:0.1:50;
magCs = zeros(length(omegalist));
magGst = zeros(length(omegalist));

for l=1:1:length(omegalist)
    omega = omegalist(l);
    w = 0.09;
    fs = w/(1i*omega+w);
    Fs = (fs*(1-exp(-1i*tau*omega))/tau)^2;
    Cs = (1i*omega)^2 - (1-lambda*beta1)*Fs + beta1*lambda*(alpha^2 + 2*alpha*1i*omega);
    gst = (exp(-taua*1i*omega)-1)*beta1*lambda*(Fs + alpha^2 + (2*alpha+a)*1i*omega);
    
    magCs(l) = abs(Cs);
    magGst(l) = abs(gst);
    
end

plot(omegalist,magCs);
hold on
plot(omegalist,magGst)
hold off
% legend('Cs','gst')













% 
% taualist = 0:0.01:2;
% betalist = 2:0.5:25;
% dmest = [];
% 
% for k=1:1:length(betalist)
%     beta1 = betalist(k);
%     for i=1:1:length(taualist)
%        taua = taualist(i);
%        Cs = (1i*omega)^2 - (1-lambda*beta1)*Fs + beta1*lambda*(alpha^2 + 2*alpha*1i*omega);
%        gst = (exp(-taua*1i*omega)-1)*beta1*lambda*(Fs + alpha^2 + (2*alpha+a)*1i*omega);
%        disp(abs(gst))
%        disp(abs(Cs));
%        
%         if abs(Cs)<=abs(gst)
%             disp(i);
%             disp(taua);
%             dmest = [dmest,taua];
%             break 
%         end
%        
%     end
% end
% 
% plot(betalist,dmest)







