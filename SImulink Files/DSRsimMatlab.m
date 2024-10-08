clc


var = 1;
outdata = dsr2;

if var==1
%     mdl = 'STDConsCDOB';
    mdl = 'DSRCDOB';

    open_system(mdl)

    simIn = Simulink.SimulationInput(mdl);

    simIn = setModelParameter(simIn,"StopTime","15");

    out = sim(mdl);
%     Simulink.sdi.plot(out);
    
end

var = 0;
if var == 0
    % Define the range of x values
t1 = out.yout{1}.Values.Time;
t2 = out.yout{2}.Values.Time;
t3 = out.yout{3}.Values.Time;
t4 = out.yout{4}.Values.Time;
t5 = out.yout{5}.Values.Time;
t6 = out.yout{6}.Values.Time;

y1 = out.yout{1}.Values.Data;
y2 = out.yout{2}.Values.Data;
y3 = out.yout{3}.Values.Data;
y4 = out.yout{4}.Values.Data;
y5 = out.yout{5}.Values.Data;
y6 = out.yout{6}.Values.Data;


% Calculate the sine and cosine of x

% Create a new figure with a specified size
figure('Position', [100, 100, 260, 570]); % [left, bottom, width, height]

% Plot the sine wave
% plot(t1,y1,t2,y2,t3,y3,t4,y4,t5,y5,t6,y6)
plot(t1, y1, 'r', 'LineWidth', 2); % 'r-' specifies a red solid line
hold on;
plot(t2, y2, 'b', 'LineWidth', 2);
hold on;
plot(t3, y3, 'y', 'LineWidth', 2);
hold on;
plot(t4, y4, 'g', 'LineWidth', 2);
hold on;
plot(t5, y5, 'm', 'LineWidth', 2);
hold on;
plot(t6, y6, 'c', 'LineWidth', 2);

% Hold on to the current plot so we can add another plot to it


% Plot the cosine wave
% plot(x, y2, 'b--', 'LineWidth', 2); % 'b--' specifies a blue dashed line
ylim([0,1.2])
% Add title and labels
title('DSR (\beta = 2)');
% title('Without DSR')
xlabel('time (s)');
ylabel('Network Response (X(t))');

% Add a legend

% Display grid lines
grid on;

% Release the hold on the current plot
hold off;
end



