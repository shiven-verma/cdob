%%% Formation error and DSR plots

timestamps = [0.001, 0.0015, 0.002, 0.0025, 0.003, 0.004, 0.005, 0.006, 0.007, 0.008, 0.009, 0.01, 0.011, 0.012];

Cons_form_error = [0.6292 0.6313 0.6335 0.6357 0.6379 0.6423 0.6467 0.6512 0.6557 0.6602 0.6647 0.6694 0.6740 0.6787];
DSR_form_error = [0.2008 0.2024 0.2039 0.2055 0.2070 0.2101 0.2132 0.2163 0.2193 0.2224 0.2255 0.2285 0.2316 0.2347];
timestamps2 = [0.001,0.0015,0.002,0.0025];
DSR_20_error = [0.0232,0.0232,0.0233,0.0233]; 

plot(timestamps,Cons_form_error,'r')
hold on
plot(timestamps,DSR_form_error,'b')
hold on
plot(timestamps2,DSR_20_error,'b')
legend('Consensus','DSR \beta=2','DSR \beta=20')
title('Formation Error wrt delay')
xlabel('Time delay')
ylabel('Formation Error')
xlim([0.0009,0.013])
ylim([0,2.73])