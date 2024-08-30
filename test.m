%%% Test for lowpass
clc;
syms w tau s

% F = -w/(w+s)*exp(-tau*s);
F = w/(w+s);
f = ilaplace(F);
