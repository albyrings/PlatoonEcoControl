%% Bode Comparison – PID with/without Filter vs Pure P
% Generates a double-panel Bode diagram that compares:
%   • PID with derivative filter
%   • PID without derivative filter
%   • Pure proportional controller
% The plot is saved as “control_strategy_comparison.png”.

clear;  close all;  clc;

s = tf('s');
G = 1/s;                           % Integrator plant

%% Controller definitions -------------------------------------------------
% Pure proportional controller (tuned for crossover ≈ 10 rad/s)
Kp_P  = 10;
C_P   = Kp_P;                      % P-only controller
L_P   = C_P*G;

% Base PID parameters
Kp    = 8;
Ti    = 1;                         % integral time [s]
Td    = 0.1;                       % derivative time [s]

% PID without derivative filter
C_PID_unfilt = Kp*(1 + 1/(Ti*s) + Td*s);
L_PID_unfilt = C_PID_unfilt*G;

% PID with derivative low-pass filter (1st-order, factor N)
N     = 20;                        % filter coefficient (≈ 1/Tf)
C_PID_filt   = Kp*(1 + 1/(Ti*s) + (Td*s)/(1 + Td/N*s));
L_PID_filt   = C_PID_filt*G;

%% Bode plot --------------------------------------------------------------
opts        = bodeoptions('cstprefs');
opts.Grid   = 'on';
opts.PhaseWrapping = 'on';

figure;
bodeplot(L_PID_filt,  'b', ...
         L_PID_unfilt,'r', ...
         L_P,         'g', ...
         opts, {1e-1, 1e3});
legend({'PID with filter', 'PID without filter', 'Pure P'}, ...
       'Location', 'SouthWest');
title('Comparison of Control Strategies');


