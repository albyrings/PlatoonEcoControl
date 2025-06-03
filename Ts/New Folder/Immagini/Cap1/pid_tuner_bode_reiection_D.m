% MATLAB code for PID controller tuning for G(s) = 1/s using Bode criterion
% Using high bandwidth for disturbance rejection

% Design parameters
w_d = 2;           % Disturbance frequency (rad/s)
w_c = 20;          % Desired crossover frequency (rad/s) >> w_d
PM_desired = 60;   % Desired phase margin (degrees)

% 1. Define the integrator system
s = tf('s');
G = 1/s;

% 2. Analysis of the uncompensated open-loop system
figure(1);
bode(G);
grid on;
title('Bode Diagram of Uncompensated System G(s) = 1/s');

% 3. Design of lead compensator using Bode criterion
% to ensure high crossover frequency
alpha = 3;  % Ratio between zero and pole (alpha > 1 for lead compensator)

% Calculate T so that maximum phase contribution occurs at crossover frequency
T = 1/(w_c * sqrt(alpha));

% Calculate gain K to have |C(jw_c)G(jw_c)| = 1
mag_lead_at_wc = abs((1 + 1i*w_c*alpha*T)/(1 + 1i*w_c*T));
K = 1/(mag_lead_at_wc * (1/w_c));

% 3.1 Define the lead compensator as a filtered PID
C_lead = K * (1 + alpha*T*s)/(1 + T*s);

% 4. Analysis of the compensated open-loop system
L = C_lead * G;  % Open-loop transfer function

figure(2);
bode(G, 'b', C_lead, 'r', L, 'g');
grid on;
legend('G(s)', 'C(s)', 'L(s) = C(s)G(s)');
title('Bode Diagram of Compensated System with High Bandwidth');

% 5. Analysis of phase margin and crossover frequency
[Gm, Pm, Wcg, Wcp] = margin(L);
fprintf('Phase margin: %.2f degrees at frequency %.2f rad/s\n', Pm, Wcp);

% 6. Calculate sensitivity function
S = 1/(1 + L);

% 7. Verify disturbance attenuation at the specific frequency
w_range = logspace(-2, 2, 1000);
[mag_S, phase_S] = bode(S, w_range);

% Find the index corresponding to the disturbance frequency
[~, idx] = min(abs(w_range - w_d));

% 8. Calculate loop gain and sensitivity at disturbance frequency
[mag_L, ~] = bode(L, w_d);
[mag_S_at_wd, ~] = bode(S, w_d);

fprintf('\nDisturbance attenuation at %.2f rad/s:\n', w_d);
fprintf('Loop gain |L(j%.1f)| = %.2f = %.2f dB\n', w_d, mag_L, 20*log10(mag_L));
fprintf('Sensitivity |S(j%.1f)| = %.4f = %.2f dB\n', w_d, mag_S_at_wd, 20*log10(mag_S_at_wd));

% 9. Step response of the closed-loop system
T_cl = feedback(L, 1);  % Closed-loop system

figure(3);
step(T_cl);
grid on;
title('Step Response of the Closed-loop System');

% 10. Response to disturbance
% Create a sinusoidal disturbance at frequency w_d
t = 0:0.01:10;
d_t = sin(w_d*t);

figure(4);
lsim(S, d_t, t);
grid on;
title(['System Response to Sinusoidal Disturbance at ', num2str(w_d), ' rad/s']);
xlabel('Time (s)');
ylabel('Amplitude');

% 11. Bode diagram of sensitivity function
figure(5);
bodemag(S);
grid on;
hold on;
scatter(w_d, 20*log10(mag_S_at_wd), 100, 'r', 'filled');
text(w_d*1.2, 20*log10(mag_S_at_wd), ['Attenuation at \omega_d = ', num2str(w_d), ': ', num2str(20*log10(mag_S_at_wd)), ' dB']);
hold off;
title('Sensitivity Function S(j\omega)');
xlabel('Frequency (rad/s)');
ylabel('Magnitude (dB)');