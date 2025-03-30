clear;
clc;
close all;

% Parametri di simulazione
t_span = [0 1000];        % 10 secondi di simulazione
x0 = [0; 0; 0];         % Condizioni iniziali: [posizione; velocità; errore integrale]
setpoint = 1;           % Posizione target

% Parametri PID
Kp = 7;                 % Proporzionale
Ki = 0;                 % Integrale
Kd = 0.1;                 % Derivativo

% Simulazione
[t, x] = ode45(@(t, x) double_integrator_pid(t, x, setpoint, Kp, Ki, Kd), t_span, x0);

% Estrazione risultati
position = x(:, 1);
velocity = x(:, 2);
integral = x(:, 3);

% Calcolo dell'azione di controllo u per il plot
u = zeros(size(t));
for i = 1:length(t)
    error = setpoint - position(i);
    if i > 1
        dt = t(i) - t(i-1);
        error_derivative = (error - (setpoint - position(i-1))) / dt;
    else
        error_derivative = 0;
    end
    u(i) = Kp * error + Ki * integral(i) + Kd * error_derivative;
end

% Visualizzazione
figure;
plot(t, position, 'b-', t, setpoint*ones(size(t)), 'r--');
xlabel('Tempo [s]');
ylabel('Posizione');
title(['Controllo PID di G(s) = 1/s^2 (Kp=', num2str(Kp), ', Ki=', num2str(Ki), ', Kd=', num2str(Kd), ')']);
legend('Posizione', 'Setpoint');
grid on;



% Funzione per simulare il sistema a doppio integratore con PID
function dxdt = double_integrator_pid(t, x, setpoint, Kp, Ki, Kd)
    % Stati: x(1) = posizione, x(2) = velocità, x(3) = errore integrale
    position = x(1);
    velocity = x(2);
    error_integral = x(3);
    
    % Calcolo errore
    error = setpoint - position;
    
    % Calcolo dt
    persistent last_t;
    if isempty(last_t), last_t = t; end
    dt = t - last_t;
    if dt <= 0, dt = 0.001; end  % Evita dt = 0
    last_t = t;
    
    % Derivata dell'errore = -velocità per questo specifico sistema
    error_derivative = -velocity;
    
    % Azione di controllo PID
    u = Kp * error + Ki * error_integral + Kd * error_derivative;
    
    % Equazioni del sistema - G(s) = 1/(s^2)
    dposition_dt = velocity;          % dx1/dt = x2
    dvelocity_dt = u;                 % dx2/dt = u
    derror_integral_dt = error;       % Integrazione dell'errore
    
    dxdt = [dposition_dt; dvelocity_dt; derror_integral_dt];
end