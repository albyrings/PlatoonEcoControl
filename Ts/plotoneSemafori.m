clear;
clc;
close all;

% Parametri dei veicoli
n_vehicles = 3; % Numero di veicoli

m = 1000 * ones(1, n_vehicles); 
b1 = 450;
b2 = 450;
b3 = 1;
b4 = 3;

% Massa uguale per tutti i veicoli
delta = @(t) 0 * (b1 + b2*(sin((1/b3)*t + b4) + 0.25*rand)); % Forza esterna (N)

% Distanza target
d_init = 4;
d_min = 1; 

% Parametri PID per il leader (controllo velocità)
K_p_speed = 7000;     
K_i_speed = 0;
K_d_speed = 0.7;

% Parametri PID per i follower (controllo distanza)
K_p_dist = 2000;  
K_i_dist = 0.8;  
K_d_dist = 0.4;  

% Tempo di separazione (CTH)
t_CTH = 1.5; % Tempo di separazione tra i veicoli

% Definisci le posizioni dei semafori e le velocità target corrispondenti
traffic_light_positions = [300, 600, 900, 1200, 1550];
v_targets = [9, 12, 6, 8, 10];  % Un valore di velocità per ogni semaforo

% Condizioni iniziali: posizioni e velocità
x0 = zeros(2 * n_vehicles, 1);
x0(1) = 0; % Il leader parte dalla posizione 0
for i = 2:n_vehicles
    x0(i) = -d_init * (i - 1);  
end

% Tempo di simulazione
t_span = [0 200];

% Risoluzione numerica con ODE45
[t, x] = ode45(@(t, x) system_dynamics( ...
    t, x, n_vehicles, m, delta, d_min, ...
    traffic_light_positions, v_targets, ...
    t_CTH, ...
    K_p_speed, K_i_speed, K_d_speed, ...
    K_p_dist, K_i_dist, K_d_dist), ...
    t_span, x0);

% Grafico delle velocità e posizioni
figure;
for i = 1:n_vehicles
    % Velocità
    subplot(2, n_vehicles, i);
    plot(t, x(:, n_vehicles + i));
    title(['Vel v' num2str(i)]);
    xlabel('Tempo [s]');
    ylabel('Velocità [m/s]');
    
    % Posizione
    subplot(2, n_vehicles, n_vehicles + i);
    plot(t, x(:, i));
    title(['Pos p' num2str(i)]);
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
end

% Calcolo delle distanze tra i veicoli
distances = zeros(n_vehicles-1, length(t));
for i = 2:n_vehicles
    distances(i-1, :) = x(:, 1) - x(:, i);  % distanza tra veicolo i e il leader
end

% Grafico delle distanze tra veicoli
figure;
for i = 1:n_vehicles-1
    plot(t, distances(i, :));
    hold on;
end
title('Distanza tra i veicoli');
xlabel('Tempo [s]');
ylabel('Distanza [m]');
legend(arrayfun(@(x) ['Distanza p' num2str(x) ' - p' num2str(x+1)], 1:n_vehicles-1, 'UniformOutput', false));

% ------------------------------------------------
% Grafico semafori e posizioni veicoli
figure;
hold on;

% Durata del ciclo del semaforo
T = 30;
traffic_lights = [
    create_traffic_light(300, 0, 10, T),
    create_traffic_light(600, 10, 20, T),
    create_traffic_light(900, 20, 30, T),
    create_traffic_light(1200, 0, 10, T),
    create_traffic_light(1550, 10, 20, T)
];

% Tracciamento scatter semafori
times = floor(t_span(1)):floor(t_span(2));
all_times = [];
all_distances = [];
all_colors = [];

for i = 1:length(traffic_lights)
    for j = 1:length(times)
        time = times(j);
        all_times = [all_times, time];
        all_distances = [all_distances, traffic_lights(i).distance];

        if is_green(traffic_lights(i), time)
            all_colors = [all_colors; [0, 1, 0]];  % Verde
        else
            all_colors = [all_colors; [1, 0, 0]];  % Rosso
        end
    end
end

scatter(all_times, all_distances, 10, all_colors, 'filled', 'DisplayName','Semafori');

% Plot delle posizioni dei veicoli
for i = 1:n_vehicles
    plot(t, x(:, i), 'DisplayName', ['Posizione p' num2str(i)]);
end
title('Posizioni dei veicoli + Mappa semafori');
xlabel('Tempo [s]');
ylabel('Posizione [m]');
legend('show');

% =============================================================================
% Funzione di dinamica - con velocità target dinamica
% =============================================================================
function dx = system_dynamics( ...
    t, x, n_vehicles, m, delta, d_min, ...
    traffic_light_positions, v_targets, ...
    t_CTH, ...
    K_p_speed, K_i_speed, K_d_speed, ...
    K_p_dist,  K_i_dist,  K_d_dist)

    dx = zeros(2 * n_vehicles, 1);

    % Calcolo del passo temporale (dt) in maniera semplificata
    persistent t_prev
    if isempty(t_prev)
        t_prev = t;
    end
    dt = 0.01;  % step fisso per l'approssimazione derivata
    t_prev = t;

    % Inizializzazione variabili PID se necessario
    persistent error_integral_speed previous_error_speed
    persistent error_integral_dist  previous_error_dist
    if isempty(error_integral_speed) || isempty(error_integral_dist)
        error_integral_speed = 0;             % Leader
        previous_error_speed = 0;
        error_integral_dist = zeros(n_vehicles, 1); 
        previous_error_dist = zeros(n_vehicles, 1);
    end

    for i = 1:n_vehicles
        
        % Derivata della posizione = velocità
        dx(i) = x(n_vehicles + i);
        
        if i == 1
            % Leader: PID sulla velocità con v_target dinamico
            current_v_target = get_current_v_target(x(1), traffic_light_positions, v_targets);
            velocity_error = current_v_target - x(n_vehicles + 1);

            % Integrale e derivata dell'errore
            error_integral_speed = error_integral_speed + velocity_error * dt;
            velocity_derivative  = (velocity_error - previous_error_speed) / dt;

            % Controllo PID della velocità -> forza
            U_leader = K_p_speed * velocity_error ...
                     + K_i_speed * error_integral_speed ...
                     + K_d_speed * velocity_derivative;
            previous_error_speed = velocity_error;

            % a = (U_leader + delta(t)) / m(1)
            dx(n_vehicles + 1) = (U_leader + delta(t)) / m(1);

            % Limiti di sicurezza sulla velocità
            max_speed = 30;  
            current_speed = x(n_vehicles + 1) + dx(n_vehicles + 1)*dt; 
            if current_speed < 0
                current_speed = 0;
            elseif current_speed > max_speed
                current_speed = max_speed;
            end

            % Sovrascrivo la derivata
            dx(n_vehicles + 1) = (current_speed - x(n_vehicles + 1)) / dt;

        else
            % Follower: PID sulla distanza dal veicolo precedente
            distance = x(i) - x(i-1);

            v_1 = x(n_vehicles + i);  
            d_CHT = -(d_min + t_CTH * v_1);
            
            dist_error = d_CHT - distance;
            
            % Integrale e derivata dell'errore di distanza
            error_integral_dist(i) = error_integral_dist(i) + dist_error * dt;
            distance_derivative = (dist_error - previous_error_dist(i)) / dt;
            previous_error_dist(i) = dist_error;
            
            % Controllo PID della distanza
            U_dist = K_p_dist * dist_error ...
                   + K_i_dist * error_integral_dist(i) ...
                   + K_d_dist * distance_derivative;

            dx(n_vehicles + i) = (U_dist) / m(i);
            
            % Velocità ipotetica
            hypotetical_speed = x(n_vehicles + i) + dx(n_vehicles + i)*dt;
            if hypotetical_speed < 0
                % Esempio: potrei impostare a zero
                % hypotetical_speed = 0;
            end
            
            dx(n_vehicles + i) = (hypotetical_speed - x(n_vehicles + i)) / dt;
        end
    end
end

% =============================================================================
% Definisci la funzione che determina la velocità target
% =============================================================================
function vt = get_current_v_target(x_leader, traffic_light_positions, v_targets)
    % Trova la prima posizione di semaforo che x_leader non ha ancora raggiunto
    idx = find(x_leader < traffic_light_positions, 1);

    % Se il veicolo ha superato tutti i semafori, usa l'ultimo target
    if isempty(idx)
        vt = v_targets(end);
    else
        vt = v_targets(idx);
    end
end

% =============================================================================
% Funzioni per la gestione dei semafori
% =============================================================================
function light = create_traffic_light(distance, green_start, green_end, cycle_time)
    light.distance = distance;
    light.green_start = green_start;
    light.green_end = green_end;
    light.cycle_time = cycle_time;
    light.green_duration = green_end - green_start;
    light.offset = mod(green_start, cycle_time);
end

function is_green_status = is_green(light, time)
    time_in_cycle = mod(time - light.offset, light.cycle_time);
    if light.green_start <= light.green_end
        is_green_status = (time_in_cycle >= light.green_start) && (time_in_cycle < light.green_end);
    else
        is_green_status = (time_in_cycle >= light.green_start) || (time_in_cycle < light.green_end);
    end
end