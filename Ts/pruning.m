% filepath: C:\Code\GitHub\PlatoonEcoControl\Ts\simulation.m
% =============================================================================
% Main Script
% =============================================================================
clear;
clc;
close all;

%% Costanti
final_time = 150;         % Tempo di simulazione (secondi)
final_distance = 1800;    % Distanza finale (metri)
T = 30;                   % Durata del ciclo del semaforo (secondi)
tf = final_time;

% Limiti di velocità (m/s)
v_min = 5;   % Velocità minima
v_max = 30;   % Velocità massima

%% Creazione degli incroci e semafori
traffic_lights = [
    create_traffic_light(300, 0, 10, T)
    create_traffic_light(600, 10, 20, T)
    create_traffic_light(900, 20, 30, T)
    create_traffic_light(1200, 0, 10, T)
    create_traffic_light(1550, 10, 20, T)
];

%% Creazione dei dati per il plot dello stato dei semafori
times = 0:final_time-1;  % Vettore dei tempi
nLights = length(traffic_lights);
all_times = [];
all_distances = [];
all_colors = [];

for i = 1:nLights
    for t = times
        all_times(end+1) = t;
        all_distances(end+1) = traffic_lights(i).distance;
        if is_green(traffic_lights(i), t)
            all_colors(end+1, :) = [0, 1, 0];  % Verde
        else
            all_colors(end+1, :) = [1, 0, 0];  % Rosso
        end
    end
end

%% Plot dello stato dei semafori
figure;
scatter(all_times, all_distances, 10, all_colors, 'filled');
xlabel('Tempo (s)');
ylabel('Distanza (m)');
title('Stato dei semafori nel tempo');
legend(arrayfun(@(i) ['Semaforo ' num2str(i)], 1:nLights, 'UniformOutput', false));
grid on;
hold on;

%% Disegna la traiettoria del veicolo:
% Traiettoria basata sui tempi minimi (blue line) e 
% traiettoria basata sui tempi massimi (red dashed line)
draw_vehicle_path_simulation(tf, final_distance, traffic_lights, v_min, v_max);
draw_vehicle_path_simulation_max(tf, final_distance, traffic_lights, v_min, v_max);
hold off;

%% Funzioni locali

% Funzione per creare un semaforo
function light = create_traffic_light(distance, green_start, green_end, cycle_time)
    light.distance = distance;
    light.green_start = green_start;
    light.green_end = green_end;
    light.cycle_time = cycle_time;
    light.green_duration = green_end - green_start;
    light.offset = mod(green_start, cycle_time);
end

% Funzione per verificare se il semaforo è verde in un dato istante
function status = is_green(light, time)
    time_in_cycle = mod(time - light.offset, light.cycle_time);
    if light.green_start <= light.green_end
        status = (time_in_cycle >= light.green_start) && (time_in_cycle < light.green_end);
    else
        status = (time_in_cycle >= light.green_start) || (time_in_cycle < light.green_end);
    end
end

% Algoritmo di velocity pruning: determina i tempi minimi e massimi di attraversamento
function [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max)
    n = length(traffic_lights);
    t_min = zeros(1, n);
    t_max = zeros(1, n);
    d = [traffic_lights.distance];
    
    % Passaggio forward
    t_min(1) = d(1) / v_max;
    t_max(1) = d(1) / v_min;
    t_min(1) = next_green(traffic_lights(1), t_min(1));
    t_max(1) = prev_green(traffic_lights(1), t_max(1));
    
    for i = 2:n
        dt = d(i) - d(i-1);
        t_min(i) = t_min(i-1) + dt / v_max;
        t_max(i) = t_max(i-1) + dt / v_min;
        % Verifica per il rispetto di v_max fino alla destinazione
        t_max(i) = min(t_max(i), tf - (final_distance - d(i)) / v_max);
        
        t_min(i) = next_green(traffic_lights(i), t_min(i));
        t_max(i) = prev_green(traffic_lights(i), t_max(i));
    end
    
    % Passaggio backward per adeguare i tempi
    for i = n:-1:2
        if t_max(i) > t_max(i-1) + (d(i) - d(i-1)) / v_max
            t_max(i-1) = t_max(i) - (d(i) - d(i-1)) / v_max;
            t_max(i-1) = prev_green(traffic_lights(i-1), t_max(i-1));
        end
    end
end

% Funzione per ottenere il primo istante in cui il semaforo è verde a partire da t
function t_next = next_green(light, t)
    if is_green(light, t)
        t_next = t;
    else
        time_in_cycle = mod(t - light.offset, light.cycle_time);
        if time_in_cycle < light.green_start
            t_next = t + (light.green_start - time_in_cycle);
        else
            t_next = t + (light.cycle_time - time_in_cycle) + light.green_start;
        end
    end
end

% Funzione per ottenere l'ultimo istante in cui il semaforo era verde a partire da t
function t_prev = prev_green(light, t)
    if is_green(light, t)
        t_prev = t;
    else
        time_in_cycle = mod(t - light.offset, light.cycle_time);
        if time_in_cycle >= light.green_end
            t_prev = t - (time_in_cycle - light.green_end);
        else
            t_prev = t - time_in_cycle - (light.cycle_time - light.green_end);
        end
    end
end

% Disegna la traiettoria del veicolo basata sui tempi minimi (acrossing times)
function draw_vehicle_path_simulation(tf, final_distance, traffic_lights, v_min, v_max)
    [t_min, ~] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance];
    % Traiettoria: partenza (0), attraversamenti e destinazione
    traj_time = [0, t_min, tf];
    traj_distance = [0, d, final_distance];
    plot(traj_time, traj_distance, 'b-', 'LineWidth', 2);
end

% Disegna la traiettoria del veicolo basata sui tempi massimi (acrossing times)
function draw_vehicle_path_simulation_max(tf, final_distance, traffic_lights, v_min, v_max)
    [~, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance];
    traj_time = [0, t_max, tf];
    traj_distance = [0, d, final_distance];
    plot(traj_time, traj_distance, 'b-', 'LineWidth', 2);
end