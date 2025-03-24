% =============================================================================
% Main Script
% =============================================================================
clear;
clc;
close all;

% =============================================================================
% Costanti
% =============================================================================

final_time = 100;  % Tempo di simulazione esteso a 1000 secondi
final_distance = 1800;

T = 30;  % Durata del ciclo del semaforo (secondi)
tf = final_time;

% =============================================================================
% Creazione degli incroci e semafori
% =============================================================================
traffic_lights = [
    create_traffic_light(300, 0, 10, T),
    create_traffic_light(600, 10, 20, T),
    create_traffic_light(900, 20, 30, T),
    create_traffic_light(1200, 0, 10, T),
    create_traffic_light(1550 ...
    , 10, 20, T)
];

% =============================================================================
% Creazione dei dati per il grafico
% =============================================================================
times = 0:final_time-1;  % Vettore dei tempi
distances = zeros(length(traffic_lights), length(times));
colors = zeros(length(traffic_lights) * length(times), 3);  % Array per i colori RGB
all_times = []; % Array per i tempi
all_distances = []; % Array per le distanze
all_colors = []; % Array per i colori

% Generazione dei punti
for i = 1:length(traffic_lights)
    for j = 1:length(times)
        time = times(j);
        distances(i, j) = traffic_lights(i).distance;
        
        all_times = [all_times, times(j)];
        all_distances = [all_distances, traffic_lights(i).distance];
        
        if is_green(traffic_lights(i), time)
            all_colors = [all_colors; [0, 1, 0]];  % Verde (RGB)
        else
            all_colors = [all_colors; [1, 0, 0]];  % Rosso (RGB)
        end
    end
end

% =============================================================================
% Plot dei risultati con miglioramento delle prestazioni
% =============================================================================
figure;
scatter(all_times, all_distances, 10, all_colors, 'filled');
xlabel('Tempo (secondi)');
ylabel('Distanza (metri)');
title('Stato dei semafori nel tempo');
legend(arrayfun(@(i) ['Semaforo ' num2str(i)], 1:length(traffic_lights), 'UniformOutput', false));
grid on;

% Funzione per creare un semaforo
function light = create_traffic_light(distance, green_start, green_end, cycle_time)
    light.distance = distance;
    light.green_start = green_start;
    light.green_end = green_end;
    light.cycle_time = cycle_time;
    light.green_duration = green_end - green_start;
    light.offset = mod(green_start, cycle_time);
end

% Funzione per verificare se il semaforo è verde
function is_green_status = is_green(light, time)
    time_in_cycle = mod(time - light.offset, light.cycle_time);
    if light.green_start <= light.green_end
        is_green_status = (time_in_cycle >= light.green_start) && (time_in_cycle < light.green_end);
    else
        is_green_status = (time_in_cycle >= light.green_start) || (time_in_cycle < light.green_end);
    end
end
