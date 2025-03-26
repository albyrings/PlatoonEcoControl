
clear; clc; close all;

%% Costanti
final_time = 150;         % Tempo di simulazione (s)
final_distance = 1800;    % Distanza finale (m)
T = 30;                   % Durata del ciclo del semaforo (s)
tf = final_time;

% Limiti di velocità (m/s)
v_min = 5;   
v_max = 30;  

% Costanti per il calcolo dell'energia (valori esemplificativi)
b1 = 0.1;  b2 = 0.01;         % Energia per il viaggio costante (E_link)
b1_jump = 0.5;  b2_jump = 0.05; % Energia per il salto (E_jump)
accel = 1000;                 % Accelerazione (usata per decelerazione)

%% Parametro per simulazione: velocità iniziale
initial_speed = 0;   % Il veicolo parte a velocità massima

%% Creazione degli incroci e semafori
traffic_lights = [
    create_traffic_light(300, 0, 10, T)
    create_traffic_light(600, 10, 20, T)
    create_traffic_light(900, 20, 30, T)
    create_traffic_light(1200, 0, 10, T)
    create_traffic_light(1550, 10, 20, T)
];

%% Plot dello stato dei semafori
figure;
times = 0:tf-1;
nLights = length(traffic_lights);
all_times = []; all_distances = []; all_colors = [];
for i = 1:nLights
    for t = times
        all_times(end+1) = t;
        all_distances(end+1) = traffic_lights(i).distance;
        if is_green(traffic_lights(i), t)
            all_colors(end+1, :) = [0, 1, 0];  % verde
        else
            all_colors(end+1, :) = [1, 0, 0];  % rosso
        end
    end
end
scatter(all_times, all_distances, 10, all_colors, 'filled');
xlabel('Tempo (s)'); ylabel('Distanza (m)');
title('Stato dei semafori nel tempo');
legend(arrayfun(@(i) ['Semaforo ' num2str(i)], 1:nLights, 'UniformOutput', false));
grid on; hold on;

%% Tracciamento delle traiettorie di riferimento (pruning)
draw_vehicle_path_simulation(tf, final_distance, traffic_lights, v_min, v_max);       % linea blu (t_min)
draw_vehicle_path_simulation_max(tf, final_distance, traffic_lights, v_min, v_max);   % linea blu (t_max)

%% =============================================================================
% Costruzione del grafo per il percorso ottimo
% =============================================================================
% Per ogni incrocio l'algoritmo di pruning dà l'intervallo [t_min, t_max]. Definiamo
% 3 nodi per incrocio (min, medio, max) all'interno di questo intervallo.
nNodesPerIntersection = 3; 
[t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
d = [traffic_lights.distance]; nIntersections = length(traffic_lights);

% Costruiamo la lista dei nodi: S (sorgente), nodi degli incroci, D (destinazione)
Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
nodeId = 1;
Nodes(nodeId) = struct('id', nodeId, 't', 0, 'd', 0, 'int', 0); % sorgente
nodeId = nodeId + 1;
for i = 1:nIntersections
    light = traffic_lights(i);
    for k = 0:ceil(tf / light.cycle_time) % Iterate through enough cycles
        cycle_start = k * light.cycle_time + light.offset;
        green_start_cycle = light.green_start;
        green_end_cycle = light.green_end;

        if green_start_cycle <= green_end_cycle
            abs_green_start = cycle_start + green_start_cycle;
            abs_green_end = cycle_start + green_end_cycle;
            if abs_green_start <= tf
                overlap_start = max(abs_green_start, t_min(i));
                overlap_end = min(abs_green_end, t_max(i));
                if overlap_start < overlap_end
                    middle_time = ceil((overlap_start + overlap_end) / 2);
                    Nodes(nodeId) = struct('id', nodeId, 't', middle_time, 'd', d(i), 'int', i);
                    nodeId = nodeId + 1;
                end
            end
        else % Wrap around
            abs_green_start_1 = cycle_start + green_start_cycle;
            abs_green_end_1 = cycle_start + light.cycle_time;
            if abs_green_start_1 <= tf
                overlap_start = max(abs_green_start_1, t_min(i));
                overlap_end = min(abs_green_end_1, t_max(i));
                if overlap_start < overlap_end
                    middle_time = (overlap_start + overlap_end) / 2;
                    Nodes(nodeId) = struct('id', nodeId, 't', middle_time, 'd', d(i), 'int', i);
                    nodeId = nodeId + 1;
                end
            end

            abs_green_start_2 = cycle_start;
            abs_green_end_2 = cycle_start + green_end_cycle;
            if abs_green_end_2 <= tf
                overlap_start = max(abs_green_start_2, t_min(i));
                overlap_end = min(abs_green_end_2, t_max(i));
                if overlap_start < overlap_end
                    middle_time = (overlap_start + overlap_end) / 2;
                    Nodes(nodeId) = struct('id', nodeId, 't', middle_time, 'd', d(i), 'int', i);
                    nodeId = nodeId + 1;
                end
            end
        end
    end
end
Nodes(nodeId) = struct('id', nodeId, 't', tf, 'd', final_distance, 'int', nIntersections+1); % destinazione
nNodes = nodeId;

% Costruiamo gli archi (grafo diretto aciclico)
Edges = struct('from', {}, 'to', {}, 'w', {});
edgeCount = 1;
for i = 1:nNodes
    current_level = Nodes(i).int;
    for j = 1:nNodes
        next_level = Nodes(j).int;
        % Un nodo è al "livello successivo" se il suo indice di intersezione è maggiore di uno
        % rispetto al nodo corrente. La sorgente ha int=0, le intersezioni hanno int da 1 a nIntersections,
        % e la destinazione ha int=nIntersections+1.
        if next_level == current_level + 1
            if Nodes(j).t > Nodes(i).t && Nodes(j).d > Nodes(i).d
                % Se il nodo j indica un incrocio, assicuriamoci che il crossing time sia verde.
                if Nodes(j).int > 0 && Nodes(j).int <= nIntersections
                    if ~is_green(traffic_lights(Nodes(j).int), Nodes(j).t)
                        continue;
                    end
                end
                delta_t = Nodes(j).t - Nodes(i).t;
                delta_d = Nodes(j).d - Nodes(i).d;
                v_link = delta_d / delta_t;
                if v_link >= v_min && v_link <= v_max
                    E_link = delta_t * (b1*v_link + b2*v_link^2);
                    w = E_link;  % modelliamo E_jump = 0 in questa semplificazione
                    Edges(edgeCount) = struct('from', Nodes(i).id, 'to', Nodes(j).id, 'w', w);
                    edgeCount = edgeCount + 1;
                end
            end
        end
    end
end

%% Risoluzione del percorso ottimo con Dijkstra
[path, cost] = dijkstra(Nodes, Edges, Nodes(1).id, Nodes(end).id);

fprintf('Costo energetico ottimo: %f\n', cost);
fprintf('Percorso ottimo (node id e crossing time):\n');
for k = 1:length(path)
    n_idx = path(k);
    fprintf('Node %d: t = %f s, d = %f m\n', Nodes(n_idx).id, Nodes(n_idx).t, Nodes(n_idx).d);
end
opt_nodes = Nodes(path);
opt_t = arrayfun(@(n) n.t, opt_nodes);
opt_d = arrayfun(@(n) n.d, opt_nodes);

speeds = zeros(1, length(path) - 1);
for k = 1:length(path)-1
    idxA = path(k);
    idxB = path(k+1);
    delta_d = Nodes(idxB).d - Nodes(idxA).d;
    delta_t = Nodes(idxB).t - Nodes(idxA).t;
    speeds(k) = delta_d / delta_t;  % velocità media fra i due nodi
end

disp('Velocità media su ciascun segmento (m/s):');
disp(['[ ' num2str(speeds, '%.3f, ') ']']);
plot(opt_t, opt_d, 'k--', 'LineWidth', 3);  % visualizza solo la traiettoria ottima

%% =============================================================================
% Simulazione del veicolo lungo il percorso ottimo
% =============================================================================
% Per ogni segmento (da S a incrocio, da incrocio a incrocio, etc.),
% il veicolo parte con una certa velocità e:
% - Se la velocità corrente è superiore a quella richiesta per arrivare esattamente
%   al crossing time, viene simulata una fase di decelerazione (con accelerazione costante -a)
%   fino al valore richiesto.
% - Successivamente, il veicolo percorre il tratto a velocità costante.
% - Se il tempo totale del tratto risulta inferiore al tempo previsto, il veicolo attende.
% - Infine, se al momento dell'arrivo il semaforo è rosso, attende fino a next_green.
dt = 0.01;  % passo di simulazione


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FUNZIONI LOCALI
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function light = create_traffic_light(distance, green_start, green_end, cycle_time)
    light.distance = distance;
    light.green_start = green_start;
    light.green_end = green_end;
    light.cycle_time = cycle_time;
    light.green_duration = green_end - green_start;
    light.offset = mod(green_start, cycle_time);
end

function status = is_green(light, time)
    time_in_cycle = mod(time - light.offset, light.cycle_time);
    if light.green_start <= light.green_end
        status = (time_in_cycle >= light.green_start) && (time_in_cycle < light.green_end);
    else
        status = (time_in_cycle >= light.green_start) || (time_in_cycle < light.green_end);
    end
end

function [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max)
    n = length(traffic_lights);
    t_min = zeros(1,n); t_max = zeros(1,n);
    d = [traffic_lights.distance];
    
    t_min(1) = d(1)/v_max;
    t_max(1) = d(1)/v_min;
    t_min(1) = next_green(traffic_lights(1), t_min(1));
    t_max(1) = prev_green(traffic_lights(1), t_max(1));
    for i = 2:n
        dt = d(i)-d(i-1);
        t_min(i) = t_min(i-1) + dt/v_max;
        t_max(i) = t_max(i-1) + dt/v_min;
        t_max(i) = min(t_max(i), tf - (final_distance-d(i))/v_max);
        t_min(i) = next_green(traffic_lights(i), t_min(i));
        t_max(i) = prev_green(traffic_lights(i), t_max(i));
    end
    for i = n:-1:2
        if t_max(i) > t_max(i-1) + (d(i)-d(i-1))/v_max
            t_max(i-1) = t_max(i) - (d(i)-d(i-1))/v_max;
            t_max(i-1) = prev_green(traffic_lights(i-1), t_max(i-1));
        end
    end
end

function t_next = next_green(light, t)
    if is_green(light,t)
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

function t_prev = prev_green(light, t)
    if is_green(light,t)
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

function draw_vehicle_path_simulation(tf, final_distance, traffic_lights, v_min, v_max)
    [t_min, ~] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance];
    traj_t = [0, t_min, tf];
    traj_d = [0, d, final_distance];
    plot(traj_t, traj_d, 'b-', 'LineWidth', 2);
end

function draw_vehicle_path_simulation_max(tf, final_distance, traffic_lights, v_min, v_max)
    [~, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance];
    traj_t = [0, t_max, tf];
    traj_d = [0, d, final_distance];
    plot(traj_t, traj_d, 'b-', 'LineWidth', 2);
end

function [path, cost] = dijkstra(Nodes, Edges, source, target)
    nNodes = length(Nodes);
    cost = inf(1,nNodes); prev = nan(1,nNodes);
    cost(source) = 0;
    Q = 1:nNodes;
    while ~isempty(Q)
        [~, idx] = min(cost(Q));
        u = Q(idx); Q(Q==u) = [];
        if u == target, break; end
        for e = Edges
            if e.from == u
                v = e.to;
                alt = cost(u) + e.w;
                if alt < cost(v)
                    cost(v) = alt; prev(v) = u;
                end
            end
        end
    end
    path = []; u = target;
    if ~isnan(prev(u)) || u == source
        while ~isnan(u)
            path = [u, path];
            u = prev(u);
        end
    end
end

