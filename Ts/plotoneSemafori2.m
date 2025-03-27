%%%% segnala se passa col rosso

clear; clc; close all;

%% PARAMETRI PER LA PRIMA PARTE (CALCOLO PERCORSO OTTIMO)
final_time = 150;         
final_distance = 1800;    
T = 30;                   
tf = final_time;
v_min = 5;   
v_max = 30;  
b1 = 0.1;  
b2 = 0.01;  

%% CREAZIONE SEMAFORI
traffic_lights = [
    create_traffic_light(300, 0, 10, T)
    create_traffic_light(600, 10, 20, T)
    create_traffic_light(900, 20, 30, T)
    create_traffic_light(1200, 0, 10, T)
    create_traffic_light(1550, 10, 20, T)
];

%% PLOT SEMAFORI
figure;
times = 0:tf-1;
nLights = length(traffic_lights);
all_times = []; 
all_distances = []; 
all_colors = [];
for i = 1:nLights
    for t_ = times
        all_times(end+1) = t_;
        all_distances(end+1) = traffic_lights(i).distance;
        if is_green(traffic_lights(i), t_)
            all_colors(end+1, :) = [0, 1, 0];
        else
            all_colors(end+1, :) = [1, 0, 0];
        end
    end
end
scatter(all_times, all_distances, 10, all_colors, 'filled');
xlabel('Tempo (s)'); ylabel('Distanza (m)');
title('Stato dei semafori nel tempo');
grid on; hold on;

%% TRACCIAMENTO TRAIETTORIE RIFERIMENTO
draw_vehicle_path_simulation(tf, final_distance, traffic_lights, v_min, v_max);
draw_vehicle_path_simulation_max(tf, final_distance, traffic_lights, v_min, v_max);

%% COSTRUZIONE GRAFO E NODI
[t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
d = [traffic_lights.distance]; 
nIntersections = length(traffic_lights);
Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
nodeId = 1;
Nodes(nodeId) = struct('id', nodeId, 't', 0, 'd', 0, 'int', 0); % sorgente
nodeId = nodeId + 1;

for i = 1:nIntersections
    light = traffic_lights(i);
    for k = 0:ceil(tf / light.cycle_time)
        cycle_start = k * light.cycle_time + light.offset;
        if light.green_start <= light.green_end
            abs_green_start = cycle_start + light.green_start;
            abs_green_end   = cycle_start + light.green_end;
            if abs_green_start <= tf
                overlap_start = max(abs_green_start, t_min(i));
                overlap_end   = min(abs_green_end, t_max(i));
                if overlap_start < overlap_end
                    middle_time = ceil((overlap_start + overlap_end) / 2);
                    Nodes(nodeId) = struct('id', nodeId, 't', middle_time, 'd', d(i), 'int', i);
                    nodeId = nodeId + 1;
                end
            end
        else
            abs_green_start_1 = cycle_start + light.green_start;
            abs_green_end_1   = cycle_start + light.cycle_time;
            if abs_green_start_1 <= tf
                overlap_start = max(abs_green_start_1, t_min(i));
                overlap_end   = min(abs_green_end_1, t_max(i));
                if overlap_start < overlap_end
                    middle_time = (overlap_start + overlap_end) / 2;
                    Nodes(nodeId) = struct('id', nodeId, 't', middle_time, 'd', d(i), 'int', i);
                    nodeId = nodeId + 1;
                end
            end
            abs_green_start_2 = cycle_start;
            abs_green_end_2   = cycle_start + light.green_end;
            if abs_green_end_2 <= tf
                overlap_start = max(abs_green_start_2, t_min(i));
                overlap_end   = min(abs_green_end_2, t_max(i));
                if overlap_start < overlap_end
                    middle_time = (overlap_start + overlap_end) / 2;
                    Nodes(nodeId) = struct('id', nodeId, 't', middle_time, 'd', d(i), 'int', i);
                    nodeId = nodeId + 1;
                end
            end
        end
    end
end

Nodes(nodeId) = struct('id', nodeId, 't', tf, 'd', final_distance, 'int', nIntersections+1);
nNodes = nodeId;

%% COSTRUZIONE ARCHI
Edges = struct('from', {}, 'to', {}, 'w', {});
edgeCount = 1;
for i = 1:nNodes
    curr_lvl = Nodes(i).int;
    for j = 1:nNodes
        next_lvl = Nodes(j).int;
        if next_lvl == curr_lvl + 1
            if Nodes(j).t > Nodes(i).t && Nodes(j).d > Nodes(i).d
                if Nodes(j).int > 0 && Nodes(j).int <= nIntersections
                    if ~is_green(traffic_lights(Nodes(j).int), Nodes(j).t)
                        continue;
                    end
                end
                delta_t = Nodes(j).t - Nodes(i).t;
                delta_d = Nodes(j).d - Nodes(i).d;
                v_link  = delta_d / delta_t;
                if v_link >= v_min && v_link <= v_max
                    E_link = delta_t * (b1*v_link + b2*v_link^2);
                    Edges(edgeCount) = struct('from', Nodes(i).id, 'to', Nodes(j).id, 'w', E_link);
                    edgeCount = edgeCount + 1;
                end
            end
        end
    end
end

%% DIJKSTRA
[path, cost] = dijkstra(Nodes, Edges, Nodes(1).id, Nodes(end).id);
fprintf('Costo energetico ottimo: %f\n', cost);
fprintf('Percorso ottimo (node id - crossing time):\n');
for k = 1:length(path)
    n_idx = path(k);
    fprintf('Node %d: t=%f s, d=%f m\n', ...
        Nodes(n_idx).id, Nodes(n_idx).t, Nodes(n_idx).d);
end

opt_nodes = Nodes(path);
opt_t = arrayfun(@(n) n.t, opt_nodes);
opt_d = arrayfun(@(n) n.d, opt_nodes);

% Vettore di velocità media (speeds) lungo il percorso
speeds = zeros(1, length(path) - 1);
for k = 1:(length(path) - 1)
    idxA = path(k);
    idxB = path(k+1);
    delta_d = Nodes(idxB).d - Nodes(idxA).d;
    delta_t = Nodes(idxB).t - Nodes(idxA).t;
    speeds(k) = delta_d / delta_t;
end
disp('Velocità media su ciascun segmento (m/s):');
disp(['[ ' num2str(speeds, '%.3f, ') ']']);

plot(opt_t, opt_d, 'k--', 'LineWidth', 3);  
title('Traiettoria ottima dal pruning');
legend('Semafori','t_{min}','t_{max}','Traiettoria ottima','Location','Best');

%% SECONDA PARTE: SIMULAZIONE ODE CON "speeds" COME v_targets
% ==========================================================

% Funzione "delta"
b1_for_delta = 450;
b2_for_delta = 450;
b3_for_delta = 1; 
b4_for_delta = 3;
delta = @(t) 0 * (b1_for_delta + b2_for_delta*( ...
                  sin((1/b3_for_delta)*t + b4_for_delta) + 0.25*rand));

% Parametri veicoli
n_vehicles = 5;
m_vehicles = 1000 * ones(1, n_vehicles);

% PID
K_p_speed = 7000;     
K_i_speed = 0;
K_d_speed = 0.7;
K_p_dist = 2000;  
K_i_dist = 0.8;  
K_d_dist = 0.4;
t_CTH = 1.5;  
d_init = 4;

% In questo esempio, associamo "speeds" come un vettore di target
v_targets = speeds;  

% Condizioni iniziali
x0 = zeros(2 * n_vehicles, 1);
x0(1) = 0; 
for i = 2:n_vehicles
    x0(i) = -d_init * (i - 1);  
end

% Integrale numerico
t_span = [0 150];
[t_sim, x_sim] = ode45(@(t, x) system_dynamics( ...
    t, x, n_vehicles, m_vehicles, delta, ...
    traffic_lights, v_targets, t_CTH, ...
    K_p_speed, K_i_speed, K_d_speed, ...
    K_p_dist, K_i_dist, K_d_dist), ...
    t_span, x0);

% Controllo eventuali passaggi col rosso
check_red_light_violations(t_sim, x_sim, traffic_lights);

figure; hold on;

% Mappa dei semafori (verde/rosso) nel tempo
times = floor(t_span(1)):floor(t_span(2));
all_times = [];
all_positions = [];
all_colors = [];
for i = 1:length(traffic_lights)
    for j = 1:length(times)
        time = times(j);
        all_times(end+1) = time;
        all_positions(end+1) = traffic_lights(i).distance;
        if is_green(traffic_lights(i), time)
            all_colors(end+1, :) = [0, 1, 0];  % verde
        else
            all_colors(end+1, :) = [1, 0, 0];  % rosso
        end
    end
end
scatter(all_times, all_positions, 10, all_colors, 'filled', 'DisplayName','Semafori');

% Posizioni dei veicoli
for i = 1:n_vehicles
    plot(t_sim, x_sim(:, i), 'DisplayName', ['Veicolo ' num2str(i)]);
end

title('Mappa semafori e posizioni dei veicoli');
xlabel('Tempo [s]');
ylabel('Posizione [m]');
legend('show');


% Visualizza risultati
figure;
for i = 1:n_vehicles
    subplot(2, n_vehicles, i);
    plot(t_sim, x_sim(:, n_vehicles + i));
    title(['Vel Veicolo ' num2str(i)]);
    xlabel('Tempo [s]'); ylabel('Velocità [m/s]');
    
    subplot(2, n_vehicles, n_vehicles + i);
    plot(t_sim, x_sim(:, i));
    title(['Posizione Veicolo ' num2str(i)]);
    xlabel('Tempo [s]'); ylabel('Posizione [m]');
end

%%%% =============================================================================
%%%% FUNZIONI LOCALI
%%%% =============================================================================

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
    d = [traffic_lights.distance];
    t_min = zeros(1,n); 
    t_max = zeros(1,n);

    t_min(1) = d(1)/v_max;
    t_max(1) = d(1)/v_min;
    t_min(1) = next_green(traffic_lights(1), t_min(1));
    t_max(1) = prev_green(traffic_lights(1), t_max(1));

    for i = 2:n
        dist_inc = d(i) - d(i-1);
        t_min(i) = t_min(i-1) + dist_inc / v_max;
        t_max(i) = t_max(i-1) + dist_inc / v_min;
        t_max(i) = min(t_max(i), tf - (final_distance - d(i))/v_max);
        t_min(i) = next_green(traffic_lights(i), t_min(i));
        t_max(i) = prev_green(traffic_lights(i), t_max(i));
    end

    for i = n:-1:2
        needed_time = (d(i) - d(i-1))/v_max;
        if t_max(i) > t_max(i-1) + needed_time
            t_max(i-1) = t_max(i) - needed_time;
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
    hold on;
end

function draw_vehicle_path_simulation_max(tf, final_distance, traffic_lights, v_min, v_max)
    [~, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance];
    traj_t = [0, t_max, tf];
    traj_d = [0, d, final_distance];
    plot(traj_t, traj_d, 'b-', 'LineWidth', 2);
    hold on;
end

function [path, cost] = dijkstra(Nodes, Edges, source, target)
    nNodes = length(Nodes);
    cost = inf(1,nNodes);
    prev = nan(1,nNodes);
    cost(source) = 0;
    Q = 1:nNodes;
    while ~isempty(Q)
        [~, idx] = min(cost(Q));
        u = Q(idx);
        Q(Q==u) = [];
        if u == target
            break;
        end
        for e = Edges
            if e.from == u
                v = e.to;
                alt = cost(u) + e.w;
                if alt < cost(v)
                    cost(v) = alt;
                    prev(v) = u;
                end
            end
        end
    end

    path = [];
    u = target;
    if ~isnan(prev(u)) || u == source
        while ~isnan(u)
            path = [u, path];
            u = prev(u);
        end
    end
end

%% Dinamica per ODE45
function dx = system_dynamics(t, x, n_vehicles, m, delta_func, ...
                              traffic_lights, v_targets, t_CTH, ...
                              K_p_speed, K_i_speed, K_d_speed, ...
                              K_p_dist, K_i_dist, K_d_dist)

    dx = zeros(2*n_vehicles, 1);

    % Calcolo del dt
    persistent t_prev
    if isempty(t_prev)
        t_prev = t;
    end
    dt = t - t_prev;
    if dt <= 0
        dt = 0.01; 
    end
    t_prev = t;

    % Variabili PID persistenti
    persistent error_integral_speed previous_error_speed
    persistent error_integral_dist  previous_error_dist
    if isempty(error_integral_speed) || isempty(error_integral_dist)
        error_integral_speed = 0;
        previous_error_speed = 0;
        error_integral_dist  = zeros(n_vehicles,1);
        previous_error_dist  = zeros(n_vehicles,1);
    end

    for i = 1:n_vehicles
        dx(i) = x(n_vehicles + i);
        
        if i == 1
            % Leader: usa i v_targets come “speeds”
            vt = get_current_v_target_indexed(x(1), traffic_lights, v_targets);
            velocity_error = vt - x(n_vehicles + 1);

            error_integral_speed = error_integral_speed + velocity_error * dt;
            velocity_derivative  = (velocity_error - previous_error_speed) / dt;
            previous_error_speed = velocity_error;

            U_leader = K_p_speed * velocity_error ...
                     + K_i_speed * error_integral_speed ...
                     + K_d_speed * velocity_derivative;

            dx(n_vehicles + 1) = (U_leader + delta_func(t)) / m(1);

            max_speed = 30;  
            current_speed = x(n_vehicles + 1) + dx(n_vehicles + 1)*dt; 
            if current_speed < 0
                current_speed = 0;
            elseif current_speed > max_speed
                current_speed = max_speed;
            end
            dx(n_vehicles + 1) = (current_speed - x(n_vehicles + 1)) / dt;

        else
            % Follower
            distance  = x(i) - x(i-1);
            v_cur     = x(n_vehicles + i);
            d_min_val = 1;  % la costante d_min
            d_CHT     = -(d_min_val + t_CTH * v_cur);
            dist_error = d_CHT - distance;

            error_integral_dist(i) = error_integral_dist(i) + dist_error * dt;
            distance_derivative = (dist_error - previous_error_dist(i)) / dt;
            previous_error_dist(i) = dist_error;

            U_dist = K_p_dist * dist_error ...
                   + K_i_dist * error_integral_dist(i) ...
                   + K_d_dist * distance_derivative;

            dx(n_vehicles + i) = U_dist / m(i);

            hypotetical_speed = x(n_vehicles + i) + dx(n_vehicles + i)*dt;
            if hypotetical_speed < 0
                hypotetical_speed = 0;
            end
            dx(n_vehicles + i) = (hypotetical_speed - x(n_vehicles + i)) / dt;
        end
    end
end

function vt = get_current_v_target_indexed(x_leader, traffic_lights, v_targets)
    idx = find(x_leader < [traffic_lights.distance], 1);
    if isempty(idx)
        vt = v_targets(end);
    else
        idx_clamp = min(idx, length(v_targets));
        vt = v_targets(idx_clamp);
    end
end


%%%% Aggiungi dopo la fine del file principale
function check_red_light_violations(t_sim, x_sim, traffic_lights)
    % Recupera il numero di veicoli (dalla matrice x_sim con posizioni+velocità)
    n_vehicles = size(x_sim, 2) / 2;  
    for v = 1:n_vehicles
        % La posizione del veicolo v è in x_sim(:, v).
        pos_v = x_sim(:, v);
        for L = 1:length(traffic_lights)
            light_dist = traffic_lights(L).distance;
            for k = 2:length(t_sim)
                % Controlla se nell’intervallo [k-1, k] superiamo la distanza del semaforo
                if pos_v(k-1) < light_dist && pos_v(k) >= light_dist
                    % Trova l’istante di crossing (interpolazione lineare)
                    cross_time = t_sim(k-1) + (t_sim(k)-t_sim(k-1)) * ...
                        ((light_dist - pos_v(k-1)) / (pos_v(k) - pos_v(k-1)));
                    % Se il semaforo è rosso in quell’istante, stampiamo warning
                    if ~is_green(traffic_lights(L), cross_time)
                        fprintf('Veicolo %d ha passato col rosso l''incrocio %d a t=%.2f s\n', ...
                            v, L, cross_time);
                    end
                end
            end
        end
    end
end