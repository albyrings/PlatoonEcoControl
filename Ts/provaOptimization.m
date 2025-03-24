
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
initial_speed = v_max;   % Il veicolo parte a velocità massima

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
                    middle_time = (overlap_start + overlap_end) / 2;
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
[t_seg, d_seg, v_seg] = simulate_vehicle_segmented(opt_nodes, dt, traffic_lights, initial_speed, accel);
figure;
subplot(2,1,1);
plot(t_seg, d_seg, 'm-', 'LineWidth', 2);
xlabel('Tempo (s)'); ylabel('Distanza (m)');
title('Traiettoria simulata (velocità costante fra incroci)');
grid on;
subplot(2,1,2);
plot(t_seg, v_seg, 'c-', 'LineWidth', 2);
xlabel('Tempo (s)'); ylabel('Velocità (m/s)');
title('Andamento della velocità (varia in prossimità degli incroci)');
grid on;

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

% Funzione per simulare il veicolo lungo il percorso ottimo (opt_nodes)
% con velocità costante negli intervalli, introducendo una fase di decelerazione
% se la velocità corrente supera quella richiesta per raggiungere in tempo il crossing.
% Il veicolo utilizza l'accelerazione "accel" (per decelerare) e, se serve, attende.
function [t_sim, d_sim, v_sim] = simulate_vehicle_segmented(opt_nodes, dt, traffic_lights, init_speed, accel)
    t_sim = []; d_sim = []; v_sim = [];
    currentTime = 0; currentDist = 0;
    currentSpeed = init_speed;
    
    t_sim(end+1) = currentTime; d_sim(end+1) = currentDist; v_sim(end+1) = currentSpeed;
    
    for i = 2:length(opt_nodes)
        targetTime = opt_nodes(i).t;
        targetDist = opt_nodes(i).d;
        T_seg = targetTime - currentTime;
        D_seg = targetDist - currentDist;
        
        % Calcola la velocità "naturale" richiesta per percorrere il segmento in tempo
        v_req = D_seg / T_seg;
        
        if currentSpeed > v_req
            % Simula fase di decelerazione da currentSpeed a v_req con decelerazione costante -accel
            t_dec = (currentSpeed - v_req)/accel;
            d_dec = currentSpeed*t_dec - 0.5*accel*t_dec^2;
            % Dopo la decelerazione, percorrere il rimanente a velocità v_req
            d_rem = D_seg - d_dec;
            t_const = d_rem / v_req;
            t_total = t_dec + t_const;
            
            % Se t_total è inferiore al tempo previsto, inserisci attesa
            t_wait = max(0, T_seg - t_total);
            
            % Simula decelerazione
            t_vec_dec = currentTime:dt:(currentTime + t_dec);
            if t_vec_dec(end) < currentTime+t_dec, t_vec_dec(end+1) = currentTime+t_dec; end
            v_vec_dec = currentSpeed - accel*(t_vec_dec - currentTime);
            d_vec_dec = currentDist + currentSpeed*(t_vec_dec - currentTime) - 0.5*accel*(t_vec_dec - currentTime).^2;
            
            % Simula corsa a velocità costante v_req
            t_start_const = t_vec_dec(end);
            t_vec_const = t_start_const:dt:(t_start_const + t_const);
            if t_vec_const(end) < t_start_const+t_const, t_vec_const(end+1) = t_start_const+t_const; end
            d_start_const = d_vec_dec(end);
            d_vec_const = d_start_const + v_req*(t_vec_const - t_start_const);
            
            % Simula attesa (velocità zero)
            t_start_wait = t_vec_const(end);
            t_vec_wait = t_start_wait:dt:(t_start_wait+t_wait);
            if t_vec_wait(end) < t_start_wait+t_wait, t_vec_wait(end+1) = t_start_wait+t_wait; end
            d_vec_wait = d_vec_const(end)*ones(size(t_vec_wait));
            v_vec_wait = zeros(size(t_vec_wait));
            
            % Aggiorna vettori per il segmento
            t_seg_vect = [t_vec_dec(2:end) t_vec_const(2:end) t_vec_wait(2:end)];
            d_seg_vect = [d_vec_dec(2:end) d_vec_const(2:end) d_vec_wait(2:end)];
            v_seg_vect = [v_vec_dec(2:end) v_vec_const(2:end) v_vec_wait(2:end)];
            
            currentSpeed = v_req; % al termine la velocità è v_req
            currentTime = t_seg_vect(end);
            currentDist = d_seg_vect(end);
        else
            % Se currentSpeed <= v_req, il veicolo parte a velocità costante v_req
            t_vec = currentTime:dt:targetTime;
            if t_vec(end) < targetTime, t_vec(end+1) = targetTime; end
            d_vec = currentDist + v_req*(t_vec - currentTime);
            v_vec = repmat(v_req, size(t_vec));
            currentSpeed = v_req;
            currentTime = t_vec(end);
            currentDist = d_vec(end);
            
            t_seg_vect = t_vec(2:end);
            d_seg_vect = d_vec(2:end);
            v_seg_vect = v_vec(2:end);
        end
        
        % Quando si è ad un incrocio, controlla il semaforo
        if opt_nodes(i).int > 0 && opt_nodes(i).int <= length(traffic_lights)
            light = traffic_lights(opt_nodes(i).int);
            if ~is_green(light, currentTime)
                t_wait2 = next_green(light, currentTime) - currentTime;
                t_vec_wait2 = currentTime:dt:(currentTime+t_wait2);
                if t_vec_wait2(end) < currentTime+t_wait2, t_vec_wait2(end+1) = currentTime+t_wait2; end
                d_vec_wait2 = currentDist * ones(size(t_vec_wait2));
                v_vec_wait2 = zeros(size(t_vec_wait2));
                t_seg_vect = [t_seg_vect, t_vec_wait2(2:end)];
                d_seg_vect = [d_seg_vect, d_vec_wait2(2:end)];
                v_seg_vect = [v_seg_vect, v_vec_wait2(2:end)];
                currentTime = t_vec_wait2(end);
            end
        end
        
        % Accumula i dati di simulazione
        t_sim = [t_sim, t_seg_vect];
        d_sim = [d_sim, d_seg_vect];
        v_sim = [v_sim, v_seg_vect];
    end
end