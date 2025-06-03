function [plotonsData] = multi_optimizer(vehicles, traffic_lights, v_min, v_max, ...
       b1, b2, t_final, init_time, init_positions)
% MULTI_OPTIMIZER:
% 1) Calcola la traiettoria ottimale del leader (usando sub_optimizer)
% 2) Per ogni follower, genera la traiettoria vincolata spostandola in base a una distanza iniziale
% 3) Verifica se i follower passano col rosso
% 4) Se c'è violazione, spezza il plotone e richiama ricorsivamente multi_optimizer
% 5) Restituisce una struttura (o array di strutture) con i dati di tutti i sottoplotoni

    % Calcolo della traiettoria del leader
    leaderID = vehicles(1);
    [optT_leader, optD_leader, cost, path_leader] = sub_optimizer(...
        leaderID, traffic_lights, v_min, v_max, b1, b2, init_time, t_final );
    
    disp(['>>> Leader=', num2str(leaderID), ', Costo ottimo=', num2str(cost, '%.3f')]);

    % Generazione delle traiettorie per i follower (esempio semplificato)
    nVehicles = length(vehicles);
    followerTrajectories = cell(1, nVehicles-1);
    for i = 2:nVehicles
        followerID = vehicles(i);
        % La differenza in posizione iniziale rispetto al leader
        distShift  = init_positions(followerID) - init_positions(leaderID);
        followerTrajectories{i-1}.vehicleID = followerID;
        [followerT, followerD] = build_follower_trajectory(optT_leader, optD_leader, distShift);
        followerTrajectories{i-1}.t = followerT;
        followerTrajectories{i-1}.d = followerD;
    end

    % Creazione della struttura del plotone corrente
    currentPloton.vehicles     = vehicles;
    currentPloton.leader       = leaderID;
    currentPloton.optT_leader  = optT_leader;
    currentPloton.optD_leader  = optD_leader;
    currentPloton.followerData = followerTrajectories;
    currentPloton.pathLeader   = path_leader;
    currentPloton.costLeader   = cost;
    currentPloton.traffic_lights = traffic_lights;  % Campo per i plot

    % Aggiungo i campi "t" ed "x" per la compatibilità con funzioni di plotting precedenti
    currentPloton.t = optT_leader;
    % Calcolo la velocità del leader in modo corretto
    vel_leader = gradient(optD_leader(:)) ./ gradient(optT_leader(:));
    x = zeros(length(optT_leader), 2*nVehicles);
    x(:, 1) = optD_leader(:);          % posizione leader
    x(:, nVehicles+1) = vel_leader(:);   % velocità leader
    for i = 2:nVehicles
        ft = followerTrajectories{i-1}.t;
        fd = followerTrajectories{i-1}.d;
        for i = 2:nVehicles
    ft = followerTrajectories{i-1}.t(:);
    fd = followerTrajectories{i-1}.d(:);
    if numel(ft) < 2
        % Se c'è un solo campione, replicalo per tutti i tempi del leader
        x(:, i) = repmat(fd, length(optT_leader), 1);
    else
        x(:, i) = interp1(ft, fd, optT_leader(:), 'linear', 'extrap');
    end
    % Calcolo della velocità del follower tramite gradient (semplice, in questo caso)
    x(:, nVehicles+i) = gradient(x(:, i)) ./ gradient(optT_leader(:));
end
        % Calcolo della velocità del follower in modo corretto
        x(:, nVehicles+i) = gradient(x(:, i)) ./ gradient(optT_leader(:));
    end
    currentPloton.x = x;
    
    % Verifica delle violazioni (passaggio al rosso) per ogni veicolo
    splittedPlotons = multi_check_red_light_violations(currentPloton, traffic_lights);

    if ~isempty(splittedPlotons)
        plotonsData = [];
        for sp = 1:length(splittedPlotons)
            subPlotonInfo = splittedPlotons{sp};
            newVehicles   = subPlotonInfo.vehicles;
            newInitPos    = subPlotonInfo.initPositions;
            newInitTime   = subPlotonInfo.initTime;
            % Chiamata ricorsiva per il nuovo plotone (richiama la stessa funzione)
            plotonsDataLocal = multi_optimizer(newVehicles, traffic_lights, v_min, v_max, ...
                                     b1, b2, t_final, newInitTime, newInitPos);
            plotonsData = [plotonsData, plotonsDataLocal]; %#ok<AGROW>
        end
    else
        plotonsData = currentPloton;
    end
end

%% SUB-FUNZIONI

function [opt_t, opt_d, cost, path] = sub_optimizer( ...
    leaderID, traffic_lights, v_min, v_max, b1, b2, init_time, tf )
% SUB_OPTIMIZER: calcolo ottimale per il leader utilizzando il grafo dei nodi
    [Nodes, Edges] = build_nodes_and_edges(traffic_lights, v_min, v_max, b1, b2, init_time, tf);
    [path, cost] = dijkstra(Nodes, Edges, 1, length(Nodes));
    opt_nodes = Nodes(path);
    opt_t = arrayfun(@(n)n.t, opt_nodes);
    opt_d = arrayfun(@(n)n.d, opt_nodes);
end

function [Nodes, Edges] = build_nodes_and_edges(traffic_lights, v_min, v_max, ...
        b1, b2, init_time, tf)
% BUILD_NODES_AND_EDGES: costruisce nodi e archi ispirandosi alla logica del vecchio optimizer
%
% Ogni nodo ha:
%   id - identificativo
%   t  - tempo (con offset init_time)
%   d  - distanza (in base alla posizione del semaforo)
%   int - indice dell'intersezione (0 per inizio, n+1 per fine)
    final_distance = traffic_lights(end).distance;
    [t_min, t_max] = velocity_pruning(traffic_lights, tf - init_time, final_distance, v_min, v_max);
    
    nodeId = 1;
    Nodes(nodeId) = struct('id', nodeId, 't', init_time, 'd', 0, 'int', 0);
    nodeId = nodeId + 1;
    
    nIntersections = length(traffic_lights);
    for i = 1:nIntersections
        curr_light = traffic_lights(i);
        nCycles = ceil((tf - init_time) / curr_light.cycle_time);
        for j = 0:nCycles
            cycle_start = j * curr_light.cycle_time + curr_light.offset;
            if curr_light.green_start <= curr_light.green_end
                abs_green_start = cycle_start + curr_light.green_start;
                abs_green_end   = cycle_start + curr_light.green_end;
                if abs_green_start <= tf
                    overlap_start = max(abs_green_start, t_min(i));
                    overlap_end   = min(abs_green_end, t_max(i));
                    if overlap_start < overlap_end
                        middle_time = (overlap_start + overlap_end) / 2;
                        Nodes(nodeId) = struct('id', nodeId, 't', middle_time + init_time, 'd', curr_light.distance, 'int', i);
                        nodeId = nodeId + 1;
                    end
                end
            else
                abs_green_start_1 = cycle_start + curr_light.green_start;
                abs_green_end_1   = cycle_start + curr_light.cycle_time;
                if abs_green_start_1 <= tf
                    overlap_start = max(abs_green_start_1, t_min(i));
                    overlap_end   = min(abs_green_end_1, t_max(i));
                    if overlap_start < overlap_end
                        mid_t = (overlap_start + overlap_end)/2;
                        Nodes(nodeId) = struct('id', nodeId, 't', mid_t + init_time, 'd', curr_light.distance, 'int', i);
                        nodeId = nodeId + 1;
                    end
                end
                abs_green_start_2 = cycle_start;
                abs_green_end_2   = cycle_start + curr_light.green_end;
                if abs_green_end_2 <= tf
                    overlap_start = max(abs_green_start_2, t_min(i));
                    overlap_end   = min(abs_green_end_2, t_max(i));
                    if overlap_start < overlap_end
                        mid_t = (overlap_start + overlap_end)/2;
                        Nodes(nodeId) = struct('id', nodeId, 't', mid_t + init_time, 'd', curr_light.distance, 'int', i);
                        nodeId = nodeId + 1;
                    end
                end
            end
        end
    end
    % Nodo finale
    Nodes(nodeId) = struct('id', nodeId, 't', tf, 'd', final_distance, 'int', nIntersections+1);
    
    % Costruzione degli archi
    Edges = struct('from', {}, 'to', {}, 'w', {});
    edgeCount = 1;
    nNodes = length(Nodes);
    for i = 1:nNodes
        lvlA = Nodes(i).int;
        for j = 1:nNodes
            lvlB = Nodes(j).int;
            if (lvlB == lvlA + 1) && (Nodes(j).t > Nodes(i).t) && (Nodes(j).d > Nodes(i).d)
                if (lvlB <= nIntersections)
                    if ~is_green(traffic_lights(lvlB), Nodes(j).t)
                        continue;
                    end
                end
                delta_t = Nodes(j).t - Nodes(i).t;
                delta_d = Nodes(j).d - Nodes(i).d;
                v_link = delta_d / delta_t;
                if (v_link >= v_min) && (v_link <= v_max)
                    cost_edge = delta_t * (b1 * v_link + b2 * v_link^2);
                    Edges(edgeCount) = struct('from', Nodes(i).id, 'to', Nodes(j).id, 'w', cost_edge);
                    edgeCount = edgeCount + 1;
                end
            end
        end
    end
end

function [followerT, followerD] = build_follower_trajectory(leaderT, leaderD, distShift)
% BUILD_FOLLOWER_TRAJECTORY: genera la traiettoria del follower tramite shift
    followerT = leaderT;
    followerD = leaderD - distShift;
end

function splittedPlotons = multi_check_red_light_violations(currentPloton, traffic_lights)
% MULTI_CHECK_RED_LIGHT_VIOLATIONS: controlla se un veicolo passa al rosso
    splittedPlotons = {};
    vehicles = currentPloton.vehicles;
    leaderT = currentPloton.optT_leader;
    leaderD = currentPloton.optD_leader;
    followerData = currentPloton.followerData;
    
    for i = 1:length(vehicles)
        vehID = vehicles(i);
        if vehID == currentPloton.leader
            tVeh = leaderT; 
            dVeh = leaderD;
        else
            allFollowers = [followerData{:}];
            idx = find([allFollowers.vehicleID] == vehID, 1);
            tVeh = followerData{idx}.t;
            dVeh = followerData{idx}.d;
        end
        [lightID, crossing_time] = check_single_vehicle_violations(tVeh, dVeh, traffic_lights);
        if ~isempty(lightID)
            disp(['[WARNING] Veicolo ', num2str(vehID), ' passa col rosso a incrocio ', num2str(lightID), ' (t=', num2str(crossing_time,'%.2f'), ')']);
            splittedPlotons = do_split(currentPloton, vehID, crossing_time);
            return;
        end
    end
end

function [lightID, crossing_time] = check_single_vehicle_violations(tVeh, dVeh, traffic_lights)
% CHECK_SINGLE_VEHICLE_VIOLATIONS: determina se e quando il veicolo passa il semaforo rosso
    lightID = [];
    crossing_time = [];
    for L = 1:length(traffic_lights)
        dist_light = traffic_lights(L).distance;
        cross_idx = find(dVeh(1:end-1) < dist_light & dVeh(2:end) >= dist_light, 1);
        if ~isempty(cross_idx)
            crossing_time_temp = tVeh(cross_idx);
            if ~is_green(traffic_lights(L), crossing_time_temp)
                lightID = L;
                crossing_time = crossing_time_temp;
                return;
            end
        end
    end
end

function splittedPlotons = do_split(currentPloton, violatingVehicle, crossing_time)
% DO_SPLIT: divide il plotone corrente in sottoplotoni a partire dal veicolo che ha violato il rosso
    splittedPlotons = {};
    vehicles = currentPloton.vehicles;
    idxViolator = find(vehicles == violatingVehicle, 1);
    vehNewPloton = vehicles(idxViolator:end);
    vehOldPloton = vehicles(1:idxViolator-1);
    
    if isempty(vehOldPloton)
        splittedPlotons{1} = struct('vehicles', vehNewPloton, ...
            'initPositions', quick_positions(currentPloton, vehNewPloton, crossing_time), ...
            'initTime', crossing_time);
    elseif isempty(vehNewPloton)
        splittedPlotons{1} = struct('vehicles', vehOldPloton, ...
            'initPositions', quick_positions(currentPloton, vehOldPloton, crossing_time), ...
            'initTime', crossing_time);
    else
        splittedPlotons{1} = struct('vehicles', vehOldPloton, ...
            'initPositions', quick_positions(currentPloton, vehOldPloton, crossing_time), ...
            'initTime', crossing_time);
        splittedPlotons{2} = struct('vehicles', vehNewPloton, ...
            'initPositions', quick_positions(currentPloton, vehNewPloton, crossing_time), ...
            'initTime', crossing_time);
    end
end

function initPos = quick_positions(currentPloton, subsetVehicles, crossingTime)
% QUICK_POSITIONS: calcola le posizioni dei veicoli del subset al momento dello splitting
    initPos = zeros(1, max(subsetVehicles));
    vehicles = currentPloton.vehicles;
    for i = 1:length(subsetVehicles)
        v = subsetVehicles(i);
        if v == currentPloton.leader
            tVeh = currentPloton.optT_leader;
            dVeh = currentPloton.optD_leader;
        else
            idx = find([currentPloton.followerData{:}.vehicleID] == v, 1);
            tVeh = currentPloton.followerData{idx}.t;
            dVeh = currentPloton.followerData{idx}.d;
        end
        posAtSplit = interp1(tVeh, dVeh, crossingTime, 'linear', 'extrap');
        initPos(v) = posAtSplit;
    end
end

%% FUNZIONI DI SUPPORTO

function [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max)
% VELOCITY_PRUNING: calcola gli intervalli di tempo validi per ciascun semaforo
    n = length(traffic_lights);
    d = [traffic_lights.distance];
    t_min = zeros(1, n);
    t_max = zeros(1, n);
    t_min(1) = d(1) / v_max;
    t_max(1) = d(1) / v_min;
    t_min(1) = next_green(traffic_lights(1), t_min(1));
    t_max(1) = prev_green(traffic_lights(1), t_max(1));
    for i = 2:n
        dist_inc = d(i) - d(i-1);
        t_min(i) = t_min(i-1) + dist_inc / v_max;
        t_max(i) = t_max(i-1) + dist_inc / v_min;
        t_max(i) = min(t_max(i), tf - (final_distance - d(i)) / v_max);
        t_min(i) = next_green(traffic_lights(i), t_min(i));
        t_max(i) = prev_green(traffic_lights(i), t_max(i));
    end
    for i = n:-1:2
        needed_t = (d(i) - d(i-1)) / v_max;
        if t_max(i) > t_max(i-1) + needed_t
            t_max(i-1) = t_max(i) - needed_t;
            t_max(i-1) = prev_green(traffic_lights(i-1), t_max(i-1));
        end
    end
end

function status = is_green(light, time)
% IS_GREEN: verifica se il semaforo è verde al tempo specificato
    t_in_cycle = mod(time - light.offset, light.cycle_time);
    if light.green_start <= light.green_end
        status = (t_in_cycle >= light.green_start) && (t_in_cycle < light.green_end);
    else
        status = (t_in_cycle >= light.green_start) || (t_in_cycle < light.green_end);
    end
end

function t_next = next_green(light, time)
% NEXT_GREEN: calcola il successivo istante in cui il semaforo diventa verde
    if is_green(light, time)
        t_next = time;
    else
        cyc = mod(time - light.offset, light.cycle_time);
        if cyc < light.green_start
            t_next = time + (light.green_start - cyc);
        else
            t_next = time + (light.cycle_time - cyc) + light.green_start;
        end
    end
end

function t_prev = prev_green(light, time)
% PREV_GREEN: calcola l'istante precedente in cui il semaforo era verde
    if is_green(light, time)
        t_prev = time;
    else
        cyc = mod(time - light.offset, light.cycle_time);
        if cyc >= light.green_end
            t_prev = time - (cyc - light.green_end);
        else
            t_prev = time - cyc - (light.cycle_time - light.green_end);
        end
    end
end

function [path, cost] = dijkstra(Nodes, Edges, source, target)
% DIJKSTRA: algoritmo per trovare il percorso a costo minimo sul grafo
    nNodes = length(Nodes);
    cost = inf(1, nNodes);
    prev = nan(1, nNodes);
    cost(source) = 0;
    Q = 1:nNodes;
    while ~isempty(Q)
        [~, idx] = min(cost(Q));
        u = Q(idx);
        Q(Q == u) = [];
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
    if ~isnan(prev(target)) || target == source
        u = target;
        while ~isnan(u)
            path = [u, path];
            u = prev(u);
        end
    end
end