%% Script principale
clear;
clc;
close all;

global SIM_RUNS N_PLATOON;
SIM_RUNS = {};
N_PLATOON = 1;

disp('=== Avvio prima simulazione con Leader=1 ===');
reset_persistent_variables();
run_optimizer_and_plot(1, 0);
final_plot();
plot_delta_velocities();
plot_velocity_trigger_per_vehicle();


%% =========================================================================
%% Funzioni locali
%% =========================================================================
function run_optimizer_and_plot(leader_vehicle, time_offset, start_position)
    if nargin < 3
         start_position = 0;
    end
    global SIM_RUNS N_PLATOON;
    SIM_RUNS = {};
    N_PLATOON = 1;
     clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear check_velocity_triggers
    clear get_current_v_target_indexed
    
    % Pulizia variabili persistenti
    reset_persistent_variables();
    
    % Parametri e soglie
    final_time     = 150;         
    final_distance = 1800;    
    T              = 30;                   
    tf             = final_time;
    v_min          = 5;   
    v_max          = 30;  
    b1             = 0.1;  
    b2             = 0.01;
    b3             = 10;
    b4             = 4;
    
    delta_func = @(t) -0 * (b1 + b2*(sin((1/b3)*t+b4) + 0.25*rand));
    
    fprintf('\n[INFO] run_optimizer_and_plot(Leader=%d, offset=%.2f, start_pos=%.2f)\n', ...
        leader_vehicle, time_offset, start_position);
    
    % Definizione semafori
    traffic_lights = [
         create_traffic_light(300,   0, 10, T)
         create_traffic_light(600,  10, 15, T)
         create_traffic_light(900,  20, 30, T)
         create_traffic_light(1200,  0, 10, T)
         create_traffic_light(1550, 10, 20, T)
     ];
     
    % Esempio di pruning
    [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance]; 
    nIntersections = length(traffic_lights);
    
    % Costruzione nodi: ora il nodo iniziale usa start_position e time_offset
    Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
    nodeId = 1;
    Nodes(nodeId) = struct('id', nodeId, 't', time_offset, 'd', start_position, 'int', 0); 
    nodeId = nodeId + 1;
    
    for i = 1:nIntersections
        light = traffic_lights(i);
        for k = 0:ceil(tf / light.cycle_time)
            cycle_start = k * light.cycle_time + light.offset;
            if light.green_start <= light.green_end
                abs_green_start = cycle_start + light.green_start;
                abs_green_end   = cycle_start + light.green_end;
                if abs_green_start <= tf + time_offset
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
                if abs_green_start_1 <= tf + time_offset
                    ov_start = max(abs_green_start_1, t_min(i));
                    ov_end   = min(abs_green_end_1, t_max(i));
                    if ov_start < ov_end
                        mid_t = (ov_start + ov_end) / 2; 
                        Nodes(nodeId) = struct('id', nodeId, 't', mid_t, 'd', d(i), 'int', i);
                        nodeId = nodeId + 1;
                    end
                end
                abs_green_start_2 = cycle_start;
                abs_green_end_2   = cycle_start + light.green_end;
                if abs_green_end_2 <= tf + time_offset
                    ov_start = max(abs_green_start_2, t_min(i));
                    ov_end   = min(abs_green_end_2, t_max(i));
                    if ov_start < ov_end
                        mid_t = (ov_start + ov_end) / 2;
                        Nodes(nodeId) = struct('id', nodeId, 't', mid_t, 'd', d(i), 'int', i);
                        nodeId = nodeId + 1;
                    end
                end
            end
        end
    end
    % Nodo finale
    Nodes(nodeId) = struct('id', nodeId, 't', time_offset + tf, 'd', final_distance, 'int', nIntersections + 1);
    nNodes = nodeId;
    
    % Costruzione archi
    Edges = struct('from', {}, 'to', {}, 'w', {});
    edgeCount = 1;
    for i = 1:nNodes
        lvlA = Nodes(i).int;
        for j = 1:nNodes
            lvlB = Nodes(j).int;
            if lvlB == lvlA + 1
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
                        E_link = delta_t * (b1 * v_link + b2 * v_link^2);
                        Edges(edgeCount) = struct('from', Nodes(i).id, 'to', Nodes(j).id, 'w', E_link);
                        edgeCount = edgeCount + 1;
                    end
                end
            end
        end
    end
    
    [path, cost] = dijkstra(Nodes, Edges, 1, nNodes);
    
    if isinf(cost)
        error('[ERROR] Nessun percorso valido trovato tramite l''ottimizzazione.');
    end
    
    opt_nodes = Nodes(path);
    opt_t = arrayfun(@(n) n.t, opt_nodes);
    opt_d = arrayfun(@(n) n.d, opt_nodes);
    
    fprintf('>>> Leader=%.1f, Costo ottimo=%.3f\n', leader_vehicle, cost);
    
    speeds = zeros(1, length(opt_t) - 1);
    for k = 1:(length(opt_t) - 1)
        d_ = opt_d(k + 1) - opt_d(k);
        t_ = opt_t(k + 1) - opt_t(k);
        speeds(k) = d_ / t_;
    end
    
    n_vehicles = 5;
    m_vehicles = 1000 * ones(1, n_vehicles);
    v_targets = speeds;  
    
    % Parametri PID
    K_p_speed = 7000; K_i_speed = 0; K_d_speed = 0.1;
    K_p_dist  = 2000; K_i_dist  = 0.8; K_d_dist  = 0.4;
    t_CTH     = 1.5;  
    d_init    = 4;
    
    % Stato iniziale: il leader parte da start_position e i follower sono posizionati dietro
    x0 = zeros(2 * n_vehicles, 1);
    for i = 1:n_vehicles
        if i == leader_vehicle
            x0(i) = start_position;
        else
            x0(i) = start_position - d_init * (i - leader_vehicle);
        end
    end
    
    t_span = [time_offset, time_offset + tf];
    [t_sim, x_sim] = ode45(@(t,x) system_dynamics_new_platoon( ...
        t, x, n_vehicles, m_vehicles, delta_func, traffic_lights, v_targets, t_CTH, ...
        K_p_speed, K_i_speed, K_d_speed, K_p_dist, K_i_dist, K_d_dist, ...
        leader_vehicle, time_offset), t_span, x0);
    
    T_abs = t_sim;  % poichè t_sim include già l'offset
    SIM_RUNS{end+1} = struct( ...
        'leader', leader_vehicle, ...
        't', T_abs, ...
        'x', x_sim, ...
        'offset', time_offset, ...
        'traffic_lights', traffic_lights, ...
        'splittedVehicles', [], ...
        'v_targets', speeds, ...
        'opt_t', opt_t, ...
        'opt_d', opt_d);
    
    % Verifica semafori e trigger di velocità
    check_red_light_violations(T_abs, x_sim, traffic_lights, T);
    check_velocity_triggers(T_abs, x_sim, traffic_lights);
end

function dx = system_dynamics_new_platoon(t, x, n_vehicles, m, delta_func, ...
    traffic_lights, v_targets, t_CTH, K_p_speed, K_i_speed, K_d_speed, ...
    K_p_dist, K_i_dist, K_d_dist, leader_vehicle, time_offset)

    dx = zeros(2*n_vehicles, 1);
    persistent t_prev
    if isempty(t_prev), t_prev = t; end
    dt = t - t_prev;
    if dt <= 0, dt = 0.00001; end
    t_prev = t;

    persistent e_int_speed e_old_speed;
    persistent e_int_dist  e_old_dist;
    if isempty(e_int_speed)
       e_int_speed = 0; e_old_speed = 0;
    end
    if isempty(e_int_dist)
       e_int_dist = zeros(n_vehicles,1); e_old_dist = zeros(n_vehicles,1);
    end

    abs_t = t + time_offset;

    for i = 1:n_vehicles
        dx(i) = x(n_vehicles + i);   % aggiornamento posizione
        if i == leader_vehicle
            vt = get_current_v_target_indexed(x(leader_vehicle), traffic_lights, v_targets);
            vel_err = vt - x(n_vehicles + i);
            e_int_speed = e_int_speed + vel_err * dt;
            vel_deriv = (vel_err - e_old_speed) / dt;
            e_old_speed = vel_err;
            U_leader = K_p_speed * vel_err + K_i_speed * e_int_speed + K_d_speed * vel_deriv;
            % Per il leader non reintegro il nuovo valore, ma lascio il sistema reagire direttamente al PID
            dx(n_vehicles + i) = (U_leader + delta_func(abs_t)) / m(i);
            % Puoi eventualmente mostrare vt per debug:
            % fprintf('t=%.2f, vt=%.2f, v_act=%.2f\n',abs_t, vt, x(n_vehicles+i));
        else
            if i > 1
                dist = x(i-1) - x(i);
            else
                dist = 10;
            end
            v_cur = x(n_vehicles + i);
            d_min_val = 1;
            d_desired = d_min_val + t_CTH * v_cur;
            dist_err = dist - d_desired;
            e_int_dist(i) = e_int_dist(i) + dist_err * dt;
            dist_deriv = (dist_err - e_old_dist(i)) / dt;
            e_old_dist(i) = dist_err;
            U_dist = K_p_dist * dist_err + K_i_dist * e_int_dist(i) + K_d_dist * dist_deriv;
            dx(n_vehicles + i) = U_dist / m(i);
            % Impostiamo dx per i follower in maniera simile
        end
    end
end

function vt = get_current_v_target_indexed(x_leader, traffic_lights, v_targets)
    % Controllo di sicurezza sui parametri in ingresso
    if isempty(v_targets)
        fprintf('[WARNING] v_targets è vuoto, uso velocità predefinita 10 m/s\n');
        vt = 10;  % Velocità di sicurezza predefinita
        return;
    end
    
    idx = find(x_leader < [traffic_lights.distance], 1);
    if isempty(idx)
        % Se non ci sono semafori davanti, usa l'ultima velocità
        vt = v_targets(end);
    else
        % Assicurati che l'indice sia valido e positivo
        idx = min(max(1, idx), length(v_targets));
        vt = v_targets(idx);
    end
end

function check_red_light_violations(t_abs, x_sim, traffic_lights, T)
    persistent violations_detected
    if isempty(violations_detected), violations_detected = []; end

    n_vehicles = size(x_sim,2)/2;
    global SIM_RUNS
    current_run = length(SIM_RUNS);
    
    % Ottieni l'attuale leader
    current_leader = SIM_RUNS{current_run}.leader;
    
    % MODIFICA: Distanza di anticipo per il rilevamento
    look_ahead_distance = 30;  % Metri di anticipo per fermarsi prima del semaforo
    
    % Controlla ogni veicolo
    for v = 1:n_vehicles
        % Salta i veicoli che non fanno parte di questo plotone
        if ~is_in_current_platoon(v, current_run)
            continue;
        end
        
        pos_v = x_sim(:, v);
        v_vel = x_sim(:, n_vehicles + v);
        
        for L = 1:length(traffic_lights)
            light_d = traffic_lights(L).distance;
            
            % MODIFICA: Cerca veicoli che si avvicinano al semaforo
            approaching_idx = find(pos_v >= light_d - look_ahead_distance & pos_v < light_d, 1, 'last');
            
            if ~isempty(approaching_idx)
                approach_time = t_abs(approaching_idx);
                current_pos = pos_v(approaching_idx);
                current_vel = v_vel(approaching_idx);
                
                % Tempo stimato di arrivo al semaforo
                time_to_light = (light_d - current_pos) / max(current_vel, 0.1);
                estimated_crossing_time = approach_time + time_to_light;
                
                % Verifica se il semaforo sarà rosso quando il veicolo lo attraversa
                if ~is_green(traffic_lights(L), estimated_crossing_time)
                    % Verifica se questa violazione è già stata rilevata
                    if any(violations_detected == v)
                        continue;
                    end
                    
                    % Debug
                    fprintf('\n[WARNING] Veicolo %d sta per passare col rosso a incrocio %d (t=%.2f s)\n', v, L, approach_time);
                    fprintf('          Posizione attuale: %.2f m, Velocità: %.2f m/s\n', current_pos, current_vel);
                    fprintf('          Semaforo a: %.2f m, Tempo stimato attraversamento: %.2f s\n', light_d, estimated_crossing_time);
                    
                    violations_detected = [violations_detected, v];
                    
                    fprintf('>> Veicolo %d si ferma al semaforo e diventa leader di un nuovo plotone!\n', v);

                    % Aggiorna i veicoli nel plotone corrente
                    SIM_RUNS{current_run}.splittedVehicles = v:n_vehicles;

                    % Calcola quando il semaforo diventerà verde
                    next_green_time = next_green(traffic_lights(L), estimated_crossing_time);
                    
                    % MODIFICA: Usa la funzione di stop e attesa al semaforo
                    rerun_optimizer_from_traffic_light(v, next_green_time, light_d - 5, traffic_lights(L));
                    return;
                end
            end
        end
    end
end


function run_optimizer_from_traffic_light(leader_vehicle, start_time, start_position, ~)
    global SIM_RUNS N_PLATOON

    % Pulizia variabili persistent
    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear get_current_v_target_indexed
    
    % Parametri principali
    final_time     = 150;       % Finestra massima di simulazione
    final_distance = 1800;      % Distanza totale da coprire
    T              = 30;        % Orizzonte per i check di semafori
    tf             = final_time;
    v_min          = 5;         % Velocità minima
    v_max          = 30;        % Velocità massima
    b1             = 0.1;       % Parametri di costo
    b2             = 0.01;
    b3             = 10;
    b4             = 4;

    delta_func = @(t) -0 * (b1 + b2*(sin((1/b3)*t+b4) + 0.25*rand));
    
    fprintf('\n[INFO] run_optimizer_from_traffic_light(Leader=%d, tempo=%.2f, posizione=%.2f)\n', ...
        leader_vehicle, start_time, start_position);

    % Recupera i semafori dalla prima simulazione
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    
    % Filtra semafori ancora davanti al veicolo
    remaining_lights = [];
    for i = 1:length(traffic_lights)
        if traffic_lights(i).distance > start_position
            remaining_lights = [remaining_lights; traffic_lights(i)];
        end
    end
    
    % Se non ci sono semafori, costruzione semplificata di nodi/archi
    if isempty(remaining_lights)
        dist_to_final = final_distance - start_position;
        t_min_final = dist_to_final / v_max + start_time;
        t_max_final = dist_to_final / v_min + start_time;
        
        Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
        Nodes(1) = struct('id', 1, 't', start_time, 'd', start_position, 'int', 0);
        Nodes(2) = struct('id', 2, 't', start_time + tf, 'd', final_distance, 'int', 1);
        nNodes = 2;
        
        Edges = struct('from', {}, 'to', {}, 'w', {});
        delta_t = Nodes(2).t - Nodes(1).t;
        delta_d = Nodes(2).d - Nodes(1).d;
        v_link = delta_d / delta_t;
        E_link = delta_t * (b1*v_link + b2*v_link^2);
        Edges(1) = struct('from', 1, 'to', 2, 'w', E_link);
        
    else
        % Caso con semafori: calcolo t_min/t_max per ogni semaforo
        d = [remaining_lights.distance];
        nIntersections = length(remaining_lights);
        t_min = zeros(1, nIntersections);
        t_max = zeros(1, nIntersections);

        for i = 1:nIntersections
            dist_from_start = d(i) - start_position;
            if dist_from_start < 0
                % Semaforo già superato, fissa t_min=t_max=start_time
                t_min(i) = start_time; 
                t_max(i) = start_time;
            else
                % Limiti di velocità
                t_min(i) = start_time + dist_from_start / v_max;
                t_max(i) = start_time + dist_from_start / v_min;
                
                % Allinea ai cicli semaforici
                t_min(i) = next_green(remaining_lights(i), t_min(i));
                t_max(i) = prev_green(remaining_lights(i), t_max(i));
                
                % Limita t_max in base alla distanza finale
                t_max(i) = min(t_max(i), start_time + tf - (final_distance - d(i)) / v_max);
                
                % Evita t_max < t_min
                if t_max(i) < t_min(i)
                    t_max(i) = t_min(i);
                end
            end
        end
        
        % Costruzione dei nodi
        Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
        nodeId = 1;
        Nodes(nodeId) = struct('id', nodeId, 't', start_time, 'd', start_position, 'int', 0);
        nodeId = nodeId + 1;
        
        % Aggiunge nodi semaforo all'interno delle finestre verdi
        for i = 1:nIntersections
            light = remaining_lights(i);
            for k = 0:ceil(tf / light.cycle_time)
                cyc_start = k * light.cycle_time + light.offset;
                
                % Gestione intervallo verde (caso green_start <= green_end)
                if light.green_start <= light.green_end
                    abs_start = cyc_start + light.green_start;
                    abs_end   = cyc_start + light.green_end;
                    
                    if abs_start <= start_time + tf
                        overlap_start = max(abs_start, t_min(i));
                        overlap_end   = min(abs_end,   t_max(i));
                        if overlap_start < overlap_end
                            mid_time = (overlap_start + overlap_end) / 2;
                            Nodes(nodeId) = struct('id', nodeId, 't', mid_time, 'd', d(i), 'int', i);
                            nodeId = nodeId + 1;
                        end
                    end
                else
                    % Se il verde attraversa la fine del ciclo
                    abs_startA = cyc_start + light.green_start;
                    abs_endA   = cyc_start + light.cycle_time;
                    if abs_startA <= start_time + tf
                        ov_s = max(abs_startA, t_min(i));
                        ov_e = min(abs_endA,   t_max(i));
                        if ov_s < ov_e
                            mid_time = (ov_s + ov_e) / 2;
                            Nodes(nodeId) = struct('id', nodeId, 't', mid_time, 'd', d(i), 'int', i);
                            nodeId = nodeId + 1;
                        end
                    end
                    
                    abs_startB = cyc_start;
                    abs_endB   = cyc_start + light.green_end;
                    if abs_startB <= start_time + tf
                        ov_s = max(abs_startB, t_min(i));
                        ov_e = min(abs_endB,   t_max(i));
                        if ov_s < ov_e
                            mid_time = (ov_s + ov_e) / 2;
                            Nodes(nodeId) = struct('id', nodeId, 't', mid_time, 'd', d(i), 'int', i);
                            nodeId = nodeId + 1;
                        end
                    end
                end
            end
        end

        % Nodo finale
        Nodes(nodeId) = struct('id', nodeId, 't', start_time + tf, 'd', final_distance, 'int', nIntersections+1);
        nNodes = nodeId;
        
        % Costruzione archi
        Edges = struct('from', {}, 'to', {}, 'w', {});
        edgeCount = 1;
        for i = 1:nNodes
            lvlA = Nodes(i).int;
            for j = 1:nNodes
                lvlB = Nodes(j).int;
                % Consenti arco se passiamo dal nodo i al j con int "successivo"
                if lvlB == lvlA + 1
                    if (Nodes(j).t > Nodes(i).t) && (Nodes(j).d > Nodes(i).d)
                        % Check semaforo sul nodo di arrivo
                        if lvlB > 0 && lvlB <= nIntersections
                            if ~is_green(remaining_lights(lvlB), Nodes(j).t)
                                continue;
                            end
                        end
                        % Calcolo del costo
                        delta_t = Nodes(j).t - Nodes(i).t;
                        delta_d = Nodes(j).d - Nodes(i).d;
                        v_link = delta_d / delta_t;
                        if v_link >= v_min && v_link <= v_max
                            E_link = delta_t * (b1*v_link + b2*v_link^2);
                            Edges(edgeCount) = struct('from', Nodes(i).id, 'to', Nodes(j).id, 'w', E_link);
                            edgeCount = edgeCount + 1;
                        end
                    end
                end
            end
        end
    end

    % Dijkstra per trovare percorso migliore
    [path, cost] = dijkstra(Nodes, Edges, 1, nNodes);
    fprintf('>>> Leader=%d, Costo ottimo=%.3f\n', leader_vehicle, cost);

    % Se il percorso non è valido
    invalid_cost = isscalar(cost) && isinf(cost);
    invalid_path = isempty(path) || (length(path) < 2);
    if invalid_cost || invalid_path
        fprintf('[ERROR] Nessun percorso valido trovato. Creo un percorso predefinito sicuro.\n');
        opt_t = [start_time; start_time + 100];
        opt_d = [start_position; final_distance];
        speeds = [10]; % Velocità costante "sicura"
    else
        % Percorso valido
        opt_nodes = Nodes(path);
        opt_t = arrayfun(@(n)n.t, opt_nodes);
        opt_d = arrayfun(@(n)n.d, opt_nodes);
        speeds = zeros(1, length(path) - 1);
        for k = 1:(length(path) - 1)
            dd = opt_nodes(k+1).d - opt_nodes(k).d;
            dt = opt_nodes(k+1).t - opt_nodes(k).t;
            speeds(k) = max(1, dd / dt);
        end
    end
    if isempty(speeds), speeds = 10; end

    % Imposta lo stato iniziale per il nuovo plotone
    n_vehicles = size(SIM_RUNS{1}.x, 2) / 2;
    current_run = length(SIM_RUNS);
    previous_run = current_run;
    for i = current_run:-1:1
        if isfield(SIM_RUNS{i}, 'splittedVehicles') && ~isempty(SIM_RUNS{i}.splittedVehicles) ...
           && ismember(leader_vehicle, SIM_RUNS{i}.splittedVehicles)
            previous_run = i; break;
        end
    end
    platoon_vehicles = SIM_RUNS{previous_run}.splittedVehicles;
    
    m_vehicles = 1000 * ones(1, n_vehicles);
    v_targets = speeds;
    K_p_speed = 7000; K_i_speed = 0;   K_d_speed = 0.1;
    K_p_dist  = 2000; K_i_dist = 0.8; K_d_dist = 0.4;
    t_CTH = 1.5; d_init = 4;

    x0 = zeros(2*n_vehicles, 1);
    for i = 1:n_vehicles
        if ismember(i, platoon_vehicles)
            if i == leader_vehicle
                x0(i) = start_position;
            else
                idx_in_platoon = find(platoon_vehicles == i);
                idx_leader     = find(platoon_vehicles == leader_vehicle);
                pos_diff       = idx_in_platoon - idx_leader;
                x0(i) = start_position - d_init * pos_diff;
            end
            x0(n_vehicles + i) = 0; 
        else
            x0(i) = -1000;
            x0(n_vehicles + i) = 0;
        end
    end

    t_span = [start_time, start_time + tf];
    [t_sim, x_sim] = ode45(@(t,x) system_dynamics_new_platoon( ...
        t, x, n_vehicles, m_vehicles, delta_func, traffic_lights, v_targets, t_CTH, ...
        K_p_speed, K_i_speed, K_d_speed, K_p_dist, K_i_dist, K_d_dist, ...
        leader_vehicle, start_time), t_span, x0);

    SIM_RUNS{end+1} = struct( ...
        'leader', leader_vehicle, ...
        't', t_sim, ...
        'x', x_sim, ...
        'offset', start_time, ...
        'traffic_lights', traffic_lights, ...
        'splittedVehicles', [], ...
        'v_targets', speeds, ...
        'opt_t', opt_t, ...
        'opt_d', opt_d);

    check_red_light_violations(t_sim, x_sim, traffic_lights, T);
    check_velocity_triggers(t_sim, x_sim, traffic_lights);
end

function result = is_in_current_platoon(vehicle_id, current_run)
    global SIM_RUNS
    result = false;
    
    if current_run == 1
        % Per il primo run, verifica se il veicolo è stato splittato
        if isfield(SIM_RUNS{1}, 'splittedVehicles') && ...
           ~isempty(SIM_RUNS{1}.splittedVehicles) && ...
           ismember(vehicle_id, SIM_RUNS{1}.splittedVehicles)
            result = false;
        else
            result = true;
        end
        return;
    end
    
    % Per i run successivi, controlla se il veicolo è stato splittato dal run precedente
    prev_run = SIM_RUNS{current_run-1};
    if isfield(prev_run, 'splittedVehicles') && ...
       ~isempty(prev_run.splittedVehicles) && ...
       ismember(vehicle_id, prev_run.splittedVehicles)
        
        % Verifica che non sia stato splittato anche dal run corrente
        if isfield(SIM_RUNS{current_run}, 'splittedVehicles') && ...
           ~isempty(SIM_RUNS{current_run}.splittedVehicles) && ...
           ismember(vehicle_id, SIM_RUNS{current_run}.splittedVehicles)
            result = false;
        else
            result = true;
        end
    end
end

function rerun_optimizer_for_new_leader(violating_vehicle, T, stop_position)
    clear system_dynamics_new_platoon
    clear get_current_v_target_indexed
    
    global SIM_RUNS
    global N_PLATOON
    
    % Mostra informazioni sulla creazione del nuovo plotone
    current_platoon = N_PLATOON + 1;
    disp(['[INFO] Creazione NUOVO PLOTONE ' num2str(current_platoon) ' con LEADER=' ...
          num2str(violating_vehicle) ', riparto da tempo assoluto=' ...
          num2str(N_PLATOON*T) ', posizione=' num2str(stop_position)]);

    % Incrementa il contatore di plotoni prima di eseguire la simulazione
    start_offset = N_PLATOON*T;  
    N_PLATOON = N_PLATOON + 1;
    
    % Esegui la simulazione con il nuovo leader, partendo dalla posizione di stop
    run_optimizer_and_plot(violating_vehicle, start_offset, stop_position);
    
    % Pulisci lo stato delle violazioni per il nuovo plotone
    clear check_red_light_violations
end

function final_plot()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[final_plot] Nessun dato da plottare.');
        return;
    end
    
    % Manteniamo lo stesso flusso di plotting e aggiungiamo "hold on"
    % per consentire la sovrapposizione dei dati dei vari run.
    hold on;
    
    plot_optimal_trajectories_and_lights();
    hold on;
    plot_real_trajectories();
    hold on;
    plot_comparison();
    hold on;
    plot_leader_velocities();
    hold on;
    plot_energy_consumption();
    hold on;
    plot_inter_vehicle_distances();
end
function plot_optimal_trajectories_and_lights()
    global SIM_RUNS
    figure('Name','Posizioni Ottimali e Semafori', 'Position', [100, 100, 900, 600]);
    hold on;
    
    % Disegna semafori
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    [all_times, all_distances, all_colors] = prepare_traffic_light_data(traffic_lights);
    scatter(all_times, all_distances, 10, all_colors, 'filled', 'DisplayName', 'Semafori');
    
    % Prepara stili e colori
    markers = {'o','s','d','^','v','>','<'};
    colors = {'b','r','g','m','c','k',[0.8 0.4 0],[0.5 0.5 0.5],[0.2 0.6 0.8]};
    line_styles = {'-','--',':','-.'};
    legend_handles = [];
    legend_texts = {};
    
    % Per ogni run, disegna traiettorie ottimali
    for run_i = 1:length(SIM_RUNS)
        [legend_handles, legend_texts] = plot_run_optimal_trajectories(run_i, ...
                                          markers, colors, line_styles, ...
                                          legend_handles, legend_texts);
    end
    
    legend(legend_handles, legend_texts, 'Location', 'Best', 'NumColumns', 2);
    xlabel('Tempo [s]', 'FontSize', 12);
    ylabel('Posizione [m]', 'FontSize', 12);
    title('Traiettorie ottimali e semafori', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
end

function [legend_handles, legend_texts] = plot_run_optimal_trajectories(run_i, markers, colors, line_styles, legend_handles, legend_texts)
    global SIM_RUNS
    
    runData = SIM_RUNS{run_i};
    leader = runData.leader;
    
    if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d')
        return;
    end
    
    opt_t = runData.opt_t;
    opt_d = runData.opt_d;
    
    % Determina quali veicoli appartengono a questo plotone
    platoon_vehicles = get_platoon_vehicles(run_i);
    
    % Calcola velocità target
    v_targets = calculate_target_velocities(opt_t, opt_d);
    
    % Disegna la traiettoria del leader
    color_idx = mod(leader-1, length(colors))+1;
    line_idx = mod(run_i-1, length(line_styles))+1;
    marker_idx = mod(run_i-1, length(markers))+1;
    
    h = plot(opt_t, opt_d, [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 3);
    scatter(opt_t, opt_d, 70, colors{color_idx}, markers{marker_idx}, 'filled');
    
    legend_handles(end+1) = h;
    legend_texts{end+1} = ['Leader ' num2str(leader) ' (Plotone ' num2str(run_i) ')'];
    
    % Disegna le traiettorie ottimali dei follower
    for v = platoon_vehicles
        if v ~= leader
            follower_opt_d = calculate_follower_trajectory(v, leader, opt_t, opt_d, v_targets);
            
            follower_color_idx = mod(v-1, length(colors))+1;
            follower_line_idx = mod(run_i-1, length(line_styles))+1;
            follower_marker_idx = mod(v-1, length(markers))+1;
            
            h_follower = plot(opt_t, follower_opt_d, ...
                [colors{follower_color_idx}, line_styles{follower_line_idx}], 'LineWidth', 2);
            scatter(opt_t, follower_opt_d, 40, colors{follower_color_idx}, ...
                markers{follower_marker_idx}, 'filled');
            
            legend_handles(end+1) = h_follower;
            legend_texts{end+1} = ['Follower ' num2str(v) ' (Plotone ' num2str(run_i) ')'];
        end
    end
end

function plot_real_trajectories()
    global SIM_RUNS
    
    figure('Name', 'Grafico Traiettorie Reali', 'Position', [150, 150, 900, 600]);
    hold on;
    
    % Disegna semafori
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    [all_times, all_distances, all_colors] = prepare_traffic_light_data(traffic_lights);
    scatter(all_times, all_distances, 10, all_colors, 'filled');
    
    % Configurazione stili e colori
    colors = {'b','r','g','m','c','y','k'};
    line_styles = {'-','-',':','-.'};
    
    % Crea una legenda per i veicoli
    legend_handles = [];
    legend_texts = {};
    
    % Per ogni plotone
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        leader = runData.leader;
        
        % Ottieni i veicoli in questo plotone
        platoon_vehicles = get_platoon_vehicles(run_i);
        
        % Traccia traiettorie per i veicoli in questo plotone
        for v = platoon_vehicles
            color_idx = mod(v-1, length(colors))+1;
            line_idx = mod(run_i-1, length(line_styles))+1;
            
            h = plot(t, x(:,v), [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 2);
            
            % Aggiungi alla legenda
            if v == leader
                legend_texts{end+1} = ['Leader ' num2str(v) ' (Plotone ' num2str(run_i) ')'];
            else
                legend_texts{end+1} = ['Veicolo ' num2str(v) ' (Plotone ' num2str(run_i) ')'];
            end
            legend_handles(end+1) = h;
        end
    end
    
    legend(legend_handles, legend_texts, 'Location', 'Best', 'NumColumns', 2);
    xlabel('Tempo [s]', 'FontSize', 12);
    ylabel('Posizione [m]', 'FontSize', 12);
    title('Traiettorie reali e semafori', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
end

function plot_comparison()
    global SIM_RUNS
    
    figure('Name', 'Confronto Reale vs Ottimali + Semafori', 'Position', [200, 200, 1000, 600]);
    hold on;
    
    % Disegna semafori
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    [all_times, all_distances, all_colors] = prepare_traffic_light_data(traffic_lights);
    scatter(all_times, all_distances, 10, all_colors, 'filled');
    
    % Configurazione stili
    colors_real = {'b','r','g','m','c','y','k'};
    colors_opt = {'b','r','g','m','c','k',[0.8 0.4 0],[0.5 0.5 0.5],[0.2 0.6 0.8]};
    
    % Crea elementi per la legenda
    legend_handles = [];
    legend_texts = {};
    
    % Per ogni plotone, traccia le traiettorie reali e ottimali
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        leader = runData.leader;
        
        % Ottieni i veicoli di questo plotone
        platoon_vehicles = get_platoon_vehicles(run_i);
        
        % Traiettorie reali (linea continua)
        for v = platoon_vehicles
            color_idx = mod(v-1, length(colors_real))+1;
            line_idx = mod(run_i-1, length(colors_opt))+1;
            
            h_real = plot(t, x(:,v), [colors_real{color_idx}, '-'], 'LineWidth', 2);
            
            if v == leader
                legend_texts{end+1} = ['Leader ' num2str(v) ' (reale, Plotone ' num2str(run_i) ')'];
            else
                legend_texts{end+1} = ['Veicolo ' num2str(v) ' (reale, Plotone ' num2str(run_i) ')'];
            end
            legend_handles(end+1) = h_real;
        end
        
        % Traiettorie ottimali (linea tratteggiata)
        if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
            opt_t = runData.opt_t;
            opt_d = runData.opt_d;
            v_targets = calculate_target_velocities(opt_t, opt_d);
            
            % Leader ottimale
            color_idx = mod(leader-1, length(colors_opt))+1;
            h_opt_leader = plot(opt_t, opt_d, [colors_opt{color_idx}, '--'], 'LineWidth', 2);
            legend_texts{end+1} = ['Leader ' num2str(leader) ' (ottimale, Plotone ' num2str(run_i) ')'];
            legend_handles(end+1) = h_opt_leader;
            
            % Follower ottimali
            for v = platoon_vehicles
                if v ~= leader
                    follower_opt_d = calculate_follower_trajectory(v, leader, opt_t, opt_d, v_targets);
                    color_idx = mod(v-1, length(colors_opt))+1;
                    h_opt_follower = plot(opt_t, follower_opt_d, [colors_opt{color_idx}, '--'], 'LineWidth', 2);
                    legend_texts{end+1} = ['Veicolo ' num2str(v) ' (ottimale, Plotone ' num2str(run_i) ')'];
                    legend_handles(end+1) = h_opt_follower;
                end
            end
        end
    end
    
    legend(legend_handles, legend_texts, 'Location', 'Best', 'NumColumns', 2);
    xlabel('Tempo [s]', 'FontSize', 12);
    ylabel('Posizione [m]', 'FontSize', 12);
    title('Confronto Traiettorie (Reali vs Ottimali) + Semafori', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
end

function plot_leader_velocities()
    global SIM_RUNS
    
    % Identifica tutti i leader nelle simulazioni
    leaders = [];
    for run_i = 1:length(SIM_RUNS)
        leader = SIM_RUNS{run_i}.leader;
        if ~ismember(leader, leaders)
            leaders = [leaders, leader];
        end
    end
    
    % Crea un grafico per ogni leader
    for l = 1:length(leaders)
        current_leader = leaders(l);
        figure('Name', ['Velocità Leader ' num2str(current_leader)], 'Position', [250+l*50, 250+l*50, 800, 500]);
        hold on;
        
        legend_handles = [];
        legend_texts = {};
        
        for run_i = 1:length(SIM_RUNS)
            runData = SIM_RUNS{run_i};
            if runData.leader == current_leader
                t = runData.t;
                x = runData.x;
                nv = size(x, 2)/2;
                leader_velocity = x(:, nv + current_leader);
                
                h_real = plot(t, leader_velocity, 'b-', 'LineWidth', 2);
                legend_handles(end+1) = h_real;
                legend_texts{end+1} = ['Velocità reale (Plotone ' num2str(run_i) ')'];
                
                % Aggiungi velocità target se disponibili
                if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
                    opt_t = runData.opt_t;
                    opt_d = runData.opt_d;
                    opt_v = calculate_target_velocities(opt_t, opt_d);
                    
                    % Modifica qui: usa un approccio diverso per plottare le velocità target
                    for i = 1:length(opt_v)
                        t_segment = [opt_t(i), opt_t(i+1)];
                        v_segment = [opt_v(i), opt_v(i)];
                        if i == 1
                            h_opt = plot(t_segment, v_segment, 'r--', 'LineWidth', 2);
                            legend_handles(end+1) = h_opt;
                            legend_texts{end+1} = ['Velocità target (Plotone ' num2str(run_i) ')'];
                        else
                            plot(t_segment, v_segment, 'r--', 'LineWidth', 2);
                        end
                    end
                    
                    % Aggiungi i punti per chiarezza
                    scatter(opt_t(1:end-1), opt_v, 50, 'r', 'filled');
                end
            end
        end
        
        legend(legend_handles, legend_texts, 'Location', 'Best');
        xlabel('Tempo [s]', 'FontSize', 12);
        ylabel('Velocità [m/s]', 'FontSize', 12);
        title(['Velocità Leader ' num2str(current_leader)], 'FontSize', 14, 'FontWeight', 'bold');
        grid on;
        ylim([0, 35]);
    end
end
function plot_energy_consumption()
    global SIM_RUNS
    
    % Calcola il consumo energetico per ogni plotone
    figure('Name', 'Consumo Energetico', 'Position', [300, 300, 800, 500]);
    hold on;
    
    platoon_energy = zeros(1, length(SIM_RUNS));
    platoon_leaders = zeros(1, length(SIM_RUNS));
    platoon_vehicle_count = zeros(1, length(SIM_RUNS));
    
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        leader = runData.leader;
        platoon_leaders(run_i) = leader;
        
        % Parametri energetici
        b1 = 0.1;  
        b2 = 0.01;
        
        % Calcola il consumo energetico (approssimato)
        n_vehicles = size(x, 2)/2;
        total_energy = 0;
        
        % Determina i veicoli in questo plotone
        platoon_vehicles = get_platoon_vehicles(run_i);
        platoon_vehicle_count(run_i) = length(platoon_vehicles);
        
        for v = platoon_vehicles
            velocity = x(:, n_vehicles + v);
            dt = diff(t);
            
            % Calcola energia: E = Σ dt*(b1*v + b2*v^2)
            segment_energy = sum(dt .* (b1*velocity(1:end-1) + b2*velocity(1:end-1).^2));
            total_energy = total_energy + segment_energy;
        end
        
        platoon_energy(run_i) = total_energy;
    end
    
    % Visualizza il consumo energetico per platoon
    bar_h = bar(1:length(SIM_RUNS), platoon_energy);
    
    % Etichetta ogni barra con l'ID del leader e il numero di veicoli
    for i = 1:length(SIM_RUNS)
        text(i, platoon_energy(i) + max(platoon_energy)*0.03, ...
             ['Leader ' num2str(platoon_leaders(i)) ' (' num2str(platoon_vehicle_count(i)) ' veicoli)'], ...
             'HorizontalAlignment', 'center');
    end
    
    xlabel('Numero Plotone', 'FontSize', 12);
    ylabel('Consumo Energetico [J]', 'FontSize', 12);
    title('Consumo Energetico per Plotone', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
    set(gca, 'XTick', 1:length(SIM_RUNS));
end

function plot_inter_vehicle_distances()
    global SIM_RUNS
    
    % Figura per le distanze tra i veicoli in ogni plotone
    figure('Name', 'Distanze tra veicoli', 'Position', [350, 350, 1000, 600]);
    
    % Crea un subplot per ogni plotone
    n_plots = length(SIM_RUNS);
    rows = ceil(sqrt(n_plots));
    cols = ceil(n_plots/rows);
    
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        leader = runData.leader;
        
        % Crea subplot
        subplot(rows, cols, run_i);
        hold on;
        
        % Ottieni i veicoli di questo plotone
        platoon_vehicles = get_platoon_vehicles(run_i);
        
        % Calcola e visualizza le distanze tra i veicoli
        distances = [];
        labels = {};
        
        % Controlla se ci sono abbastanza veicoli nel plotone
        if length(platoon_vehicles) >= 2
            % Ordina i veicoli per posizione
            [~, order_idx] = sort(x(end, platoon_vehicles), 'descend');
            sorted_vehicles = platoon_vehicles(order_idx);
            
            % Calcola le distanze tra veicoli consecutivi
            for i = 1:length(sorted_vehicles)-1
                v1 = sorted_vehicles(i);
                v2 = sorted_vehicles(i+1);
                distance = x(:, v1) - x(:, v2);
                plot(t, distance, 'LineWidth', 2);
                distances(end+1) = distance(end);
                labels{end+1} = ['Distanza ' num2str(v1) '-' num2str(v2)];
            end
            
            title(['Distanze tra veicoli - Plotone ' num2str(run_i) ' (Leader ' num2str(leader) ')'], 'FontSize', 12);
            xlabel('Tempo [s]');
            ylabel('Distanza [m]');
            if ~isempty(labels)
                legend(labels, 'Location', 'Best');
            end
            grid on;
            
            % Controlla se distances ha valori validi prima di impostare i limiti dell'asse Y
            if ~isempty(distances) && all(isfinite(distances))
                ylim([0, max(max(distances), 10)]);
            else
                ylim([0, 10]); % Imposta un limite predefinito
            end
        else
            text(0.5, 0.5, 'Veicoli insufficienti per calcolare le distanze', 'HorizontalAlignment', 'center');
            axis([0 1 0 1]);
        end
    end
end

function rerun_optimizer_from_traffic_light(vehicle_id, start_time, start_position, traffic_light)
    clear system_dynamics_new_platoon
    clear get_current_v_target_indexed
    
    global SIM_RUNS
    global N_PLATOON
    
    % Mostra informazioni sulla creazione del nuovo plotone
    current_platoon = N_PLATOON + 1;
    disp(['[INFO] Creazione NUOVO PLOTONE ' num2str(current_platoon) ' con LEADER=' ...
          num2str(vehicle_id) ', attende semaforo verde a t=' ...
          num2str(start_time) ', posizione=' num2str(start_position)]);

    % Incrementa il contatore di plotoni
    N_PLATOON = N_PLATOON + 1;
    
    % Esegui la simulazione con il nuovo leader
    run_optimizer_from_traffic_light(vehicle_id, start_time, start_position, traffic_light);
    
    % Pulisci lo stato delle violazioni per il nuovo plotone
    clear check_red_light_violations
end

function platoon_vehicles = get_platoon_vehicles(run_i)
    global SIM_RUNS
    n_vehicles = size(SIM_RUNS{1}.x, 2)/2;
    
    if run_i == 1
        % Per il primo plotone, inizialmente include tutti i veicoli
        platoon_vehicles = 1:n_vehicles;
        
        % Ma esclude i veicoli che sono stati divisi
        if isfield(SIM_RUNS{1}, 'splittedVehicles') && ~isempty(SIM_RUNS{1}.splittedVehicles)
            platoon_vehicles = setdiff(platoon_vehicles, SIM_RUNS{1}.splittedVehicles);
        end
        return;
    end
    
    % Per i plotoni successivi
    current_leader = SIM_RUNS{run_i}.leader;
    
    % Trova i veicoli che appartengono a questo plotone (quelli divisi dal plotone precedente)
    prev_run = SIM_RUNS{run_i - 1};
    if isfield(prev_run, 'splittedVehicles') && ~isempty(prev_run.splittedVehicles)
        platoon_vehicles = prev_run.splittedVehicles;
        
        % Escludi i veicoli che sono stati ulteriormente divisi
        if isfield(SIM_RUNS{run_i}, 'splittedVehicles') && ~isempty(SIM_RUNS{run_i}.splittedVehicles)
            platoon_vehicles = setdiff(platoon_vehicles, SIM_RUNS{run_i}.splittedVehicles);
        end
    else
        % Se non ci sono veicoli divisi dal plotone precedente, il plotone è vuoto
        platoon_vehicles = [];
    end
end

function [all_times, all_distances, all_colors] = prepare_traffic_light_data(traffic_lights)
    max_time = 0;
    global SIM_RUNS
    for i=1:length(SIM_RUNS)
        max_time = max(max_time, max(SIM_RUNS{i}.t));
    end
    times = 0:ceil(max_time);
    
    all_times = [];
    all_distances = [];
    all_colors = [];
    for i=1:length(traffic_lights)
        for j=1:length(times)
            time = times(j);
            all_times = [all_times, time];
            all_distances = [all_distances, traffic_lights(i).distance];
            if is_green(traffic_lights(i), time)
                all_colors = [all_colors; [0, 1, 0]];
            else
                all_colors = [all_colors; [1, 0, 0]];
            end
        end
    end
end

function v_targets = calculate_target_velocities(opt_t, opt_d)
    v_targets = [];
    for i = 1:length(opt_t)-1
        v = (opt_d(i+1) - opt_d(i)) / (opt_t(i+1) - opt_t(i));
        v_targets(i) = v;
    end
end

function follower_opt_d = calculate_follower_trajectory(follower_id, leader_id, opt_t, opt_d, v_targets)
    follower_opt_d = zeros(size(opt_d));
    
    % Parametri per il calcolo delle distanze di sicurezza
    t_CTH = 1.5;  % Costante di tempo per la distanza di sicurezza
    d_min = 1;    % Distanza minima
    
    % Offset iniziale basato sulla differenza di ID
    initial_offset = abs(follower_id - leader_id);
    
    % Inizializza la posizione del follower
    if follower_id > leader_id
        follower_opt_d(1) = opt_d(1) - initial_offset;
    else
        follower_opt_d(1) = opt_d(1) + initial_offset;
    end
    
    % Calcola la traiettoria del follower
    for idx_opt = 2:length(opt_t)
        if follower_id > leader_id
            desired_gap = d_min + t_CTH*v_targets(idx_opt-1);
            follower_opt_d(idx_opt) = opt_d(idx_opt) - desired_gap*(follower_id-leader_id);
        else
            desired_gap = d_min + t_CTH*v_targets(idx_opt-1);
            follower_opt_d(idx_opt) = opt_d(idx_opt) + desired_gap*(leader_id-follower_id);
        end
    end
end

function reset_persistent_variables()
    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear check_velocity_triggers
    clear get_current_v_target_indexed
    clear next_green
    clear prev_green
    clear dijkstra
end

function light = create_traffic_light(distance, green_start, green_end, cycle_time)
    light.distance       = distance;
    light.green_start    = green_start;
    light.green_end      = green_end;
    light.cycle_time     = cycle_time;
    light.green_duration = green_end - green_start;
    light.offset         = mod(green_start, cycle_time);
end



function [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max)
    n= length(traffic_lights);
    d= [traffic_lights.distance];
    t_min=zeros(1,n); t_max=zeros(1,n);
    t_min(1)= d(1)/v_max; t_max(1)= d(1)/v_min;
    t_min(1)= next_green(traffic_lights(1), t_min(1));
    t_max(1)= prev_green(traffic_lights(1), t_max(1));
    for i=2:n
        dist_inc= d(i)- d(i-1);
        t_min(i)= t_min(i-1)+ dist_inc/v_max;
        t_max(i)= t_max(i-1)+ dist_inc/v_min;
        t_max(i)= min(t_max(i), tf-(final_distance-d(i))/v_max);
        t_min(i)= next_green(traffic_lights(i), t_min(i));
        t_max(i)= prev_green(traffic_lights(i), t_max(i));
    end
    for i=n:-1:2
        needed_t= (d(i)-d(i-1))/v_max;
        if t_max(i)> t_max(i-1)+ needed_t
            t_max(i-1)= t_max(i)- needed_t;
            t_max(i-1)= prev_green(traffic_lights(i-1), t_max(i-1));
        end
    end
end


function [t_min, t_max] = velocity_pruning_from_position(traffic_lights, tf, final_distance, ...
                                                        v_min, v_max, start_position, start_time)
    n = length(traffic_lights);
    d = [traffic_lights.distance];
    t_min = zeros(1, n); 
    t_max = zeros(1, n);

    % Calcolo in avanti
    for i = 1:n
        dist_from_start = d(i) - start_position;
        if dist_from_start < 0
            % Semaforo già superato, imposta un intervallo minimo e passa oltre
            t_min(i) = start_time;  
            t_max(i) = start_time;
            continue;
        end

        t_min(i) = start_time + dist_from_start/v_max;
        t_max(i) = start_time + dist_from_start/v_min;

        % Limite massimo basato sul tempo finale
        t_max(i) = min(t_max(i), tf - (final_distance - d(i))/v_max);

        % Allinea con i cicli del semaforo
        t_min(i) = next_green(traffic_lights(i), t_min(i));
        t_max(i) = prev_green(traffic_lights(i), t_max(i));

        % Evita che t_max(i) scenda sotto t_min(i)
        if t_max(i) < t_min(i)
            t_max(i) = t_min(i);
        end
    end

    % Propagazione all'indietro
    for i = n:-1:2
        needed_t = (d(i) - d(i-1))/v_max;
        if t_max(i) > t_max(i-1) + needed_t
            t_max(i-1) = t_max(i) - needed_t;
            t_max(i-1) = prev_green(traffic_lights(i-1), t_max(i-1));

            if t_max(i-1) < t_min(i-1)
                t_max(i-1) = t_min(i-1);
            end
        end
    end
end
function [t_min, t_max] = velocity_pruning_simple(start_position, final_distance, v_min, v_max)
    dist_to_final = final_distance - start_position;
    t_min = dist_to_final / v_max;
    t_max = dist_to_final / v_min;
end


function [path, cost] = dijkstra(Nodes, Edges, source, target)
    nNodes= length(Nodes);
    cost= inf(1,nNodes);
    prev= nan(1,nNodes);
    cost(source)=0;
    Q= 1:nNodes;
    while ~isempty(Q)
        [~, idx]= min(cost(Q));
        u= Q(idx);
        Q(Q==u)= [];
        if u==target, break; end
        for e= Edges
            if e.from==u
                v= e.to; 
                alt= cost(u)+ e.w;
                if alt< cost(v)
                    cost(v)= alt; prev(v)= u;
                end
            end
        end
    end
    path= [];
    if ~isnan(prev(target)) || target==source
        u= target;
        while ~isnan(u)
            path= [u, path];
            u= prev(u);
        end
    end
end

function plot_delta_velocities()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('Nessun dato disponibile per il plot dei delta delle velocità.');
        return;
    end
    
    runData = SIM_RUNS{1};
    t_sim = runData.t;
    n_vehicles = size(runData.x, 2) / 2;
    
    figure('Name','Valori per Calcolo Delta Velocità', 'Position', [100, 100, 1200, 800]);
    for v = 1:n_vehicles
        v_sim = runData.x(:, n_vehicles + v);
        if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d')
            continue;
        end
        opt_t = runData.opt_t;
        opt_d = runData.opt_d;
        pos_opt = interp1(opt_t, opt_d, t_sim, 'linear', 'extrap');
        v_opt = gradient(pos_opt, t_sim);
        offset_value = v_opt(end) - v_sim(end);
        diff_v = (v_opt - v_sim) - offset_value;
        
        subplot(n_vehicles, 1, v);
        hold on; grid on;
        plot(t_sim, v_sim, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulata');
        plot(t_sim, v_opt, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Ottimale');
        plot(t_sim, diff_v, 'r-.', 'LineWidth', 1.5, 'DisplayName', 'Delta (calcolato)');
        xlabel('Tempo [s]');
        ylabel('Velocità [m/s]');
        title(['Veicolo ' num2str(v) ': v_{sim}, v_{ottimale} e Delta']);
        legend('Location', 'best');
    end
end

function trigger_events = velocity_trigger(t, diff_v, opt_v)
    trigger_threshold = 5;
    disable_duration  = 10;
    rapid_change_thresh = 0.5;
    initial_delay = 5;
    
    trigger_events = zeros(size(t));
    disable_until = -inf;
    
    for i = 1:length(t)
        if t(i) < initial_delay || t(i) < disable_until
            trigger_events(i) = 0;
        else
            if abs(diff_v(i)) > trigger_threshold
                trigger_events(i) = 1;
            else
                trigger_events(i) = 0;
            end
        end
        
        if i > 1
            delta_opt = abs(opt_v(i) - opt_v(i-1));
            if delta_opt > rapid_change_thresh
                disable_until = t(i) + disable_duration;
            end
        end
    end
end

function plot_velocity_trigger_per_vehicle()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('Nessun dato da plottare per i trigger delle velocità.');
        return;
    end
    
    % Raccogliamo informazioni sugli split dei plotoni
    split_times = [];
    split_vehicles = [];
    split_reasons = {};
    
    for i = 1:length(SIM_RUNS)
        if i < length(SIM_RUNS)  % Se non è l'ultimo run
            if isfield(SIM_RUNS{i}, 'splittedVehicles') && ~isempty(SIM_RUNS{i}.splittedVehicles)
                next_leader = SIM_RUNS{i+1}.leader;
                split_time = SIM_RUNS{i+1}.offset;  % Il tempo di inizio del plotone successivo
                split_times(end+1) = split_time;
                split_vehicles(end+1) = next_leader;
                
                % Determina se lo split è avvenuto per semaforo rosso o trigger di velocità
                if split_time == i*30  % Se è un multiplo esatto del ciclo semaforico, probabilmente è per semaforo
                    split_reasons{end+1} = 'semaforo';
                else
                    split_reasons{end+1} = 'trigger';
                end
            end
        end
    end
    
    % Ora procediamo con il plotting normale
    runData = SIM_RUNS{1};
    t_sim = runData.t;
    n_vehicles = size(runData.x, 2) / 2;
    
    figure('Name', 'Trigger per Velocità e Split dei Plotoni', 'Position', [100, 100, 1200, 800]);
    
    for v = 1:n_vehicles
        v_sim = runData.x(:, n_vehicles + v);
        if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d')
            continue;
        end
        
        opt_t = runData.opt_t;
        opt_d = runData.opt_d;
        pos_opt = interp1(opt_t, opt_d, t_sim, 'linear', 'extrap');
        v_opt = gradient(pos_opt, t_sim);
        offset_value = v_opt(end) - v_sim(end);
        diff_v = (v_opt - v_sim) - offset_value;
        trigger_state = velocity_trigger(t_sim, diff_v, v_opt);
        
        subplot(n_vehicles, 1, v);
        hold on; grid on;
        
        % Plot delle velocità
        h1 = plot(t_sim, v_sim, 'b-', 'LineWidth', 1.5);
        h2 = plot(t_sim, v_opt, 'g--', 'LineWidth', 1.5);
        h3 = plot(t_sim, diff_v, 'r-.', 'LineWidth', 1.5);
        
        % Plot dei trigger di velocità
        h4 = plot(t_sim, trigger_state * (max(abs(diff_v))*0.8), 'm-', 'LineWidth', 2);
        
        % Plot dei punti di split per semaforo rosso e trigger
        h5 = [];
        h6 = [];
        
        y_lim = get(gca, 'YLim');
        max_y = y_lim(2);
        
        % Aggiungiamo indicatori verticali per gli split
        for idx = 1:length(split_times)
            if split_vehicles(idx) == v  % Questo veicolo è diventato leader
                if strcmp(split_reasons{idx}, 'semaforo')
                    % Linea verticale rossa per split da semaforo
                    h5 = plot([split_times(idx) split_times(idx)], [0 max_y], 'r-', 'LineWidth', 3);
                    text(split_times(idx), max_y*0.9, 'SPLIT (Semaforo)', 'Color', 'r', ...
                         'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'FontWeight', 'bold');
                else
                    % Linea verticale blu per split da trigger
                    h6 = plot([split_times(idx) split_times(idx)], [0 max_y], 'c-', 'LineWidth', 3);
                    text(split_times(idx), max_y*0.9, 'SPLIT (Trigger)', 'Color', 'c', ...
                         'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'FontWeight', 'bold');
                end
            end
        end
        
        % Legenda
        legend_handles = [h1, h2, h3, h4];
        legend_names = {'Velocità Simulata', 'Velocità Ottimale', 'Delta Velocità', 'Trigger Attivo'};
        
        if ~isempty(h5)
            legend_handles(end+1) = h5;
            legend_names{end+1} = 'Split per Semaforo Rosso';
        end
        if ~isempty(h6)
            legend_handles(end+1) = h6;
            legend_names{end+1} = 'Split per Trigger Velocità';
        end
        
        legend(legend_handles, legend_names, 'Location', 'best');
        
        xlabel('Tempo [s]');
        ylabel('Velocità [m/s]');
        title(['Veicolo ' num2str(v) ': Trigger e Split Detection']);
    end
end
function check_velocity_triggers(t_abs, x_sim, traffic_lights)
    persistent trigger_detected
    persistent last_platoon_time
    
    if isempty(trigger_detected), trigger_detected = false; end
    if isempty(last_platoon_time), last_platoon_time = 0; end
    
    
    global SIM_RUNS N_PLATOON
    current_run = length(SIM_RUNS);
    n_vehicles = size(x_sim, 2) / 2;
    
    % Tempo di inibizione del trigger dopo l'inizio di un nuovo plotone
    transitorio_inibizione = 10; % secondi di inibizione
    
    % Calcola il tempo relativo dall'inizio del plotone attuale
    current_offset = SIM_RUNS{current_run}.offset;
    tempo_relativo = t_abs(1) - current_offset;
    
    % Se siamo ancora nel transitorio, non controllare i trigger
    if tempo_relativo < transitorio_inibizione
        fprintf('[INFO] Transitorio attivo: trigger inibiti per altri %.1f secondi\n', ...
                transitorio_inibizione - tempo_relativo);
        return;
    end
    
    % Ottieni i dati ottimali dal run corrente
    runData = SIM_RUNS{current_run};
    t_sim = t_abs;
    opt_t = runData.opt_t;
    opt_d = runData.opt_d;
    
    % Per ogni veicolo nel plotone corrente
    for v = 1:n_vehicles
        % Salta i veicoli che non fanno parte di questo plotone
        if ~is_in_current_platoon(v, current_run)
            continue;
        end
        
        % Se è già il leader, salta
        if v == runData.leader
            continue;
        end
        
        % 1. VERIFICA SEMAFORI ROSSI ANTICIPATA
        % -----------------------------------
        % Verifica se il veicolo sta per passare con il rosso
        pos_v = x_sim(:, v);
        v_vel = x_sim(:, n_vehicles + v);
        
        % Previsione di posizione per i prossimi secondi
        look_ahead_time = 5; % secondi di anticipazione
        anticipation_steps = min(5, length(t_abs));
        
        % Per ogni semaforo, controlla se il veicolo lo attraverserà col rosso
        for L = 1:length(traffic_lights)
            light_d = traffic_lights(L).distance;
            
            % Calcola l'indice corrente del veicolo
            current_idx = length(pos_v);
            
            % Verifica se il veicolo è vicino ma non ha ancora superato il semaforo
            if pos_v(current_idx) < light_d && pos_v(current_idx) + v_vel(current_idx)*look_ahead_time >= light_d
                % Stima quando attraverserà il semaforo
                time_to_light = (light_d - pos_v(current_idx)) / max(v_vel(current_idx), 0.1);
                cross_time = t_abs(current_idx) + time_to_light;
                
                % Verifica se il semaforo sarà rosso quando il veicolo lo attraversa
                if ~is_green(traffic_lights(L), cross_time)
                    fprintf('\n[TRIGGER] Veicolo %d sta per passare col rosso a incrocio %d (t=%.2f s)\n', ...
                            v, L, t_abs(current_idx));
                    
                    if ~trigger_detected
                        trigger_detected = true;
                        fprintf('>> Veicolo %d diventa leader di un nuovo plotone per evitare semaforo rosso!\n', v);
                        
                        % Aggiorna i veicoli nel plotone corrente
                        SIM_RUNS{current_run}.splittedVehicles = v:n_vehicles;
                        
                        % Rilancia la simulazione con il nuovo leader
                        rerun_optimizer_for_trigger(v, t_abs(current_idx));
                        trigger_detected = false;
                        last_platoon_time = t_abs(current_idx);
                        return;
                    end
                end
            end
        end
        
        % 2. VERIFICA VELOCITÀ
        % -------------------
        % Calcola le differenze di velocità
        v_sim = x_sim(:, n_vehicles + v);
        pos_opt = interp1(opt_t, opt_d, t_sim, 'linear', 'extrap');
        v_opt = gradient(pos_opt, t_sim);
        offset_value = v_opt(end) - v_sim(end);
        diff_v = (v_opt - v_sim) - offset_value;
        
        % Calcola lo stato del trigger
        trigger_state = velocity_trigger(t_sim, diff_v, v_opt);
        
        % Se c'è almeno un trigger attivo e non è già stato rilevato
       if any(trigger_state) && ~trigger_detected
        trigger_idx = find(trigger_state, 1);
        trigger_time = t_sim(trigger_idx);
        
        % Ottieni la posizione attuale del veicolo
        current_position = x_sim(end, v);
            
        fprintf('\n[TRIGGER] Veicolo %d ha attivato un trigger di velocità a t=%.2f s, pos=%.2f m\n',...
                v, trigger_time, current_position);
        
        trigger_detected = true;
        fprintf('>> Veicolo %d si ferma e diventa leader di un nuovo plotone!\n', v);
        
        % Aggiorna i veicoli nel plotone corrente
        SIM_RUNS{current_run}.splittedVehicles = v:n_vehicles;
        
        % Rilancia la simulazione con il nuovo leader, fermandolo alla posizione attuale
        rerun_optimizer_for_trigger(v, trigger_time, current_position);
        trigger_detected = false;
        last_platoon_time = trigger_time;
        return;
    end
    end
end

function rerun_optimizer_for_trigger(trigger_vehicle, trigger_time, stop_position)
    clear system_dynamics_new_platoon
    clear get_current_v_target_indexed
    
    global SIM_RUNS
    global N_PLATOON
    
    % Mostra informazioni sulla creazione del nuovo plotone
    current_platoon = N_PLATOON + 1;
    disp(['[INFO] TRIGGER ATTIVO: Creazione NUOVO PLOTONE ' num2str(current_platoon) ' con LEADER=' ...
          num2str(trigger_vehicle) ', riparto da tempo assoluto=' ...
          num2str(trigger_time) ', posizione=' num2str(stop_position)]);
    
    % Incrementa il contatore di plotoni
    start_offset = trigger_time;  
    N_PLATOON = N_PLATOON + 1;
    
    % Esegui la simulazione con il nuovo leader, fermandolo alla posizione specificata
    run_optimizer_and_plot(trigger_vehicle, start_offset, stop_position);
    
    % Pulisci lo stato
    clear check_red_light_violations
    clear check_velocity_triggers
end

function t_out = next_green(light, t_in)
    % Allinea il tempo t_in al successivo periodo di verde
    cycle = light.cycle_time;
    offset = light.offset;
    green_s = light.green_start; 
    green_e = light.green_end;

    % Esempio semplificato
    % Calcolo la fase del ciclo
    dt = t_in - offset;
    k = floor(dt / cycle);
    phase = dt - k*cycle;
    if green_s <= green_e
        if phase < green_s
            t_out = offset + k*cycle + green_s;
        elseif phase > green_e
            t_out = offset + (k+1)*cycle + green_s;
        else
            t_out = t_in; 
        end
    else
        % Verde attraversa la fine del ciclo
        if phase > green_e && phase < green_s
            t_out = offset + (k+1)*cycle + green_s;
        else
            t_out = t_in;
        end
    end
end

function t_out = prev_green(light, t_in)
    % Allinea t_in al termine del periodo di verde
    cycle = light.cycle_time;
    offset = light.offset;
    green_s = light.green_start;
    green_e = light.green_end;

    dt = t_in - offset;
    k = floor(dt / cycle);
    phase = dt - k*cycle;

    if green_s <= green_e
        if phase > green_e
            t_out = offset + k*cycle + green_e;
        elseif phase < green_s
            t_out = offset + (k-1)*cycle + green_e;
        else
            t_out = t_in;
        end
    else
        % Verde attraversa la fine del ciclo
        if phase < green_s && phase > green_e
            t_out = offset + k*cycle + green_e;
        else
            t_out = t_in;
        end
    end
end

function flag = is_green(light, t_in)
    % Ritorna true se al tempo t_in il semaforo light è in verde
    cycle = light.cycle_time;
    offset = light.offset;
    green_s = light.green_start;
    green_e = light.green_end;

    dt = t_in - offset;
    if dt < 0, flag = false; return; end
    k = floor(dt / cycle);
    phase = dt - k*cycle;

    if green_s <= green_e
        flag = (phase >= green_s) && (phase <= green_e);
    else
        % Verde a cavallo
        if phase >= green_s || phase <= green_e
            flag = true;
        else
            flag = false;
        end
    end
end

function check_and_trigger(t_abs, x_sim, traffic_lights, leader_vehicle)
    % Controlla le condizioni di trigger (semaforo rosso imminente, scostamento
    % velocità, ecc.). Se rileva un trigger, crea un nuovo plotone.

    global SIM_RUNS

    % 1) Rilevamento semaforo rosso imminente
    [veh_red, time_red, pos_red] = detect_red_light(t_abs, x_sim, traffic_lights);
    if ~isempty(veh_red)
        fprintf('[TRIGGER] Veicolo %d sta per passare col rosso.\n', veh_red);
        splitted_vehicles = veh_red : size(x_sim,2)/2;  % Semplice ipotesi: tutti i veicoli successivi
        SIM_RUNS{end}.splittedVehicles = splitted_vehicles;
        
        % Creazione del nuovo plotone con un tempo di ripartenza ipotetico
        time_stop = time_red + 20;  
        run_new_platoon(veh_red, time_stop, pos_red, traffic_lights);
        return; 
    end

    % 2) Rilevamento scostamento velocità
    [veh_vel_diff, trigger_time, trigger_pos] = detect_speed_trigger(t_abs, x_sim, traffic_lights);
    if ~isempty(veh_vel_diff)
        fprintf('[TRIGGER] Veicolo %d scostamento velocità eccessivo.\n', veh_vel_diff);
        splitted_vehicles = veh_vel_diff : size(x_sim,2)/2;
        SIM_RUNS{end}.splittedVehicles = splitted_vehicles;
        
        run_new_platoon(veh_vel_diff, trigger_time, trigger_pos, traffic_lights);
        return;
    end
end



function run_new_platoon(leader_id, start_time, start_position, traffic_lights)
    global N_PLATOON SIM_RUNS
    N_PLATOON = N_PLATOON + 1;
    fprintf('[INFO] Creazione NUOVO PLOTONE %d con Leader=%d\n', N_PLATOON, leader_id);
    
    % Ora chiamiamo l'ottimizzazione passando anche start_position
    run_optimizer_from_traffic_light(leader_id, start_time, start_position, traffic_lights);
end


function [veh_red, time_red, pos_red] = detect_red_light(t_abs, x_sim, lights)
    % Esempio minimale: restituisce il primo veicolo che troverà rosso
    veh_red = [];
    time_red = [];
    pos_red = [];

    % Inserisci qui la logica personalizzata per il tuo scenario.
    % Per esempio, scorri i veicoli e controlla se uno di essi
    % è prossimo ad un semaforo rosso.
    
    % [Solo esempio fittizio]
    for i = 1 : size(x_sim,2)/2
        vPos = x_sim(end, i);    % posizione all'ultimo step
        vVel = x_sim(end, i + size(x_sim,2)/2); % velocità
        % Se prossimo semaforo risulta rosso in base ai cicli
        % e se (vPos < distanceSem) e tempo stima passaggio t_abs ~ ...
        % Allora:
        if vPos > 500 && vVel > 0  % Esempio di "condizione"
            veh_red = i;
            time_red = t_abs;  % stima tempo in cui incrocia il semaforo
            pos_red = vPos;
            return;
        end
    end
end


function [veh_vel_diff, trigger_time, trigger_pos] = detect_speed_trigger(t_abs, x_sim, lights)
    % Esempio minimale: restituisce il primo veicolo con scostamento eccessivo dalla velocità target
    veh_vel_diff = [];
    trigger_time = [];
    trigger_pos  = [];

    % [Solo esempio fittizio]
    for i = 1 : size(x_sim,2)/2
        vVel = x_sim(end, i + size(x_sim,2)/2);
        % Se la velocità è troppo deviata da un eventuale target...
        if vVel > 35  % es. soglia
            veh_vel_diff = i;
            trigger_time = t_abs;
            trigger_pos  = x_sim(end, i);
            return;
        end
    end
end