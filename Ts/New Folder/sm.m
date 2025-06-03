
%% Main script
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
%% Local functions
%% =========================================================================

function run_optimizer_and_plot(leader_vehicle, time_offset, start_position)
    if nargin < 3
         start_position = 0;
    end
    global SIM_RUNS N_PLATOON;
    SIM_RUNS = {}; % Resetta SIM_RUNS all'inizio di una nuova ottimizzazione principale
    N_PLATOON = 1; % Resetta il contatore dei plotoni
    
    reset_persistent_variables();
    
    final_time     = 200;         
    final_distance = 1800;    
    T_cycle_semafori = 30; % Rinominato per chiarezza rispetto a 'T' usato altrove        
    tf             = final_time;
    v_min          = 3;   
    v_max          = 30;  
    b1_cost        = 0.1;  % Costo
    b2_cost        = 15;   % Costo
    % b3, b4 erano per delta_func, non usati direttamente nel costo degli archi
    
    delta_func = @(t) 0; % Semplificato, non ci sono disturbi attivi
    
    fprintf('\n[INFO] run_optimizer_and_plot(Leader=%d, offset=%.2f, start_pos=%.2f)\n', ...
        leader_vehicle, time_offset, start_position);
    
    traffic_lights = [
         create_traffic_light(300,   0, 10, T_cycle_semafori)
         create_traffic_light(600,  10, 20, T_cycle_semafori)
         create_traffic_light(900,  20, 30, T_cycle_semafori) % Verde fino alla fine del ciclo
         create_traffic_light(1200, 0, 10, T_cycle_semafori)
         create_traffic_light(1500, 10, 20, T_cycle_semafori)
    ];
     
    [t_min_pruning, t_max_pruning] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max); % Rinominato t_min, t_max per evitare shadowing
    d_semafori = [traffic_lights.distance]; 
    nIntersections = length(traffic_lights);
    
    Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
    nodeId = 1;
    Nodes(nodeId) = struct('id', nodeId, 't', time_offset, 'd', start_position, 'int', 0); 
    nodeId = nodeId + 1;
    
    for i = 1:nIntersections
        light = traffic_lights(i);
        % Usa t_min_pruning, t_max_pruning specifici per questo semaforo
        current_t_min_for_light_i = t_min_pruning(i);
        current_t_max_for_light_i = t_max_pruning(i);

        for k = 0:ceil((tf + time_offset) / light.cycle_time) % Estendi i cicli per coprire tutto il tempo
            % L'offset del semaforo definisce l'inizio del suo pattern (primo verde)
            % green_start e green_end sono relativi all'inizio del ciclo del semaforo,
            % ma il campo 'offset' nella struct light è l'inizio assoluto del primo verde.
            % La creazione dei nodi usa light.offset come inizio del primo verde,
            % e light.green_start/end come relativi a light.offset per il primo ciclo,
            % e poi ripetuti con cycle_time.
            
            % Convenzione per la creazione nodi (basata su codice originale):
            % light.offset è l'inizio del primo verde (assoluto).
            % light.green_start è l'inizio del primo verde (assoluto, == light.offset).
            % light.green_end è la fine del primo verde (assoluto).
            % La durata del verde è light.green_end - light.green_start.
            
            first_green_start_time_abs = light.green_start; % Inizio del primo verde in assoluto
            green_duration_val = light.green_end - light.green_start;

            abs_green_start_curr_cycle = first_green_start_time_abs + k * light.cycle_time;
            abs_green_end_curr_cycle   = abs_green_start_curr_cycle + green_duration_val;

            if abs_green_start_curr_cycle <= tf + time_offset % Se l'inizio del verde è nel range di simulazione
                overlap_start = max(abs_green_start_curr_cycle, current_t_min_for_light_i);
                overlap_end   = min(abs_green_end_curr_cycle,   current_t_max_for_light_i);

                if overlap_start < overlap_end - 1e-3 % Tolleranza per evitare nodi troppo vicini
                    % Scegli un punto rappresentativo, es. il centro.
                    % Se il pruning ha già dato un punto (t_min=t_max), usa quello.
                    if abs(overlap_end - overlap_start) < 1e-2 % Finestra molto stretta
                        middle_time = overlap_start;
                    else
                        middle_time = (overlap_start + overlap_end) / 2;
                    end
                    % middle_time = ceil((overlap_start + overlap_end) / 2); % Originale
                    
                    % Evita di aggiungere nodi duplicati (stesso tempo e distanza)
                    is_duplicate_node = false;
                    for nn_idx = 1:length(Nodes)
                        if Nodes(nn_idx).d == d_semafori(i) && abs(Nodes(nn_idx).t - middle_time) < 0.1 ...
                           && Nodes(nn_idx).int == i
                            is_duplicate_node = true;
                            break;
                        end
                    end
                    if ~is_duplicate_node
                        Nodes(nodeId) = struct('id', nodeId, 't', middle_time, 'd', d_semafori(i), 'int', i);
                        nodeId = nodeId + 1;
                    end
                end
            end
            % La gestione di green_start > green_end non è necessaria se interpretiamo
            % green_start/end come i tempi assoluti del *primo* periodo verde.
        end
    end
    
    % Assicura che ci sia almeno un nodo per ogni intersezione se possibile,
    % prendendo il punto centrale della finestra verde più vicina se il loop sopra non ne ha trovati.
    % Questo è complesso e potrebbe essere meglio gestito da velocity_pruning e una logica di creazione nodi più robusta.
    % Per ora, si assume che il loop sopra trovi i nodi necessari.
    
    Nodes(nodeId) = struct('id', nodeId, 't', time_offset + tf, 'd', final_distance, 'int', nIntersections + 1);
    nNodes = length(Nodes); % Aggiorna nNodes al numero effettivo di nodi creati

    % Rimuovi nodi duplicati (stesso tempo, distanza, intersezione) che potrebbero essersi formati
    if nNodes > 1
        [~, unique_indices] = unique(arrayfun(@(s) sprintf('%.2f-%.2f-%d', s.t, s.d, s.int), Nodes, 'UniformOutput', false));
        Nodes = Nodes(unique_indices);
        % Ri-assegna ID sequenziali dopo la rimozione dei duplicati
        for nn_idx = 1:length(Nodes)
            Nodes(nn_idx).id = nn_idx;
        end
        nNodes = length(Nodes);
        nodeId = nNodes +1; % Prossimo ID disponibile se servisse
         % Riassegna l'ID del nodo finale
        if Nodes(end).int == nIntersections + 1 % Se l'ultimo è il nodo finale
            Nodes(end).id = nNodes;
        else % Aggiungi di nuovo il nodo finale se è stato rimosso per errore
            Nodes(nNodes+1) = struct('id', nNodes+1, 't', time_offset + tf, 'd', final_distance, 'int', nIntersections + 1);
            nNodes = nNodes+1;
        end
    end


    Edges = struct('from', {}, 'to', {}, 'w', {});
    edgeCount = 1;
    for i = 1:nNodes
        node_A = Nodes(i); % Usa l'ID del nodo come indice per gli archi
        lvlA = node_A.int;
        for j = 1:nNodes
            node_B = Nodes(j);
            lvlB = node_B.int;
            if lvlB == lvlA + 1 % Archi solo tra livelli consecutivi di intersezione
                if node_B.t > node_A.t + 1e-3 && node_B.d > node_A.d + 1e-3 % Deve avanzare in tempo e spazio
                    % Se il nodo di destinazione è un semaforo, verifica se è verde
                    if node_B.int > 0 && node_B.int <= nIntersections 
                        if ~is_green(traffic_lights(node_B.int), node_B.t)
                            continue; % Salta arco se il semaforo è rosso al tempo di arrivo
                        end
                    end
                    
                    delta_t_edge = node_B.t - node_A.t;
                    delta_d_edge = node_B.d - node_A.d;
                    v_link  = delta_d_edge / delta_t_edge;
                    
                    if v_link >= v_min && v_link <= v_max
                        % Costo dell'arco (basato su b1_cost, b2_cost)
                        E_link = delta_t_edge * (b1_cost * v_link + b2_cost * v_link^2); 
                        Edges(edgeCount) = struct('from', node_A.id, 'to', node_B.id, 'w', E_link);
                        edgeCount = edgeCount + 1;
                    end
                end
            end
        end
    end
    
    [path_node_ids, cost] = dijkstra(Nodes, Edges, Nodes(1).id, Nodes(nNodes).id); % Usa ID dei nodi start/end
    
    if isinf(cost) || isempty(path_node_ids)
        error('[ERROR] No valid path found by optimization. Cost is Inf or path is empty.');
    end
    
    % Mappa gli ID del percorso agli indici dei nodi per recuperare i dati
    opt_nodes_on_path = [];
    for p_id = path_node_ids
        found_node = Nodes([Nodes.id] == p_id);
        opt_nodes_on_path = [opt_nodes_on_path, found_node]; %#ok<AGROW>
    end

    opt_t = [opt_nodes_on_path.t];
    opt_d = [opt_nodes_on_path.d];
    
    fprintf('>>> Leader=%d, Optimal cost=%.3f\n', leader_vehicle, cost);
    
    speeds = zeros(1, length(opt_t) - 1);
    for k = 1:(length(opt_t) - 1)
        d_seg = opt_d(k + 1) - opt_d(k);
        t_seg = opt_t(k + 1) - opt_t(k);
        if t_seg > 1e-6
            speeds(k) = d_seg / t_seg;
        else
            speeds(k) = v_min; % Fallback se intervallo temporale troppo piccolo
        end
    end
    if isempty(speeds) && length(opt_t)==1 % Percorso con un solo nodo (start=end)
        speeds = 0;
    elseif isempty(speeds) && length(opt_t)>1
        error('Optimal path has multiple nodes but no speed segments calculated.');
    end
    
    n_vehicles = 5;
    m_vehicles = 1000 * ones(1, n_vehicles); % Massa kg
    v_targets_sim = speeds;  
    
    % Parametri SMC Leader
    K_p_smc_l = 700;  % Ridotto Kp
    K_r_smc_l = 100;  % Ridotto Kr
    phi_smc_l = 0.8;  % Aumentato phi per smussare
    
    % Parametri SMC Follower
    K_p_smc_f = 200;  % Ridotto Kp
    K_r_smc_f = 80;   % Ridotto Kr
    phi_smc_f = 0.8;  % Aumentato phi
    lambda_smc_f = 0.8; % Ridotto lambda per meno enfasi su velocità relativa
    
    t_CTH     = 1.5;  % s
    d_init    = 5;    % m, aumentata un po'
    
    x0 = zeros(2 * n_vehicles, 1);
    for i = 1:n_vehicles
        if i == leader_vehicle
            x0(i) = start_position; % Posizione
            % x0(n_vehicles + i) = v_targets_sim(1) se parte in movimento, o 0 se da fermo
            % Assumiamo partenza da velocità nulla o gestita dal target iniziale
        else
            % Calcola offset rispetto al leader_vehicle
            offset_factor = 0;
            num_between = 0;
            if i > leader_vehicle % Follower dietro al leader
                num_between = i - leader_vehicle;
            elseif i < leader_vehicle % Follower "davanti" al leader (improbabile in questa config, ma gestito)
                num_between = leader_vehicle - i; % Questo darebbe offset positivo, serve negativo
                                                  % In un plotone standard, i follower hanno ID > leader
            end
            x0(i) = start_position - d_init * num_between;
        end
        % Velocità iniziale (x0(n_vehicles+i)) è zero per default
    end
    
    t_span = [time_offset, time_offset + tf];
    
    smc_params_leader = struct('Kp', K_p_smc_l, 'Kr', K_r_smc_l, 'phi', phi_smc_l);
    smc_params_follower = struct('Kp', K_p_smc_f, 'Kr', K_r_smc_f, 'phi', phi_smc_f, 'lambda', lambda_smc_f);

    [t_sim, x_sim] = ode45(@(t,x_state) system_dynamics_new_platoon( ... % Rinomina x a x_state per chiarezza
        t, x_state, n_vehicles, m_vehicles, delta_func, traffic_lights, v_targets_sim, t_CTH, ...
        smc_params_leader, smc_params_follower, leader_vehicle, time_offset), t_span, x0);
    
    T_abs = t_sim;  
    SIM_RUNS{end+1} = struct( ...
        'leader', leader_vehicle, ...
        't', T_abs, ...
        'x', x_sim, ...
        'offset', time_offset, ...
        'traffic_lights', traffic_lights, ...
        'splittedVehicles', [], ... 
        'v_targets', v_targets_sim, ...
        'opt_t', opt_t, ...
        'opt_d', opt_d, ...
        'oldPlatoonTrajectory', [] ... 
    );
    
    check_red_light_violations(T_abs, x_sim, traffic_lights, T_cycle_semafori);
    check_velocity_triggers(T_abs, x_sim, traffic_lights); % traffic_lights passato per coerenza, anche se non usato direttamente
end

% Modificata per SMC
function dx = system_dynamics_new_platoon(t, x, n_vehicles, m, delta_func, ...
    traffic_lights, v_targets, t_CTH, smc_params_leader, smc_params_follower, ...
    leader_vehicle, time_offset_sim) % Rinominato time_offset per non confondere con var globali

    dx = zeros(2*n_vehicles, 1);
    abs_t = t; % Il 't' da ode45 è già il tempo assoluto se t_span inizia da time_offset_sim

    for i = 1:n_vehicles
        pos_curr = x(i);
        vel_curr = x(n_vehicles + i);
        dx(i) = vel_curr;

        if i == leader_vehicle
            v_target_leader = get_current_v_target_indexed(pos_curr, traffic_lights, v_targets);
            s_leader = v_target_leader - vel_curr;
            
            U_leader = smc_params_leader.Kp * s_leader + ...
                       smc_params_leader.Kr * tanh(s_leader / smc_params_leader.phi);
            
            dx(n_vehicles + i) = (U_leader + delta_func(abs_t)) / m(i);
        else % Follower vehicles
            % Trova il veicolo che precede questo follower i
            % In questa configurazione, il predecessore è sempre i-1
            % Questo fallisce se il leader non è il veicolo 1 e i=1 (follower senza i-1)
            % Ma la logica di platoon implica che un follower ha un predecessore
            
            % Se leader_vehicle è > 1, allora il veicolo 1 è un follower.
            % In tal caso, il suo "predecessore" deve essere definito diversamente
            % (es. il veicolo N se è un anello, o nessun controllo se è il primo della fila).
            % Per ora, assumiamo una struttura di plotone lineare semplice dove i-1 è il predecessore.
            
            idx_predecessor = -1;
            if i > leader_vehicle % Follower standard dietro al leader
                if i == leader_vehicle + 1 % Segue direttamente il leader
                    idx_predecessor = leader_vehicle;
                else % Segue un altro follower
                    idx_predecessor = i-1; 
                end
            elseif i < leader_vehicle % Scenario in cui il follower è "davanti" al leader (es. leader al centro)
                % Questa logica non è coperta bene dalla semplice i-1.
                % Per ora, ci concentriamo sullo scenario leader = 1, o leader in testa.
                % Se il leader è il veicolo `L`, un follower `F < L` non ha un `i-1` come predecessore.
                % Lasciamo la logica originale che implica i-1 è il predecessore
                % e questo sarà problematico se leader_vehicle != 1 e i=1.
                if i > 1 % Ha un i-1 fisico
                    idx_predecessor = i-1;
                else % Veicolo 1 è follower, ma non ha i-1. Cosa fare?
                      % Questo scenario deve essere chiarito.
                      % Per ora, se i=1 e non è leader, non applichiamo controllo CTH.
                    dx(n_vehicles + i) = 0; % Nessun controllo CTH
                    continue; % Passa al prossimo veicolo
                end
            end
             % Se per qualche motivo i=1 e non è leader, idx_predecessor rimane -1
            if idx_predecessor == -1 && i > 1 % Fallback per i>1 se la logica sopra fallisce
                idx_predecessor = i-1;
            end


            if idx_predecessor > 0 % Se un predecessore valido è stato identificato
                pos_prec = x(idx_predecessor);
                vel_prec = x(n_vehicles + idx_predecessor);
                
                dist_actual = pos_prec - pos_curr;
                d_min_val = 1.0; 
                dist_desired = d_min_val + t_CTH * vel_curr; % CTH basato sulla propria velocità
                
                error_dist = dist_actual - dist_desired;
                error_vel_rel = vel_prec - vel_curr;
                
                s_follower = error_dist + smc_params_follower.lambda * error_vel_rel;
                
                U_follower = smc_params_follower.Kp * s_follower + ...
                             smc_params_follower.Kr * tanh(s_follower / smc_params_follower.phi);
                
                dx(n_vehicles + i) = U_follower / m(i);
            elseif i==1 && i~=leader_vehicle % Veicolo 1 è follower e non ha predecessore definito
                % In questo caso, potrebbe seguire una velocità target fissa o non essere controllato
                % Per semplicità, se è il primo veicolo e non è leader, non applichiamo controllo CTH.
                % Potrebbe seguire una v_target fissa se necessario.
                dx(n_vehicles + i) = 0; % Nessun controllo specifico
            end
        end
    end
end


function vt = get_current_v_target_indexed(x_leader, traffic_lights, v_targets_segments) % Rinominato v_targets
    if isempty(v_targets_segments)
        % fprintf('[WARNING] v_targets_segments is empty in get_current_v_target_indexed, using default speed 10 m/s\n');
        vt = 10; 
        return;
    end
    
    if isempty(traffic_lights) || x_leader >= traffic_lights(end).distance || x_leader >= 1800 % Final distance
        % Leader ha superato tutti i semafori o la distanza finale
        vt = v_targets_segments(end);
        return;
    end
    
    % Trova l'indice del segmento corrente.
    % v_targets_segments(k) è la velocità per il segmento k, che termina al semaforo k (o nodo k).
    % Se il leader è tra il semaforo k-1 e il semaforo k, la velocità target è v_targets_segments(k).
    % (Assumendo che il primo segmento, prima del primo semaforo, sia v_targets_segments(1)).
    
    idx_next_light = find(x_leader < [traffic_lights.distance], 1, 'first');
    
    if isempty(idx_next_light)
        % Superati tutti i semafori, ma non ancora final_distance. Usa l'ultima velocità.
        vt = v_targets_segments(end);
    else
        % idx_next_light è l'indice del prossimo semaforo.
        % Il segmento di velocità corrente è quello che *porta* a questo semaforo.
        % Quindi la velocità target è v_targets_segments(idx_next_light).
        target_idx = min(idx_next_light, length(v_targets_segments));
        vt = v_targets_segments(target_idx);
    end
end

function check_red_light_violations(t_abs, x_sim, traffic_lights_list, T_cycle_val) % Rinominato T
    persistent violations_detected_map; % Usa una mappa per gestire le violazioni per veicolo/semaforo
    if isempty(violations_detected_map), violations_detected_map = containers.Map('KeyType','char','ValueType','logical'); end

    n_vehicles = size(x_sim,2)/2;
    global SIM_RUNS
    current_run_idx = length(SIM_RUNS); % Rinominato current_run
        
    look_ahead_dist = 30;  % m
    
    for v_id = 1:n_vehicles % Rinominato v
        if ~is_in_current_platoon(v_id, current_run_idx)
            continue;
        end
        
        pos_vehicle_v = x_sim(:, v_id);
        vel_vehicle_v = x_sim(:, n_vehicles + v_id);
        
        for tl_idx = 1:length(traffic_lights_list) % Rinominato L
            light_obj = traffic_lights_list(tl_idx); % Rinominato light
            light_pos_d = light_obj.distance; % Rinominato light_d
            
            violation_key = sprintf('v%d_tl%d', v_id, tl_idx); % Chiave univoca per veicolo-semaforo

            % Trova l'ultimo istante in cui il veicolo è in avvicinamento e non ha ancora superato il semaforo
            % e non ha già una violazione registrata per questo semaforo
            approaching_indices = find(pos_vehicle_v < light_pos_d & pos_vehicle_v >= light_pos_d - look_ahead_dist);
            if isempty(approaching_indices), continue; end
            
            last_approaching_sim_idx = approaching_indices(end);
            
            approach_time_abs = t_abs(last_approaching_sim_idx);
            current_pos_veh = pos_vehicle_v(last_approaching_sim_idx);
            current_vel_veh = vel_vehicle_v(last_approaching_sim_idx);
            
            if current_vel_veh < 0.1 % Se il veicolo è quasi fermo o va indietro, non proiettare
                continue;
            end

            time_to_reach_light = (light_pos_d - current_pos_veh) / current_vel_veh;
            estimated_crossing_time_abs = approach_time_abs + time_to_reach_light;
            
            if ~is_green(light_obj, estimated_crossing_time_abs)
                if isKey(violations_detected_map, violation_key) && violations_detected_map(violation_key)
                    continue; % Violazione già gestita per questa coppia veicolo-semaforo
                end
                
                % Verifica se uno split per questo veicolo è già in corso o pianificato
                already_splitting = false;
                if isfield(SIM_RUNS{current_run_idx}, 'splittedVehicles') && ...
                   ~isempty(SIM_RUNS{current_run_idx}.splittedVehicles) && ...
                   ismember(v_id, SIM_RUNS{current_run_idx}.splittedVehicles)
                   already_splitting = true;
                end
                if already_splitting, continue; end


                fprintf('\n[WARNING] Vehicle %d is about to pass RED light at intersection %d (pos %.1fm) at t=%.2f s.\n', ...
                        v_id, tl_idx, light_pos_d, estimated_crossing_time_abs);
                fprintf('          Current: t=%.2f s, pos=%.2f m, vel=%.2f m/s.\n', approach_time_abs, current_pos_veh, current_vel_veh);
                
                violations_detected_map(violation_key) = true; % Marca come gestita
                
                fprintf('>> Vehicle %d initiates split, new platoon leader.\n', v_id);

                % Determina i veicoli per il nuovo plotone (da v_id fino alla fine del plotone corrente)
                current_platoon_vehicles = get_platoon_vehicles(current_run_idx);
                idx_v_in_platoon = find(current_platoon_vehicles == v_id);
                if isempty(idx_v_in_platoon) % Non dovrebbe succedere
                    vehicles_for_new_platoon = v_id:n_vehicles; % Fallback
                else
                    vehicles_for_new_platoon = current_platoon_vehicles(idx_v_in_platoon:end);
                end

                SIM_RUNS{current_run_idx}.splittedVehicles = vehicles_for_new_platoon;

                next_green_start_time = next_green(light_obj, estimated_crossing_time_abs);
                
                % Posizione di arresto leggermente prima del semaforo
                stop_pos_before_light = light_pos_d - 5; 
                
                % Usa la funzione wrapper che gestisce N_PLATOON
                rerun_optimizer_from_traffic_light_EVENT(v_id, next_green_start_time, stop_pos_before_light, light_obj); 
                return; % Gestisci una violazione alla volta per run di check
            end
        end
    end
end

% Modificata per SMC
function rerun_optimizer_from_traffic_light(leader_vehicle, start_time, start_position, ~) % traffic_light_obj_at_stop non usato qui
    global SIM_RUNS % N_PLATOON è gestito dalla wrapper

    % Resetta le variabili persistenti delle funzioni chiave
    reset_persistent_variables(); % Chiamato anche all'inizio di run_optimizer_and_plot

    final_time_sim = 200; 
    final_distance_sim = 1800;
    % T_cycle non necessario qui
    tf_this_opt = final_time_sim; % Tempo relativo per questa ottimizzazione (può essere accorciato)
    v_min_opt  = 3;
    v_max_opt  = 30;
    b1_opt_cost = 0.1;
    b2_opt_cost = 10;  
    
    delta_func_rerun = @(t) 0; 
    
    fprintf('\n[INFO] Optimizing new platoon: Leader=%d, StartTime=%.2f, StartPos=%.2f\n', ...
        leader_vehicle, start_time, start_position);

    traffic_lights_cfg = SIM_RUNS{1}.traffic_lights; 
    
    old_platoon_traj = get_old_platoon_trajectory();
    
    remaining_lights_list = [];
    for i = 1:length(traffic_lights_cfg)
        if traffic_lights_cfg(i).distance > start_position + 0.5 % Piccolo margine
            remaining_lights_list = [remaining_lights_list; traffic_lights_cfg(i)];
        end
    end
    
    if isempty(remaining_lights_list)
        Nodes_rerun = struct('id', {}, 't', {}, 'd', {}, 'int', {});
        Nodes_rerun(1) = struct('id', 1, 't', start_time, 'd', start_position, 'int', 0);
        % Tempo di arrivo finale: o tf_this_opt o il tempo per coprire la distanza a v_min
        time_to_final_at_vmin = (final_distance_sim - start_position) / v_min_opt;
        if time_to_final_at_vmin < 0, time_to_final_at_vmin = tf_this_opt; end % Già oltre
        
        arrival_time_at_final = start_time + min(tf_this_opt, time_to_final_at_vmin);
        Nodes_rerun(2) = struct('id', 2, 't', arrival_time_at_final, 'd', final_distance_sim, 'int', 1);
        nNodes_rerun = 2;
        
        Edges_rerun = struct('from', {}, 'to', {}, 'w', {});
        if Nodes_rerun(2).t > Nodes_rerun(1).t && Nodes_rerun(2).d >= Nodes_rerun(1).d % Permetti d uguale se fermo
            delta_t_edge_rerun = Nodes_rerun(2).t - Nodes_rerun(1).t;
            delta_d_edge_rerun = Nodes_rerun(2).d - Nodes_rerun(1).d;
            if delta_t_edge_rerun < 1e-3 % Se il tempo è brevissimo, assumi velocità nulla o minima
                 v_link_rerun = 0;
            else
                v_link_rerun = delta_d_edge_rerun / delta_t_edge_rerun;
            end

            if (v_link_rerun >= v_min_opt && v_link_rerun <= v_max_opt) || (v_link_rerun < v_min_opt && delta_d_edge_rerun < 1e-2) % Permetti velocità basse se quasi fermo
                 allowEdge_rerun = check_non_overlapping(Nodes_rerun(1).t, Nodes_rerun(2).t, ...
                                                   Nodes_rerun(1).d, Nodes_rerun(2).d, ...
                                                   old_platoon_traj);
                if allowEdge_rerun
                    E_link_rerun = delta_t_edge_rerun * (b1_opt_cost*abs(v_link_rerun) + b2_opt_cost*v_link_rerun^2); % Usa abs(v) per b1
                    Edges_rerun(1) = struct('from', 1, 'to', 2, 'w', E_link_rerun);
                end
            end
        end
        if isempty(Edges_rerun)
            fprintf('[WARNING] No direct edge to final node in rerun. Path cost will be Inf.\n');
        end
    else
        d_rem_lights = [remaining_lights_list.distance];
        nIntersections_rem_lights = length(remaining_lights_list);
        
        [t_min_rem_pruning, t_max_rem_pruning] = velocity_pruning(remaining_lights_list, tf_this_opt, final_distance_sim - start_position, v_min_opt, v_max_opt);
        % Adatta i tempi di pruning all'offset start_time
        t_min_rem_pruning = t_min_rem_pruning + start_time;
        t_max_rem_pruning = t_max_rem_pruning + start_time;


        Nodes_rerun = struct('id', {}, 't', {}, 'd', {}, 'int', {});
        nodeId_rerun = 1;
        Nodes_rerun(nodeId_rerun) = struct('id', nodeId_rerun, 't', start_time, 'd', start_position, 'int', 0);
        nodeId_rerun = nodeId_rerun + 1;
        
        for i = 1:nIntersections_rem_lights
            light_i_rem = remaining_lights_list(i);
            current_t_min_rem_light = t_min_rem_pruning(i);
            current_t_max_rem_light = t_max_rem_pruning(i);
            
            max_time_for_nodes = start_time + tf_this_opt;
            
            % Logica di creazione nodi semafori (simile a run_optimizer_and_plot)
            first_green_start_abs_rem = light_i_rem.green_start;
            green_duration_rem = light_i_rem.green_end - light_i_rem.green_start;

            for k_cycle = 0:ceil(max_time_for_nodes / light_i_rem.cycle_time)
                abs_gs_curr_cycle_rem = first_green_start_abs_rem + k_cycle * light_i_rem.cycle_time;
                abs_ge_curr_cycle_rem = abs_gs_curr_cycle_rem + green_duration_rem;

                if abs_gs_curr_cycle_rem <= max_time_for_nodes
                    overlap_s = max(abs_gs_curr_cycle_rem, current_t_min_rem_light);
                    overlap_e = min(abs_ge_curr_cycle_rem, current_t_max_rem_light);

                    if overlap_s < overlap_e - 1e-3
                        mid_t_node = (overlap_s + overlap_e) / 2;
                        % Evita duplicati
                        is_dup = false;
                        for nn_idx_check = 1:length(Nodes_rerun)
                            if Nodes_rerun(nn_idx_check).d == d_rem_lights(i) && abs(Nodes_rerun(nn_idx_check).t - mid_t_node) < 0.1 && Nodes_rerun(nn_idx_check).int == i
                                is_dup = true; break;
                            end
                        end
                        if ~is_dup
                            Nodes_rerun(nodeId_rerun) = struct('id', nodeId_rerun, 't', mid_t_node, 'd', d_rem_lights(i), 'int', i); % int è l'indice in remaining_lights_list
                            nodeId_rerun = nodeId_rerun + 1;
                        end
                    end
                end
            end
        end

        Nodes_rerun(nodeId_rerun) = struct('id', nodeId_rerun, 't', start_time + tf_this_opt, 'd', final_distance_sim, 'int', nIntersections_rem_lights+1);
        nNodes_rerun = length(Nodes_rerun); % Usa length dopo aver aggiunto tutti
        % Rimuovi duplicati e riassegna ID
        if nNodes_rerun > 1
            [~, unique_indices_rerun] = unique(arrayfun(@(s) sprintf('%.2f-%.2f-%d', s.t, s.d, s.int), Nodes_rerun, 'UniformOutput', false));
            Nodes_rerun = Nodes_rerun(unique_indices_rerun);
            for nn_idx = 1:length(Nodes_rerun)
                Nodes_rerun(nn_idx).id = nn_idx;
            end
            nNodes_rerun = length(Nodes_rerun);
            if Nodes_rerun(end).int ~= nIntersections_rem_lights + 1
                 Nodes_rerun(nNodes_rerun+1) = struct('id', nNodes_rerun+1, 't', start_time + tf_this_opt, 'd', final_distance_sim, 'int', nIntersections_rem_lights + 1);
                 nNodes_rerun = nNodes_rerun+1;
            end
             Nodes_rerun(end).id = nNodes_rerun; % Assicura che l'ID dell'ultimo nodo sia corretto
        end

        
        Edges_rerun = struct('from', {}, 'to', {}, 'w', {});
        edgeCount_rerun = 1;
        for i_nodeA = 1:nNodes_rerun
            nodeA_rerun = Nodes_rerun(i_nodeA);
            lvlA_rerun = nodeA_rerun.int;
            for j_nodeB = 1:nNodes_rerun
                nodeB_rerun = Nodes_rerun(j_nodeB);
                lvlB_rerun = nodeB_rerun.int;
                if lvlB_rerun == lvlA_rerun + 1
                    if (nodeB_rerun.t > nodeA_rerun.t + 1e-3) && (nodeB_rerun.d >= nodeA_rerun.d) % Permetti d uguale
                        if lvlB_rerun > 0 && lvlB_rerun <= nIntersections_rem_lights
                            if ~is_green(remaining_lights_list(lvlB_rerun), nodeB_rerun.t)
                                continue;
                            end
                        end
                        delta_t_rerun_edge = nodeB_rerun.t - nodeA_rerun.t;
                        delta_d_rerun_edge = nodeB_rerun.d - nodeA_rerun.d;
                        if delta_t_rerun_edge < 1e-3, v_link_rerun_val = 0; else, v_link_rerun_val = delta_d_rerun_edge / delta_t_rerun_edge; end
                        
                        if (v_link_rerun_val >= v_min_opt && v_link_rerun_val <= v_max_opt) || (abs(v_link_rerun_val) < v_min_opt && delta_d_rerun_edge < 1e-2)
                            allowEdge_rerun_check = check_non_overlapping(nodeA_rerun.t, nodeB_rerun.t, ...
                                                              nodeA_rerun.d, nodeB_rerun.d, ...
                                                              old_platoon_traj);
                            if ~allowEdge_rerun_check
                                continue; 
                            end
                            E_link_rerun_val = delta_t_rerun_edge * (b1_opt_cost*abs(v_link_rerun_val) + b2_opt_cost*v_link_rerun_val^2);
                            Edges_rerun(edgeCount_rerun) = struct('from', nodeA_rerun.id, 'to', nodeB_rerun.id, 'w', E_link_rerun_val);
                            edgeCount_rerun = edgeCount_rerun + 1;
                        end
                    end
                end
            end
        end
    end
    
    if isempty(Edges_rerun) && nNodes_rerun > 1
         fprintf('[WARNING] No edges created for Dijkstra in rerun. Path cost will be Inf.\n');
    end
    [path_ids_rerun, cost_rerun] = dijkstra(Nodes_rerun, Edges_rerun, Nodes_rerun(1).id, Nodes_rerun(nNodes_rerun).id);
    fprintf('>>> New Platoon Leader=%d, Optimal cost=%.3f\n', leader_vehicle, cost_rerun);

    invalid_cost_rerun = isscalar(cost_rerun) && isinf(cost_rerun);
    invalid_path_rerun = isempty(path_ids_rerun) || (length(path_ids_rerun) < 2 && ~(Nodes_rerun(1).id == Nodes_rerun(nNodes_rerun).id) );


    if invalid_cost_rerun || invalid_path_rerun
        fprintf('[ERROR] No valid path found for new platoon (L%d). Creating a safe default path.\n', leader_vehicle);
        time_to_final_def = (final_distance_sim - start_position) / v_min_opt;
        if time_to_final_def < 0, time_to_final_def = tf_this_opt; end
        
        opt_t_rerun = [start_time; start_time + min(tf_this_opt, time_to_final_def)];
        opt_d_rerun = [start_position; final_distance_sim];
        if opt_d_rerun(end) < opt_d_rerun(1), opt_d_rerun(end) = opt_d_rerun(1); end 
        if opt_t_rerun(end) <= opt_t_rerun(1), opt_t_rerun(end) = opt_t_rerun(1)+1; end

        dt_p = opt_t_rerun(2)-opt_t_rerun(1);
        dd_p = opt_d_rerun(2)-opt_d_rerun(1);
        if dt_p > 1e-3, speeds_rerun = [max(0, dd_p / dt_p)]; else, speeds_rerun = [v_min_opt]; end
        speeds_rerun = max(v_min_opt/2, min(speeds_rerun, v_max_opt)); 
    else
        opt_nodes_path_rerun = [];
        for p_id_rerun = path_ids_rerun
            found_node_rerun = Nodes_rerun([Nodes_rerun.id] == p_id_rerun);
            opt_nodes_path_rerun = [opt_nodes_path_rerun, found_node_rerun]; %#ok<AGROW>
        end
        opt_t_rerun = [opt_nodes_path_rerun.t];
        opt_d_rerun = [opt_nodes_path_rerun.d];
        speeds_rerun = zeros(1, length(opt_t_rerun) - 1);
        for k_sp = 1:(length(opt_t_rerun) - 1)
            dd_sp = opt_d_rerun(k_sp+1) - opt_d_rerun(k_sp);
            dt_sp = opt_t_rerun(k_sp+1) - opt_t_rerun(k_sp);
            if dt_sp < 1e-3, speeds_rerun(k_sp) = v_min_opt; 
            else, speeds_rerun(k_sp) = max(v_min_opt, dd_sp / dt_sp); end
            speeds_rerun(k_sp) = min(speeds_rerun(k_sp), v_max_opt);
        end
    end
    if isempty(speeds_rerun) && length(opt_t_rerun)==1, speeds_rerun = 0; % Fermo
    elseif isempty(speeds_rerun), speeds_rerun = v_min_opt; 
    end

    n_vehicles_total_sim = size(SIM_RUNS{1}.x, 2) / 2;
    
    vehicles_in_new_platoon_list = leader_vehicle:n_vehicles_total_sim; 
    source_run_data_for_split = SIM_RUNS{end};
    if isfield(source_run_data_for_split, 'splittedVehicles') && ~isempty(source_run_data_for_split.splittedVehicles)
         vehicles_in_new_platoon_list = source_run_data_for_split.splittedVehicles;
    end
    
    m_vehicles_list = 1000 * ones(1, n_vehicles_total_sim);
    v_targets_for_new_platoon_sim = speeds_rerun;
    
    K_p_smc_l_rerun = 700; K_r_smc_l_rerun = 100; phi_smc_l_rerun = 0.8;
    K_p_smc_f_rerun = 200; K_r_smc_f_rerun = 80; phi_smc_f_rerun = 0.8; lambda_smc_f_rerun = 0.8;
    
    smc_leader_params_rerun = struct('Kp', K_p_smc_l_rerun, 'Kr', K_r_smc_l_rerun, 'phi', phi_smc_l_rerun);
    smc_follower_params_rerun = struct('Kp', K_p_smc_f_rerun, 'Kr', K_r_smc_f_rerun, 'phi', phi_smc_f_rerun, 'lambda', lambda_smc_f_rerun);

    t_CTH_rerun = 1.5; d_init_spacing_rerun = 5;

    x0_rerun = zeros(2*n_vehicles_total_sim, 1);
    x0_rerun(1:n_vehicles_total_sim) = -10000; 
    x0_rerun(n_vehicles_total_sim+1 : 2*n_vehicles_total_sim) = 0;

    leader_idx_in_new_platoon = find(vehicles_in_new_platoon_list == leader_vehicle,1); % Indice del leader nel suo nuovo plotone

    for v_list_idx = 1:length(vehicles_in_new_platoon_list)
        veh_id_curr = vehicles_in_new_platoon_list(v_list_idx);
        if veh_id_curr == leader_vehicle
            x0_rerun(veh_id_curr) = start_position;
        else
            % Offset rispetto al leader del *nuovo* plotone
            % num_cars_behind_leader = v_list_idx - leader_idx_in_new_platoon; % Funziona se ordinato
            % Per robustezza, calcola la posizione relativa rispetto al leader ID.
            % Questo assume che gli ID siano in ordine crescente nel plotone.
            % Se vehicles_in_new_platoon_list = [L, F1, F2], allora
            % F1 è 1 posto dietro L, F2 è 2 posti dietro L.
            
            % Trova l'indice del leader_vehicle all'interno di vehicles_in_new_platoon_list
            idx_of_leader_in_list = find(vehicles_in_new_platoon_list == leader_vehicle, 1);
            % L'indice di veh_id_curr è v_list_idx
            % Quanti veicoli ci sono tra il leader e questo veicolo (inclusi loro)?
            % No, quanti veicoli sono TRA il leader e questo follower.
            
            % Semplificazione: l'offset è basato sull'ordine in vehicles_in_new_platoon_list
            % rispetto al leader del nuovo plotone.
            num_positions_behind_new_leader = v_list_idx - idx_of_leader_in_list;
            x0_rerun(veh_id_curr) = start_position - d_init_spacing_rerun * num_positions_behind_new_leader;
        end
        x0_rerun(n_vehicles_total_sim + veh_id_curr) = 0; % Velocità iniziale nulla per i membri del nuovo plotone
                                                          % O si potrebbe usare una velocità bassa (es. v_min_opt/2)
    end

    time_remaining_in_global_sim = (SIM_RUNS{1}.offset + final_time_sim) - start_time;
    tf_for_this_new_run = min(tf_this_opt, time_remaining_in_global_sim);
    if tf_for_this_new_run <= 0, tf_for_this_new_run = 10; end 

    t_span_rerun = [start_time, start_time + tf_for_this_new_run];
    if t_span_rerun(2) <= t_span_rerun(1), t_span_rerun(2) = t_span_rerun(1)+10; end % Assicura intervallo valido
    
    [t_sim_rerun, x_sim_rerun] = ode45(@(t_ode,x_ode_state) system_dynamics_new_platoon( ...
        t_ode, x_ode_state, n_vehicles_total_sim, m_vehicles_list, delta_func_rerun, traffic_lights_cfg, ...
        v_targets_for_new_platoon_sim, t_CTH_rerun, ...
        smc_leader_params_rerun, smc_follower_params_rerun, ...
        leader_vehicle, start_time), t_span_rerun, x0_rerun);

    SIM_RUNS{end+1} = struct( ...
        'leader', leader_vehicle, ...
        't', t_sim_rerun, ...
        'x', x_sim_rerun, ...
        'offset', start_time, ...
        'traffic_lights', traffic_lights_cfg, ...
        'splittedVehicles', [], ... 
        'v_targets', v_targets_for_new_platoon_sim, ...
        'opt_t', opt_t_rerun, ...
        'opt_d', opt_d_rerun, ...
        'oldPlatoonTrajectory', old_platoon_traj ...
    );

    check_red_light_violations(t_sim_rerun, x_sim_rerun, traffic_lights_cfg, SIM_RUNS{1}.traffic_lights(1).cycle_time); % Usa cycle time da config
    check_velocity_triggers(t_sim_rerun, x_sim_rerun, traffic_lights_cfg);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Funzione che recupera la traiettoria completa (tempo e posizione) 
% dell'ultimo plotone “valido” (cioè l’ultimo in SIM_RUNS).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function oldPlatoon = get_old_platoon_trajectory()
    global SIM_RUNS
    if isempty(SIM_RUNS) || length(SIM_RUNS) < 1 
        oldPlatoon = [];
    else
        oldPlatoonData = SIM_RUNS{end}; % Il run precedente è l'ultimo aggiunto a SIM_RUNS
        
        active_vehicles_old_platoon = get_platoon_vehicles(length(SIM_RUNS)); 

        if isempty(active_vehicles_old_platoon) || ~isfield(oldPlatoonData,'t') || isempty(oldPlatoonData.t)
             oldPlatoon = []; 
             return;
        end
        
        oldPlatoon = struct('t', oldPlatoonData.t, 'x', oldPlatoonData.x, 'leader', oldPlatoonData.leader);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Verifica se un arco (da tempo tA a tB e posizione dA a dB) viola 
% l’autonomia del vecchio plotone. Se sì, ritorna false, altrimenti true.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function allow = check_non_overlapping(tA, tB, dA, dB, oldPlatoonTrajectory) % Rinomina oldPlatoon
    allow = true;
    if isempty(oldPlatoonTrajectory) || ~isfield(oldPlatoonTrajectory,'t') || isempty(oldPlatoonTrajectory.t) || ...
       ~isfield(oldPlatoonTrajectory,'x') || isempty(oldPlatoonTrajectory.x) || ...
       ~isfield(oldPlatoonTrajectory,'leader')
        return;
    end
    
    times_old_traj = oldPlatoonTrajectory.t;
    x_old_all_veh_traj = oldPlatoonTrajectory.x; 
    old_leader_id_traj = oldPlatoonTrajectory.leader;
    
    n_total_vehicles_old = size(x_old_all_veh_traj,2)/2;

    if old_leader_id_traj <= 0 || old_leader_id_traj > n_total_vehicles_old
        return; % Leader del vecchio plotone non valido
    end
    x_pos_old_leader = x_old_all_veh_traj(:, old_leader_id_traj);
    
    num_check_points_overlap = 5; 
    if tB <= tA + 1e-3 % Se l'arco è un punto o quasi
        t_check_points_arc = [tA]; 
    else
        t_check_points_arc = linspace(tA, tB, num_check_points_overlap); 
    end

    min_safety_dist_overlap = 2.0; % m, distanza di sicurezza minima

    for t_check = t_check_points_arc
        if t_check > times_old_traj(end) + 1e-3 || t_check < times_old_traj(1) - 1e-3 % Tolleranza per fine intervallo
            continue; 
        end
        
        if tB <= tA + 1e-3 
            pos_new_arc_at_t_check = dA; 
        else
            pos_new_arc_at_t_check = interp1([tA, tB], [dA, dB], t_check, 'linear');
        end

        pos_old_leader_at_t_check = interp1(times_old_traj, x_pos_old_leader, t_check, 'linear', 'extrap'); % Extrap per robustezza ai bordi

        if pos_new_arc_at_t_check > pos_old_leader_at_t_check - min_safety_dist_overlap
            allow = false;
            return;
        end
    end
end


function result = is_in_current_platoon(vehicle_id, current_run_idx_check) % Rinomina current_run_idx
    global SIM_RUNS
    result = false; 
    
    if current_run_idx_check < 1 || current_run_idx_check > length(SIM_RUNS)
        return; 
    end

    current_run_struct = SIM_RUNS{current_run_idx_check};
    
    if current_run_idx_check == 1
        % Nel primo run, tutti i veicoli sono considerati parte del plotone iniziale
        % La logica di splittedVehicles nel run 1 indica chi si staccherà *alla fine* del run 1.
        result = true; 
        return;
    end
    
    % Per run > 1:
    prev_run_struct = SIM_RUNS{current_run_idx_check - 1};
    
    if isfield(prev_run_struct, 'splittedVehicles') && ...
       ~isempty(prev_run_struct.splittedVehicles) && ...
       ismember(vehicle_id, prev_run_struct.splittedVehicles) && ...
       current_run_struct.leader == prev_run_struct.splittedVehicles(1) 
        % Il veicolo era tra quelli che si sono staccati dal run precedente
        % e il leader di questo run è il leader atteso di quel gruppo staccato.
        result = true;
    else
        result = false;
    end
end

function final_plot()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[final_plot] No data to plot.');
        return;
    end
    
    plot_optimal_trajectories_and_lights(); 
    plot_real_trajectories();               
    plot_comparison();                      
    plot_leader_velocities();               
    plot_energy_consumption();              
    plot_inter_vehicle_distances();         

    save_plot_data(SIM_RUNS); 
end


function [legend_handles_out, legend_texts_out] = plot_run_optimal_trajectories(~, ~, ~, ~, ~, ~) % Rimuovi run_i e altri non usati
    % QUESTA FUNZIONE plot_run_optimal_trajectories SEMBRA NON ESSERE CHIAMATA DA NESSUNA PARTE.
    % LA LOGICA DI PLOT DELLE TRAIETTORIE OTTIMALI È IN plot_optimal_trajectories_and_lights.
    % Se necessario, questa funzione andrebbe rivista o rimossa.
    % Per ora la lascio vuota per evitare errori se qualche vecchio riferimento la chiamasse.
    legend_handles_out = [];
    legend_texts_out = {};
    % global SIM_RUNS
    % 
    % runData = SIM_RUNS{run_i};
    % leader = runData.leader;
    % 
    % if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d') || isempty(runData.opt_t)
    %     return; 
    % end
    % 
    % opt_t = runData.opt_t;
    % opt_d = runData.opt_d;
    % 
    % platoon_vehicles_in_run = get_platoon_vehicles(run_i);
    % 
    % v_targets = calculate_target_velocities(opt_t, opt_d); 
    % 
    % color_idx_leader = mod(leader-1, length(colors))+1;
    % line_idx_run = mod(run_i-1, length(line_styles))+1; 
    % marker_idx_leader = mod(leader-1, length(markers))+1;
    % 
    % h_leader_opt = plot(opt_t, opt_d, [colors{color_idx_leader}, line_styles{line_idx_run}], 'LineWidth', 2.5, 'LineStyle','--');
    % scatter(opt_t, opt_d, 60, colors{color_idx_leader}, markers{marker_idx_leader}, 'filled', 'MarkerEdgeColor','k');
    % 
    % if ~any(legend_handles == h_leader_opt) 
    %     legend_handles(end+1) = h_leader_opt;
    %     legend_texts{end+1} = sprintf('L%d Opt. (Plt %d)', leader, run_i);
    % end
    % 
    % for v_idx = 1:length(platoon_vehicles_in_run)
    %     v = platoon_vehicles_in_run(v_idx);
    %     if v == leader 
    %         continue; 
    %     end
    %         
    %     follower_opt_d = calculate_follower_trajectory(v, leader, opt_t, opt_d, v_targets, platoon_vehicles_in_run);
    %     
    %     color_idx_follower = mod(v-1, length(colors))+1;
    %     
    %     h_follower_opt = plot(opt_t, follower_opt_d, ...
    %         [colors{color_idx_follower}, line_styles{line_idx_run}], 'LineWidth', 1.5, 'LineStyle',':');
    %         
    %     if ~any(legend_handles == h_follower_opt)
    %         legend_handles(end+1) = h_follower_opt;
    %         legend_texts{end+1} = sprintf('V%d Opt. (Plt %d)', v, run_i);
    %     end
    % end
    % legend_handles_out = legend_handles; % Assegna output
    % legend_texts_out = legend_texts;     % Assegna output
end

function plot_comparison()
    global SIM_RUNS
    if isempty(SIM_RUNS), disp('No data for comparison plot.'); return; end

    figure('Name', 'Real vs Optimal + Traffic Lights Comparison', 'Position', [200, 200, 1200, 700]);
    hold on; grid on;

    traffic_lights_cfg_comp = SIM_RUNS{1}.traffic_lights;
    [tl_times_comp, tl_distances_comp, tl_colors_comp] = prepare_traffic_light_data(traffic_lights_cfg_comp);
    if ~isempty(tl_times_comp)
        scatter(tl_times_comp, tl_distances_comp, 15, tl_colors_comp, 'filled', 'Marker', 's', 'DisplayName', 'Traffic Lights');
    end

    veh_colors_comp = define_vehicle_colors();
        
    legend_handles_comp = [];
    legend_texts_comp = {};

    num_veh_total_comp = size(SIM_RUNS{1}.x,2)/2;
    plotted_veh_real_legend_comp = false(1, num_veh_total_comp);
    plotted_veh_opt_legend_comp = false(1, num_veh_total_comp);


    for run_idx_comp = 1:length(SIM_RUNS)
        runData_comp = SIM_RUNS{run_idx_comp};
        t_real_comp = runData_comp.t;
        x_real_comp = runData_comp.x;
        leader_comp = runData_comp.leader;
        
        vehicles_in_platoon_comp = get_platoon_vehicles(run_idx_comp);
        if isempty(vehicles_in_platoon_comp), continue; end

        % Traiettorie reali
        for v_plt_idx = 1:length(vehicles_in_platoon_comp)
            v_id_comp = vehicles_in_platoon_comp(v_plt_idx);
            if v_id_comp > num_veh_total_comp || v_id_comp < 1, continue; end

            color_v_comp = veh_colors_comp(mod(v_id_comp-1, size(veh_colors_comp,1))+1, :);
            
            t_plot_end_real_comp = t_real_comp(end);
            if isfield(runData_comp, 'splittedVehicles') && ~isempty(runData_comp.splittedVehicles) && ismember(v_id_comp, runData_comp.splittedVehicles)
                if run_idx_comp < length(SIM_RUNS)
                     next_run_offset_comp = SIM_RUNS{run_idx_comp+1}.offset;
                     t_plot_end_real_comp = min(t_plot_end_real_comp, next_run_offset_comp);
                end
            end
            plot_mask_real_comp = t_real_comp <= t_plot_end_real_comp & t_real_comp >= runData_comp.offset;
            if sum(plot_mask_real_comp) < 2, continue; end

            h_real_plt = plot(t_real_comp(plot_mask_real_comp), x_real_comp(plot_mask_real_comp, v_id_comp), ...
                          '-', 'Color', color_v_comp, 'LineWidth', 2);
            
            if ~plotted_veh_real_legend_comp(v_id_comp)
                set(h_real_plt, 'DisplayName', sprintf('V%d Real', v_id_comp));
                legend_handles_comp(end+1) = h_real_plt;
                legend_texts_comp{end+1} = sprintf('V%d Real', v_id_comp);
                plotted_veh_real_legend_comp(v_id_comp) = true;
            end
        end

        % Traiettorie ottimali
        if isfield(runData_comp, 'opt_t') && isfield(runData_comp, 'opt_d') && ~isempty(runData_comp.opt_t)
            opt_t_comp = runData_comp.opt_t;
            opt_d_leader_comp = runData_comp.opt_d;
            v_targets_opt_comp = calculate_target_velocities(opt_t_comp, opt_d_leader_comp);

            color_leader_comp = veh_colors_comp(mod(leader_comp-1, size(veh_colors_comp,1))+1, :);
            h_opt_leader_plt = plot(opt_t_comp, opt_d_leader_comp, '--', 'Color', color_leader_comp, 'LineWidth', 1.5);
            if ~plotted_veh_opt_legend_comp(leader_comp)
                set(h_opt_leader_plt, 'DisplayName', sprintf('L%d Opt (Plt %d)', leader_comp, run_idx_comp));
                legend_handles_comp(end+1) = h_opt_leader_plt;
                legend_texts_comp{end+1} = sprintf('L%d Opt (Plt %d)', leader_comp, run_idx_comp);
                plotted_veh_opt_legend_comp(leader_comp) = true;
            end


            for v_plt_idx = 1:length(vehicles_in_platoon_comp)
                v_id_comp_foll = vehicles_in_platoon_comp(v_plt_idx);
                if v_id_comp_foll == leader_comp || v_id_comp_foll > num_veh_total_comp || v_id_comp_foll < 1, continue; end
                
                color_v_foll_comp = veh_colors_comp(mod(v_id_comp_foll-1, size(veh_colors_comp,1))+1, :);
                opt_d_follower_comp = calculate_follower_trajectory(v_id_comp_foll, leader_comp, opt_t_comp, opt_d_leader_comp, v_targets_opt_comp, vehicles_in_platoon_comp);
                if ~isempty(opt_d_follower_comp)
                    h_opt_foll_plt = plot(opt_t_comp, opt_d_follower_comp, ':', 'Color', color_v_foll_comp, 'LineWidth', 1.0);
                     if ~plotted_veh_opt_legend_comp(v_id_comp_foll)
                        set(h_opt_foll_plt, 'DisplayName', sprintf('V%d Opt (Plt %d)', v_id_comp_foll, run_idx_comp));
                        legend_handles_comp(end+1) = h_opt_foll_plt;
                        legend_texts_comp{end+1} = sprintf('V%d Opt (Plt %d)', v_id_comp_foll, run_idx_comp);
                        plotted_veh_opt_legend_comp(v_id_comp_foll) = true;
                    end
                end
            end
        end
    end
    
    max_sim_time_comp = get_global_time_and_distance();
    xlim([0, max_sim_time_comp]);

    if ~isempty(legend_handles_comp)
        legend(legend_handles_comp, legend_texts_comp, 'Location', 'BestOutside', 'NumColumns', 1);
    end
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('Position [m]', 'FontSize', 12);
    title('Comparison: Real vs Optimal Trajectories & Traffic Lights', 'FontSize', 14);
end

function first_run_idx = find_first_run_for_vehicle(vehicle_id_find) % Rinomina vehicle_id
    global SIM_RUNS
    first_run_idx = -1; % Default se non trovato
    if isempty(SIM_RUNS), return; end
    for r_find = 1:length(SIM_RUNS)
        vehicles_in_run_r_find = get_platoon_vehicles(r_find);
        if ismember(vehicle_id_find, vehicles_in_run_r_find)
            first_run_idx = r_find;
            return;
        end
    end
end


function plot_leader_velocities()
    global SIM_RUNS
    if isempty(SIM_RUNS), disp('No data for leader velocities plot.'); return; end

    unique_leader_ids = []; % Rinomina unique_leaders
    for run_idx_lv = 1:length(SIM_RUNS) % Rinomina run_i
        if ~ismember(SIM_RUNS{run_idx_lv}.leader, unique_leader_ids)
            unique_leader_ids = [unique_leader_ids, SIM_RUNS{run_idx_lv}.leader];
        end
    end
    
    veh_colors_lv = define_vehicle_colors(); % Rinomina vehicle_colors

    for l_id_idx = 1:length(unique_leader_ids) % Rinomina l_idx
        current_leader_id_val = unique_leader_ids(l_id_idx); % Rinomina current_leader_id
        figure('Name', ['Leader Velocity Profile: Vehicle ' num2str(current_leader_id_val)], 'Position', [250+l_id_idx*30, 250+l_id_idx*30, 900, 550]);
        hold on; grid on;
        
        legend_handles_lv = []; % Rinomina legend_h
        legend_texts_lv = {};   % Rinomina legend_t
        
        max_vel_overall_lv = 0; % Rinomina max_overall_vel

        for run_idx_lv_inner = 1:length(SIM_RUNS) % Rinomina run_i
            runData_lv = SIM_RUNS{run_idx_lv_inner}; % Rinomina runData
            if runData_lv.leader == current_leader_id_val
                t_real_lv = runData_lv.t; % Rinomina t_real
                x_real_lv = runData_lv.x; % Rinomina x_real
                num_veh_total_lv = size(x_real_lv, 2)/2; % Rinomina num_veh_total
                
                if current_leader_id_val > num_veh_total_lv || isempty(t_real_lv) || length(t_real_lv)<2, continue; end 

                leader_vel_real_lv = x_real_lv(:, num_veh_total_lv + current_leader_id_val); % Rinomina leader_velocity_real
                max_vel_overall_lv = max([max_vel_overall_lv; leader_vel_real_lv(:)]); % Aggiunto (:), robustezza

                t_plot_end_vel_lv = t_real_lv(end); % Rinomina t_plot_end_vel
                % Logica di troncamento del plot se il leader si stacca
                if isfield(runData_lv, 'splittedVehicles') && ~isempty(runData_lv.splittedVehicles) && ismember(current_leader_id_val, runData_lv.splittedVehicles)
                     if run_idx_lv_inner < length(SIM_RUNS)
                         next_run_offset_lv = SIM_RUNS{run_idx_lv_inner+1}.offset;
                         t_plot_end_vel_lv = min(t_plot_end_vel_lv, next_run_offset_lv);
                     end
                end
                plot_mask_vel_lv = t_real_lv <= t_plot_end_vel_lv & t_real_lv >= runData_lv.offset; % Rinomina e aggiungi >= offset
                if sum(plot_mask_vel_lv) <2, continue; end

                color_leader_lv = veh_colors_lv(mod(current_leader_id_val-1, size(veh_colors_lv,1))+1, :);
                
                h_real_plot_lv = plot(t_real_lv(plot_mask_vel_lv), leader_vel_real_lv(plot_mask_vel_lv), '-', 'Color', color_leader_lv, 'LineWidth', 2.5);
                
                legend_entry_real_speed = sprintf('V%d Real Speed (Plt %d)', current_leader_id_val, run_idx_lv_inner);
                is_already_in_legend_real = false;
                for leg_txt_idx = 1:length(legend_texts_lv)
                    if strcmp(legend_texts_lv{leg_txt_idx}, legend_entry_real_speed)
                        is_already_in_legend_real = true; break;
                    end
                end
                if ~is_already_in_legend_real
                    legend_handles_lv(end+1) = h_real_plot_lv;
                    legend_texts_lv{end+1} = legend_entry_real_speed;
                end


                if isfield(runData_lv, 'opt_t') && isfield(runData_lv, 'opt_d') && ~isempty(runData_lv.opt_t) && length(runData_lv.opt_t)>1
                    opt_t_leader_lv = runData_lv.opt_t; % Rinomina
                    opt_d_leader_lv = runData_lv.opt_d; % Rinomina
                    opt_v_leader_segs_lv = calculate_target_velocities(opt_t_leader_lv, opt_d_leader_lv); % Rinomina
                    
                    if ~isempty(opt_v_leader_segs_lv)
                        max_vel_overall_lv = max([max_vel_overall_lv; opt_v_leader_segs_lv(:)]); % Aggiunto (:), robustezza

                        h_opt_plot_lv = []; % Per la legenda
                        for k_opt_lv = 1:length(opt_v_leader_segs_lv)
                            t_segment_lv = [opt_t_leader_lv(k_opt_lv), opt_t_leader_lv(k_opt_lv+1)];
                            v_segment_lv = [opt_v_leader_segs_lv(k_opt_lv), opt_v_leader_segs_lv(k_opt_lv)];
                            
                            current_h_opt = plot(t_segment_lv, v_segment_lv, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 2);
                            if k_opt_lv == 1, h_opt_plot_lv = current_h_opt; end % Prendi handle del primo segmento per legenda
                        end
                        
                        legend_entry_target_speed = sprintf('Target Speed (Plt %d)', run_idx_lv_inner);
                        is_already_in_legend_target = false;
                        for leg_txt_idx = 1:length(legend_texts_lv)
                            if strcmp(legend_texts_lv{leg_txt_idx}, legend_entry_target_speed)
                                is_already_in_legend_target = true; break;
                            end
                        end
                        if ~is_already_in_legend_target && ~isempty(h_opt_plot_lv)
                             legend_handles_lv(end+1) = h_opt_plot_lv; 
                             legend_texts_lv{end+1} = legend_entry_target_speed;
                        end
                        scatter(opt_t_leader_lv(1:end-1), opt_v_leader_segs_lv, 50, 'o', 'MarkerEdgeColor', [0.3 0.3 0.3], 'MarkerFaceColor', [0.7 0.7 0.7]);
                    end
                end
            end
        end
        
        if ~isempty(legend_handles_lv)
            legend(legend_handles_lv, legend_texts_lv, 'Location', 'best');
        end
        xlabel('Time [s]', 'FontSize', 12);
        ylabel('Speed [m/s]', 'FontSize', 12);
        title(sprintf('Velocity Profile: Vehicle %d (when Leader)', current_leader_id_val), 'FontSize', 14);
        
        if max_vel_overall_lv > 0
            ylim_top = max(35, ceil(max_vel_overall_lv/5)*5 + 5); % Arrotonda e aggiungi margine
            ylim([0, ylim_top]); 
        else
            ylim([0,35]);
        end
    end
end


function plot_energy_consumption()
    global SIM_RUNS
    if isempty(SIM_RUNS), disp('No data for energy plot.'); return; end

    figure('Name', 'Approximated Energy Consumption per Platoon Segment', 'Position', [300, 300, 900, 550]);
    hold on; grid on;
    
    num_runs_ec = length(SIM_RUNS); % Rinomina num_runs
    run_energies_ec = zeros(1, num_runs_ec); % Rinomina run_energies
    run_labels_ec = cell(1, num_runs_ec);   % Rinomina run_labels
    
    b1_en = 0.1; 
    b2_en = 0.01;
    
    bar_colors_ec = jet(num_runs_ec); % Rinomina bar_colors

    for run_idx_ec = 1:length(SIM_RUNS) % Rinomina run_i
        runData_ec = SIM_RUNS{run_idx_ec}; % Rinomina runData
        t_ec = runData_ec.t; % Rinomina t
        x_ec = runData_ec.x; % Rinomina x
        leader_ec = runData_ec.leader; % Rinomina current_leader
        
        if isempty(t_ec) || length(t_ec) < 2, continue; end

        n_veh_total_ec = size(x_ec, 2)/2; % Rinomina n_vehicles_total
        platoon_run_energy_ec = 0; % Rinomina platoon_total_energy_this_run
        
        vehicles_in_platoon_ec = get_platoon_vehicles(run_idx_ec); % Rinomina
        if isempty(vehicles_in_platoon_ec), continue; end

        active_veh_count_ec = 0; % Rinomina num_active_vehicles_in_run

        for v_list_idx_ec = 1:length(vehicles_in_platoon_ec) % Rinomina v_idx
            v_id_ec = vehicles_in_platoon_ec(v_list_idx_ec); % Rinomina v
            if v_id_ec > n_veh_total_ec || v_id_ec < 1, continue; end

            velocity_v_ec = x_ec(:, n_veh_total_ec + v_id_ec); % Rinomina velocity_v
            
            t_plot_end_veh_ec = t_ec(end); % Rinomina t_plot_end_vehicle
            if isfield(runData_ec, 'splittedVehicles') && ~isempty(runData_ec.splittedVehicles) && ismember(v_id_ec, runData_ec.splittedVehicles)
                 if run_idx_ec < length(SIM_RUNS)
                     next_run_offset_ec = SIM_RUNS{run_idx_ec+1}.offset; % Rinomina
                     t_plot_end_veh_ec = min(t_plot_end_veh_ec, next_run_offset_ec);
                 end
            end
            time_mask_veh_ec = (t_ec >= runData_ec.offset) & (t_ec <= t_plot_end_veh_ec); % Rinomina
            
            if sum(time_mask_veh_ec) < 2, continue; end

            t_masked_ec = t_ec(time_mask_veh_ec); % Rinomina
            velocity_v_masked_ec = velocity_v_ec(time_mask_veh_ec); % Rinomina
            
            dt_segments_ec = diff(t_masked_ec); % Rinomina
            power_segments_ec = b1_en * velocity_v_masked_ec(1:end-1) + b2_en * velocity_v_masked_ec(1:end-1).^2; % Rinomina
            power_segments_ec(power_segments_ec < 0) = 0; 
            
            segment_energies_ec = power_segments_ec .* dt_segments_ec; % Rinomina
            platoon_run_energy_ec = platoon_run_energy_ec + sum(segment_energies_ec);
            active_veh_count_ec = active_veh_count_ec +1;
        end
        
        run_energies_ec(run_idx_ec) = platoon_run_energy_ec;
        run_labels_ec{run_idx_ec} = sprintf('Plt %d (L%d, %d veh)', run_idx_ec, leader_ec, active_veh_count_ec);
    end
    
    if any(run_energies_ec > 0)
        bar_h_ec = bar(1:num_runs_ec, run_energies_ec); % Rinomina bar_h
        set(bar_h_ec, 'FaceColor', 'flat'); 
        for k_bar_ec = 1:num_runs_ec % Rinomina k_bar
            bar_h_ec.CData(k_bar_ec,:) = bar_colors_ec(k_bar_ec,:);
        end

        for i_ec = 1:num_runs_ec % Rinomina i
            if run_energies_ec(i_ec) > 0
                text(i_ec, run_energies_ec(i_ec), sprintf('%.1f kJ', run_energies_ec(i_ec)/1000), ...
                     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
            end
        end
    else
        text(0.5, 0.5, 'No energy consumption data to plot.', 'HorizontalAlignment', 'center', 'Units', 'normalized');
    end
    
    set(gca, 'XTick', 1:num_runs_ec, 'XTickLabel', run_labels_ec, 'XTickLabelRotation', 30);
    ylabel('Approximated Energy [J]', 'FontSize', 12);
    title('Energy Consumption per Platoon Segment (sum over vehicles)', 'FontSize', 14);
    
    if any(run_energies_ec > 0)
        ylim([0, max(run_energies_ec) * 1.20]); % Aumentato margine
    end
end


function plot_inter_vehicle_distances()
    global SIM_RUNS
    if isempty(SIM_RUNS), disp('No data for inter-vehicle distance plot.'); return; end

    figure_name_ivd = 'Inter-Vehicle Distances within Platoons';
    existing_fig_ivd = findobj('Type', 'Figure', 'Name', figure_name_ivd);
    if ~isempty(existing_fig_ivd), clf(existing_fig_ivd); figure(existing_fig_ivd); % Pulisci e riusa
    else, figure('Name', figure_name_ivd, 'Position', [350, 350, 1100, 700]);
    end
    
    num_runs_ivd = length(SIM_RUNS);
    ncols_ivd = floor(sqrt(num_runs_ivd));
    if ncols_ivd == 0, ncols_ivd = 1; end % Evita divisione per zero se num_runs_ivd = 0
    nrows_ivd = ceil(num_runs_ivd/ncols_ivd);
    if num_runs_ivd == 0, nrows_ivd=0; end % Evita subplot se 0 runs

    veh_colors_ivd = define_vehicle_colors(); 

    for run_idx_ivd = 1:num_runs_ivd % Rinomina run_i
        if nrows_ivd > 0 && ncols_ivd > 0 % Solo se ci sono subplot da creare
             subplot(nrows_ivd, ncols_ivd, run_idx_ivd);
        else % No subplot, plot diretto (improbabile se num_runs_ivd > 0)
            hold on; grid on;
        end
        hold on; grid on;


        runData_ivd = SIM_RUNS{run_idx_ivd};
        t_ivd = runData_ivd.t;
        x_all_ivd = runData_ivd.x;
        leader_ivd = runData_ivd.leader;
        
        if isempty(t_ivd) || length(t_ivd) < 2, title(sprintf('Plt %d (L%d) - No Sim Data', run_idx_ivd, leader_ivd)); continue; end

        vehicles_platoon_ivd = get_platoon_vehicles(run_idx_ivd);
        
        if length(vehicles_platoon_ivd) > 1
            mean_pos_ivd = zeros(1, length(vehicles_platoon_ivd));
            valid_indices_for_sort = [];
            for v_s_idx = 1:length(vehicles_platoon_ivd)
                v_s = vehicles_platoon_ivd(v_s_idx);
                if v_s <= size(x_all_ivd,2)/2 && v_s > 0 % Verifica validità ID veicolo
                    mean_pos_ivd(v_s_idx) = mean(x_all_ivd(:,v_s));
                    valid_indices_for_sort(end+1) = v_s_idx; %#ok<AGROW>
                end
            end
            if isempty(valid_indices_for_sort)
                sorted_vehicles_platoon_ivd = [];
            else
                vehicles_to_sort = vehicles_platoon_ivd(valid_indices_for_sort);
                mean_pos_to_sort = mean_pos_ivd(valid_indices_for_sort);
                [~, sort_order_ivd] = sort(mean_pos_to_sort, 'descend'); 
                sorted_vehicles_platoon_ivd = vehicles_to_sort(sort_order_ivd);
            end
        else
            sorted_vehicles_platoon_ivd = vehicles_platoon_ivd;
        end
        
        legend_h_dist_ivd = []; % Rinomina
        legend_t_dist_ivd = {}; % Rinomina
        max_dist_plot_val = 0;  % Rinomina

        if length(sorted_vehicles_platoon_ivd) >= 2
            for k_ivd = 1:(length(sorted_vehicles_platoon_ivd)-1) % Rinomina k
                id_veh_front_ivd = sorted_vehicles_platoon_ivd(k_ivd);     
                id_veh_rear_ivd = sorted_vehicles_platoon_ivd(k_ivd+1); 
                %% CORREZIONE SINTASSI: Usa nomi ASCII
                % id_veh_앞 -> id_veh_front_ivd
                % id_veh_뒤 -> id_veh_rear_ivd

                dist_inter_veh_val = x_all_ivd(:, id_veh_front_ivd) - x_all_ivd(:, id_veh_rear_ivd); % Rinomina
                
                vel_veh_rear_val = x_all_ivd(:, size(x_all_ivd,2)/2 + id_veh_rear_ivd); % Rinomina
                d_min_cth = 1; 
                t_CTH_cth = 1.5; 
                dist_desired_cth_val = d_min_cth + t_CTH_cth * vel_veh_rear_val; % Rinomina

                t_start_plot_ivd = runData_ivd.offset; % Rinomina
                t_end_plot_ivd = t_ivd(end);       % Rinomina

                if isfield(runData_ivd, 'splittedVehicles') && ~isempty(runData_ivd.splittedVehicles)
                    if ismember(id_veh_rear_ivd, runData_ivd.splittedVehicles) % Se il veicolo DIETRO si stacca
                        if run_idx_ivd < length(SIM_RUNS)
                            t_end_plot_ivd = min(t_end_plot_ivd, SIM_RUNS{run_idx_ivd+1}.offset);
                        end
                    end
                end
                plot_mask_dist_ivd = (t_ivd >= t_start_plot_ivd) & (t_ivd <= t_end_plot_ivd); % Rinomina
                if sum(plot_mask_dist_ivd) < 2, continue; end

                line_color_ivd = veh_colors_ivd(mod(id_veh_rear_ivd-1, size(veh_colors_ivd,1))+1, :); % Rinomina

                h_actual_ivd = plot(t_ivd(plot_mask_dist_ivd), dist_inter_veh_val(plot_mask_dist_ivd), '-', 'Color', line_color_ivd, 'LineWidth', 2);
                h_desired_ivd = plot(t_ivd(plot_mask_dist_ivd), dist_desired_cth_val(plot_mask_dist_ivd), '--', 'Color', line_color_ivd, 'LineWidth', 1.5);
                
                max_dist_plot_val = max([max_dist_plot_val; dist_inter_veh_val(plot_mask_dist_ivd); dist_desired_cth_val(plot_mask_dist_ivd)]);

                label_actual_ivd = sprintf('D %d-%d Real', id_veh_front_ivd, id_veh_rear_ivd); % Rinomina
                label_desired_ivd = sprintf('D %d-%d Desired', id_veh_front_ivd, id_veh_rear_ivd); % Rinomina
                
                % Aggiungi alla legenda solo una volta per etichetta (combinazione di testo)
                if ~any(strcmp(legend_t_dist_ivd, label_actual_ivd))
                    legend_h_dist_ivd(end+1) = h_actual_ivd;
                    legend_t_dist_ivd{end+1} = label_actual_ivd;
                end
                 if ~any(strcmp(legend_t_dist_ivd, label_desired_ivd))
                    legend_h_dist_ivd(end+1) = h_desired_ivd;
                    legend_t_dist_ivd{end+1} = label_desired_ivd;
                end
            end
            
            if ~isempty(legend_h_dist_ivd)
                 legend(legend_h_dist_ivd, legend_t_dist_ivd, 'Location', 'best', 'FontSize', 7);
            end
            if max_dist_plot_val > 0
                ylim([0, max(20, ceil(max_dist_plot_val/5)*5 + 5)]); % Margine e arrotondamento
            else
                ylim([0,20]); % Default se no max_dist
            end
        else
            text(0.5, 0.5, '1 vehicle or less in platoon.', 'HorizontalAlignment', 'center', 'Units', 'normalized');
            ylim([0,20]);
        end
        
        title(sprintf('Platoon %d (L%d)', run_idx_ivd, leader_ivd), 'FontSize', 10);
        xlabel('Time [s]'); ylabel('Distance [m]');
        if ~isempty(t_ivd)
            xlim([max(0,runData_ivd.offset - 5), t_ivd(end)+5]); % Margine xlim
        end
    end
    
    if num_runs_ivd > 0
        sgtitle('Inter-Vehicle Spacing (Real vs Desired CTH)', 'FontSize', 14, 'FontWeight', 'bold');
    end
end

%% MODIFICA NOTA: Rinominata la funzione wrapper per chiarezza
% Questa funzione gestisce l'evento di split per semaforo rosso e chiama l'ottimizzatore.
function rerun_optimizer_from_traffic_light_EVENT(vehicle_id_event, start_time_event, start_pos_event, traffic_light_obj_event)
    global N_PLATOON SIM_RUNS
    
    % Logica di prevenzione chiamate multiple (opzionale, ma utile)
    % for k_run_check = 1:length(SIM_RUNS)
    %     if SIM_RUNS{k_run_check}.leader == vehicle_id_event && abs(SIM_RUNS{k_run_check}.offset - start_time_event) < 0.5 % Tolleranza
    %         fprintf('[INFO] Traffic light split for V%d at t=%.1f already being processed. Skipping duplicate call.\n', vehicle_id_event, start_time_event);
    %         return;
    %     end
    % end

    N_PLATOON = N_PLATOON + 1; 
    new_platoon_id_event = N_PLATOON; 

    disp(['[EVENT] RED LIGHT SPLIT: Creating PLATOON ' num2str(new_platoon_id_event) ' for LEADER=' ...
          num2str(vehicle_id_event) '. EventTime=~' num2str(start_time_event, '%.1f') 's, EventPos=~' ...
          num2str(start_pos_event, '%.1f') 'm.']);
    
    % Chiama la funzione di ottimizzazione principale per il nuovo plotone
    rerun_optimizer_from_traffic_light(vehicle_id_event, start_time_event, start_pos_event, traffic_light_obj_event);
end


%% CORREZIONE SINTASSI: Rimossa parentesi graffa dalla definizione della funzione
function v_targets_out = calculate_target_velocities(opt_t_cv, opt_d_cv) % Rinomina opt_t, opt_d
    v_targets_out = []; % Rinomina v_targets
    if length(opt_t_cv) < 2
        if length(opt_t_cv) == 1 % Se c'è un solo nodo, velocità è indefinita o zero
             % fprintf('[WARN] opt_t has only 1 point in calc_target_vel. Returning empty/zero target.\n');
             v_targets_out = 0; % o [] a seconda di come viene gestito dopo
        end
        return;
    end
    for i_cv = 1:length(opt_t_cv)-1 % Rinomina i
        delta_d_seg_cv = opt_d_cv(i_cv+1) - opt_d_cv(i_cv); % Rinomina
        delta_t_seg_cv = opt_t_cv(i_cv+1) - opt_t_cv(i_cv); % Rinomina
        if delta_t_seg_cv > 1e-6 
            v_seg_cv = delta_d_seg_cv / delta_t_seg_cv; % Rinomina v
        else
            v_seg_cv = 0; 
            % fprintf('[WARN] Very small delta_t in calc_target_vel. Segment %d. Setting v=0.\n', i_cv);
        end
        v_targets_out(i_cv) = v_seg_cv;
    end
    if isempty(v_targets_out) && length(opt_t_cv) >= 2 % Se v_targets_out è ancora vuoto ma dovrebbe esserci almeno un segmento
        % Questo può accadere se il loop non viene eseguito, es. length(opt_t_cv)-1 < 1
        % Il check length(opt_t_cv) < 2 dovrebbe coprire questo.
        % Per sicurezza:
        warning('v_targets_out is empty in calculate_target_velocities despite >=2 opt_t points.');
        v_targets_out = 0; % Fallback
    end
end

% Modificata per prendere in input platoon_vehicles_in_run
function follower_opt_d_out = calculate_follower_trajectory(follower_id_ct, leader_id_ct, opt_t_ct, opt_d_leader_ct, v_targets_ct, platoon_vehicles_in_run_ct) % Rinomina variabili
    follower_opt_d_out = zeros(size(opt_d_leader_ct)); % Rinomina follower_opt_d
    
    t_CTH_calc = 1.5;  
    d_min_calc = 1.0;    
    d_init_spacing_calc = 5.0; % Spaziatura iniziale usata in x0, dovrebbe essere consistente
    
    if isempty(opt_t_ct) || isempty(opt_d_leader_ct)
        return; 
    end

    leader_idx_in_plt = find(platoon_vehicles_in_run_ct == leader_id_ct, 1);
    follower_idx_in_plt = find(platoon_vehicles_in_run_ct == follower_id_ct, 1);
    
    if isempty(leader_idx_in_plt) || isempty(follower_idx_in_plt)
        warning('calculate_follower_trajectory: Leader or follower not found in platoon_vehicles_in_run_ct list.');
        follower_opt_d_out = opt_d_leader_ct - d_init_spacing_calc; % Semplice fallback per evitare errore
        return;
    end
    
    % Num veicoli EFFETTIVAMENTE TRA il leader e questo follower in questo plotone
    % Se platoon_vehicles_in_run_ct è ordinato per posizione: [Leader, Foll1, Foll2, ...]
    % allora l'offset è (follower_idx_in_plt - leader_idx_in_plt)
    num_intermediate_followers = follower_idx_in_plt - leader_idx_in_plt;
    
    if num_intermediate_followers <= 0 % Follower è il leader o davanti (non dovrebbe essere per CTH dietro)
        % Questo indica un problema con la lista platoon_vehicles_in_run_ct o la logica.
        % Un follower per cui si calcola la traiettoria CTH dovrebbe essere *dietro* il leader.
        % warning('Follower %d is not behind leader %d in platoon list for CTH trajectory.', follower_id_ct, leader_id_ct);
        % Fallback: offset fisso piccolo, o errore.
        initial_total_offset_val = abs(follower_id_ct - leader_id_ct) * d_init_spacing_calc; % Fallback a offset basato su ID
    else
        initial_total_offset_val = num_intermediate_followers * d_init_spacing_calc;
    end


    follower_opt_d_out(1) = opt_d_leader_ct(1) - initial_total_offset_val; 
    
    if length(opt_t_ct) < 2 || isempty(v_targets_ct)
        if length(opt_t_ct) == 1 % Solo un punto
             follower_opt_d_out = opt_d_leader_ct(1) - initial_total_offset_val; % Scalare
        else % Più punti, ma no v_targets, applica offset a tutti
             follower_opt_d_out = opt_d_leader_ct - initial_total_offset_val; % Vettore
        end
        return;
    end

    for idx_opt_ct = 1:(length(opt_t_ct)-1) 
        if idx_opt_ct > length(v_targets_ct) % Sanity check
            warning('Index for v_targets_ct out of bounds in calculate_follower_trajectory');
            current_leader_vel_ct = v_targets_ct(end); % Usa l'ultimo disponibile
        else
            current_leader_vel_ct = v_targets_ct(idx_opt_ct); 
        end
        
        single_desired_gap_val = d_min_calc + t_CTH_calc * current_leader_vel_ct;
        
        % Il gap totale accumulato per questo follower
        total_desired_gap_val = num_intermediate_followers * single_desired_gap_val; 
        if num_intermediate_followers <= 0 % Se era un caso di fallback
            total_desired_gap_val = initial_total_offset_val; % Mantieni l'offset iniziale
        end
        
        follower_opt_d_out(idx_opt_ct+1) = opt_d_leader_ct(idx_opt_ct+1) - total_desired_gap_val;
    end
end


function reset_persistent_variables()
    clear system_dynamics_new_platoon;
    clear check_red_light_violations;
    clear check_velocity_triggers; 
    clear get_current_v_target_indexed;
    clear next_green; 
    clear prev_green; 
    clear dijkstra;   
    % Aggiungi altre funzioni che potrebbero usare 'persistent' se necessario
end

%% MODIFICA NOTA: Implementazione di create_traffic_light
function light = create_traffic_light(distance, green_start_abs_first_cycle, green_end_abs_first_cycle, cycle_time_val)
    light.distance       = distance;
    % Questi sono i tempi assoluti del primo periodo verde
    light.green_start    = green_start_abs_first_cycle; 
    light.green_end      = green_end_abs_first_cycle;   
    light.cycle_time     = cycle_time_val;
    
    % 'offset' è usato nella creazione dei nodi come l'inizio del primo pattern verde.
    % Quindi, light.offset = light.green_start (tempo assoluto del primo inizio verde)
    light.offset         = light.green_start; 
    
    % Calcola la durata del verde (per il primo ciclo e si ripete)
    light.green_duration = light.green_end - light.green_start;
    
    if light.green_duration <= 0
         warning('La durata del verde calcolata è <= 0 per il semaforo a d=%.1f. gs=%.1f, ge=%.1f. Usando fallback.', ...
                 distance, light.green_start, light.green_end);
         light.green_duration = light.cycle_time / 3; % Fallback a 1/3 del ciclo
         % Aggiusta green_end se la durata era negativa/nulla
         light.green_end = light.green_start + light.green_duration;
    end
    % Il campo 'offset_global' non sembra essere usato dal resto del codice,
    % quindi lo ometto per ora per seguire la struttura implicita.
    % Se la convenzione fosse:
    %   light.offset_ciclo_globale
    %   light.green_start_relativo_al_ciclo
    %   light.green_end_relativo_al_ciclo
    % allora la logica in 'is_green', 'next_green', 'prev_green' e nella creazione dei nodi cambierebbe.
    % Manteniamo la coerenza con l'uso esistente.
end


function [t_min_out, t_max_out] = velocity_pruning(traffic_lights_vp, tf_vp, final_dist_vp, v_min_vp, v_max_vp) % Rinomina variabili
    n_tl = length(traffic_lights_vp); % Rinomina n
    if n_tl == 0
        t_min_out = []; t_max_out = []; % Rinomina
        return;
    end
    d_tl = [traffic_lights_vp.distance]; % Rinomina d
    t_min_out = zeros(1,n_tl); 
    t_max_out = zeros(1,n_tl);

    % Forward pass
    if n_tl > 0
        t_min_out(1) = d_tl(1)/v_max_vp; 
        t_max_out(1) = d_tl(1)/v_min_vp; 
        t_max_out(1) = min(t_max_out(1), tf_vp - (final_dist_vp - d_tl(1))/v_max_vp);

        t_min_out(1) = next_green(traffic_lights_vp(1), t_min_out(1));
        t_max_out(1) = prev_green(traffic_lights_vp(1), t_max_out(1));
        if t_max_out(1) < t_min_out(1) 
            % fprintf('[Pruning Warn] Invalid window for TL1: t_min=%.1f, t_max=%.1f. Adjusting.\n', t_min_out(1), t_max_out(1));
            t_min_out(1) = next_green(traffic_lights_vp(1), t_max_out(1) + 0.1); 
            t_max_out(1) = t_min_out(1) + get_green_duration(traffic_lights_vp(1));
            if t_max_out(1) < t_min_out(1), t_max_out(1) = t_min_out(1); end % Ultimo fallback
        end

        for i_tl = 2:n_tl % Rinomina i
            dist_inc_vp = d_tl(i_tl) - d_tl(i_tl-1); % Rinomina
            t_min_out(i_tl) = t_min_out(i_tl-1) + dist_inc_vp/v_max_vp;
            t_max_out(i_tl) = t_max_out(i_tl-1) + dist_inc_vp/v_min_vp;
            t_max_out(i_tl) = min(t_max_out(i_tl), tf_vp - (final_dist_vp - d_tl(i_tl))/v_max_vp);
            
            t_min_out(i_tl) = next_green(traffic_lights_vp(i_tl), t_min_out(i_tl));
            t_max_out(i_tl) = prev_green(traffic_lights_vp(i_tl), t_max_out(i_tl));
            
            if t_max_out(i_tl) < t_min_out(i_tl)
                % fprintf('[Pruning Warn] Invalid window for TL%d: t_min=%.1f, t_max=%.1f. Adjusting.\n', i_tl, t_min_out(i_tl), t_max_out(i_tl));
                t_min_out(i_tl) = next_green(traffic_lights_vp(i_tl), t_max_out(i_tl) + 0.1); 
                t_max_out(i_tl) = t_min_out(i_tl) + get_green_duration(traffic_lights_vp(i_tl));
                 if t_max_out(i_tl) < t_min_out(i_tl), t_max_out(i_tl) = t_min_out(i_tl); end
            end
        end
    end

    % Backward pass
    for i_tl_bp = n_tl-1:-1:1 % Rinomina i
        dist_inc_vp_bp = d_tl(i_tl_bp+1) - d_tl(i_tl_bp); % Rinomina
        potential_t_max_i_bp = t_max_out(i_tl_bp+1) - dist_inc_vp_bp/v_max_vp; % Rinomina
        t_max_out(i_tl_bp) = min(t_max_out(i_tl_bp), potential_t_max_i_bp);
        
        t_max_out(i_tl_bp) = prev_green(traffic_lights_vp(i_tl_bp), t_max_out(i_tl_bp)); 
        if t_max_out(i_tl_bp) < t_min_out(i_tl_bp)
            t_max_out(i_tl_bp) = t_min_out(i_tl_bp); 
        end
    end
end

function green_duration_val = get_green_duration(light_struct_gd) % Rinomina light_struct
    % Usa i campi come definiti in create_traffic_light
    gs_gd = light_struct_gd.green_start; % Inizio primo verde assoluto
    ge_gd = light_struct_gd.green_end;   % Fine primo verde assoluto
    % cycle_gd = light_struct_gd.cycle_time; % Non necessario se gs e ge sono per il primo ciclo
    
    green_duration_val = ge_gd - gs_gd; % Durata del verde
    
    if green_duration_val <= 0 
        % fprintf('[WARN] Invalid green duration (%.2f) for light at %.0f m. gs=%.1f, ge=%.1f. Using fallback.\n', green_duration_val, light_struct_gd.distance,gs_gd,ge_gd);
        if isfield(light_struct_gd, 'cycle_time') && light_struct_gd.cycle_time > 0
            green_duration_val = light_struct_gd.cycle_time/3; % Fallback a 1/3 del ciclo se disponibile
        else
            green_duration_val = 10; % Fallback a 10s se ciclo non disponibile
        end
    end
end


function [path_ids_out, cost_out] = dijkstra(Nodes_dij, Edges_dij, source_node_id_dij, target_node_id_dij) % Rinomina variabili
    if isempty(Nodes_dij)
        path_ids_out = []; cost_out = inf;
        return;
    end
    num_nodes_dij = length(Nodes_dij); % Rinomina nNodes
    
    % Crea una mappa da ID nodo a indice array per efficienza, se gli ID non sono sequenziali 1..N
    node_id_to_idx_map = containers.Map('KeyType','double','ValueType','double');
    for k_node=1:num_nodes_dij
        node_id_to_idx_map(Nodes_dij(k_node).id) = k_node;
    end
    
    if ~isKey(node_id_to_idx_map, source_node_id_dij) || ~isKey(node_id_to_idx_map, target_node_id_dij)
         warning('Dijkstra: Source or Target node ID not found in Nodes list using map.');
         path_ids_out = []; cost_out = inf;
         return;
    end

    source_idx_dij = node_id_to_idx_map(source_node_id_dij); % Rinomina
    target_idx_dij = node_id_to_idx_map(target_node_id_dij); % Rinomina
    
    dist_costs_dij = inf(1, num_nodes_dij); % Rinomina dist_cost
    prev_nodes_dij = nan(1, num_nodes_dij); % Rinomina prev_node
    
    dist_costs_dij(source_idx_dij) = 0;
    
    Q_set_indices = 1:num_nodes_dij; % Rinomina Q_set (lavora con indici array)
    
    while ~isempty(Q_set_indices)
        min_val_q_dij = inf; % Rinomina
        u_idx_dij = -1;      % Rinomina
        
        temp_costs_in_Q = dist_costs_dij(Q_set_indices); % Costi dei soli nodi in Q
        [min_cost_in_Q, rel_idx_in_Q] = min(temp_costs_in_Q);
        
        if isinf(min_cost_in_Q) % Tutti i nodi rimanenti in Q sono irraggiungibili
            break; 
        end
        u_idx_dij = Q_set_indices(rel_idx_in_Q); % Ottieni l'indice assoluto del nodo u
        
        Q_set_indices(Q_set_indices == u_idx_dij) = []; 
        
        if u_idx_dij == target_idx_dij 
            break;
        end
        
        current_node_actual_id = Nodes_dij(u_idx_dij).id; % ID effettivo del nodo u
        
        % Trova archi che partono da current_node_actual_id
        for e_idx_dij = 1:length(Edges_dij) % Rinomina
            if Edges_dij(e_idx_dij).from == current_node_actual_id
                v_node_actual_id = Edges_dij(e_idx_dij).to; % ID effettivo del nodo v
                if ~isKey(node_id_to_idx_map, v_node_actual_id)
                    % warning('Dijkstra: Edge to non-existent node ID %d.', v_node_actual_id);
                    continue; % Salta questo arco se il nodo 'to' non è mappato
                end
                v_idx_dij = node_id_to_idx_map(v_node_actual_id); % Indice array del nodo v
                
                % Controlla se v_idx_dij è ancora in Q_set_indices (non necessario, il check del costo è sufficiente)
                % if ~ismember(v_idx_dij, Q_set_indices)
                %     continue; % Se il nodo v è già stato finalizzato, salta
                % end

                alt_cost_dij = dist_costs_dij(u_idx_dij) + Edges_dij(e_idx_dij).w; % Rinomina
                if alt_cost_dij < dist_costs_dij(v_idx_dij)
                    dist_costs_dij(v_idx_dij) = alt_cost_dij;
                    prev_nodes_dij(v_idx_dij) = u_idx_dij; 
                end
            end
        end
    end
    
    path_indices_dij = []; % Rinomina
    curr_idx_dij = target_idx_dij; % Rinomina
    if ~isinf(dist_costs_dij(target_idx_dij)) 
        while ~isnan(curr_idx_dij)
            path_indices_dij = [curr_idx_dij, path_indices_dij]; 
            if curr_idx_dij == source_idx_dij
                break; 
            end
            prev_val = prev_nodes_dij(curr_idx_dij);
            if isnan(prev_val) && curr_idx_dij ~= source_idx_dij % Percorso interrotto prima di raggiungere la sorgente
                path_indices_dij = []; 
                break;
            end
            curr_idx_dij = prev_val;
        end
    end
    
    if ~isempty(path_indices_dij)
        path_ids_out = [Nodes_dij(path_indices_dij).id]; % Converte indici in ID originali
        cost_out = dist_costs_dij(target_idx_dij);
    else
        path_ids_out = [];
        cost_out = inf;
    end
end


function plot_delta_velocities()
    global SIM_RUNS
    if isempty(SIM_RUNS) || ~isfield(SIM_RUNS{1}, 'x') || isempty(SIM_RUNS{1}.x)
        disp('No data available for the delta speed plot.');
        return;
    end
    
    runData_dv = SIM_RUNS{1}; % Rinomina runData
    t_sim_dv = runData_dv.t;  % Rinomina t_sim
    
    if isempty(t_sim_dv) || length(t_sim_dv) < 2
         disp('Not enough time points in SIM_RUNS{1} for delta speed plot.');
        return;
    end

    n_vehicles_dv = size(runData_dv.x, 2) / 2; % Rinomina n_vehicles
    
    if ~isfield(runData_dv, 'opt_t') || ~isfield(runData_dv, 'opt_d') || ...
       isempty(runData_dv.opt_t) || isempty(runData_dv.opt_d) || length(runData_dv.opt_t) < 2
        disp('No optimal trajectory data in SIM_RUNS{1} for delta speed plot.');
        return;
    end
    opt_t_leader_dv = runData_dv.opt_t; % Rinomina
    opt_d_leader_dv = runData_dv.opt_d; % Rinomina
    
    if length(opt_t_leader_dv) < 2
        % fprintf('[WARN] Optimal trajectory for leader has < 2 points. Cannot interpolate for delta_v plot.\n');
        v_opt_leader_interp_dv = zeros(size(t_sim_dv)); 
    else
        pos_opt_leader_interp_dv = interp1(opt_t_leader_dv, opt_d_leader_dv, t_sim_dv, 'linear', 'extrap'); % Rinomina
        v_opt_leader_interp_dv = gradient(pos_opt_leader_interp_dv, t_sim_dv); % Rinomina
        v_opt_leader_interp_dv = max(0, v_opt_leader_interp_dv); 
    end

    figure_name_dv = 'Delta Speed Calculation (vs Optimal Trajectory)';
    existing_fig_dv = findobj('Type', 'Figure', 'Name', figure_name_dv);
    if ~isempty(existing_fig_dv), clf(existing_fig_dv); figure(existing_fig_dv);
    else, figure('Name',figure_name_dv, 'Position', [100, 100, 1200, max(400, 200*n_vehicles_dv)]);
    end
    
    for v_id_dv = 1:n_vehicles_dv % Rinomina v
        v_sim_veh_dv = runData_dv.x(:, n_vehicles_dv + v_id_dv); % Rinomina
        
        v_opt_veh_interp_dv = zeros(size(t_sim_dv)); % Rinomina

        if v_id_dv == runData_dv.leader
            v_opt_veh_interp_dv = v_opt_leader_interp_dv;
        else 
            vehicles_plt1_dv = get_platoon_vehicles(1); % Rinomina
            v_targets_l_dv = calculate_target_velocities(opt_t_leader_dv, opt_d_leader_dv); % Rinomina
            
            if ismember(v_id_dv, vehicles_plt1_dv) && ~isempty(v_targets_l_dv) && length(opt_t_leader_dv)>=2
                opt_d_foll_dv = calculate_follower_trajectory(v_id_dv, runData_dv.leader, opt_t_leader_dv, opt_d_leader_dv, v_targets_l_dv, vehicles_plt1_dv); % Rinomina
                if ~isempty(opt_d_foll_dv)
                    pos_opt_foll_interp_dv = interp1(opt_t_leader_dv, opt_d_foll_dv, t_sim_dv, 'linear', 'extrap'); % Rinomina
                    v_opt_veh_interp_dv = gradient(pos_opt_foll_interp_dv, t_sim_dv);
                    v_opt_veh_interp_dv = max(0, v_opt_veh_interp_dv); 
                end
            end
        end
        
        diff_v_val = v_opt_veh_interp_dv - v_sim_veh_dv; % Rinomina diff_v
        
        subplot(n_vehicles_dv, 1, v_id_dv);
        hold on; grid on;
        plot(t_sim_dv, v_sim_veh_dv, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulated Speed');
        plot(t_sim_dv, v_opt_veh_interp_dv, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Optimal Speed (Interp.)');
        plot(t_sim_dv, diff_v_val, 'r-.', 'LineWidth', 1.5, 'DisplayName', 'Delta (v_{opt} - v_{sim})');
        
        xlabel('Time [s]');
        ylabel('Speed [m/s]');
        title(sprintf('Vehicle %d: Speeds and Delta (Run 1 Reference)', v_id_dv));
        legend('Location', 'best');
        ylim_curr_dv = get(gca,'YLim'); % Rinomina
        if ~all(ylim_curr_dv == 0) % Evita errore se limiti sono [0 0]
            max_abs_y_dv = max(abs(ylim_curr_dv)); % Rinomina
            ylim([-max_abs_y_dv, max_abs_y_dv]); 
        end
    end
    if n_vehicles_dv > 0 % Solo se ci sono subplot
        sgtitle('Vehicle Speeds: Simulated vs. Optimal, and Their Difference (Run 1 Base Optimal Trajectory)', 'FontWeight','bold');
    end
end

function trigger_events_out = velocity_trigger(t_vt, diff_v_vt, opt_v_vt) % Rinomina variabili
    trigger_thresh_vt = 5.0;    
    disable_dur_vt  = 10.0;   
    rapid_change_vt = 2.0; 
    initial_delay_vt = 5.0;      
    
    trigger_events_out = zeros(size(t_vt)); % Rinomina
    disable_until_t_vt = -inf; % Rinomina
    
    if isempty(t_vt) || length(t_vt) < 2, return; end

    for i_vt = 1:length(t_vt) % Rinomina i
        if t_vt(i_vt) < initial_delay_vt || t_vt(i_vt) < disable_until_t_vt
            trigger_events_out(i_vt) = 0; 
        else
            if abs(diff_v_vt(i_vt)) > trigger_thresh_vt
                trigger_events_out(i_vt) = 1; 
            else
                trigger_events_out(i_vt) = 0; 
            end
        end
        
        if i_vt > 1 
            delta_t_step_vt = t_vt(i_vt) - t_vt(i_vt-1); % Rinomina
            if delta_t_step_vt > 1e-4 % Aumentata tolleranza
                rate_change_opt_v_vt = abs(opt_v_vt(i_vt) - opt_v_vt(i_vt-1)) / delta_t_step_vt; % Rinomina
                if rate_change_opt_v_vt > rapid_change_vt 
                    disable_until_t_vt = t_vt(i_vt) + disable_dur_vt;
                end
            end
        end
    end
end


function plot_velocity_trigger_per_vehicle()
    global SIM_RUNS
    if isempty(SIM_RUNS) || ~isfield(SIM_RUNS{1},'x') || isempty(SIM_RUNS{1}.x)
        disp('No data to plot for velocity triggers.');
        return;
    end
    
    split_info_list = []; % Rinomina split_info
    for r_spi = 1:length(SIM_RUNS) % Rinomina r
        if isfield(SIM_RUNS{r_spi}, 'splittedVehicles') && ~isempty(SIM_RUNS{r_spi}.splittedVehicles)
            if r_spi < length(SIM_RUNS) 
                next_run_leader_spi = SIM_RUNS{r_spi+1}.leader; % Rinomina
                split_time_abs_spi = SIM_RUNS{r_spi+1}.offset; % Rinomina
                
                current_split_data.time = split_time_abs_spi; % Rinomina
                current_split_data.vehicle = next_run_leader_spi; 
                current_split_data.platoon_num_new = r_spi+1; 
                
                if ismember(next_run_leader_spi, SIM_RUNS{r_spi}.splittedVehicles)
                     if isempty(split_info_list)
                        split_info_list = current_split_data;
                    else
                        is_dup_spi = false; % Rinomina
                        for si_check = 1:length(split_info_list) % Rinomina si
                            if split_info_list(si_check).time == current_split_data.time && split_info_list(si_check).vehicle == current_split_data.vehicle
                                is_dup_spi = true; break;
                            end
                        end
                        if ~is_dup_spi
                            split_info_list(end+1) = current_split_data;
                        end
                    end
                end
            end
        end
    end
    
    runData_ref_vt = SIM_RUNS{1}; % Rinomina
    t_sim_ref_vt = runData_ref_vt.t; % Rinomina

    if isempty(t_sim_ref_vt) || length(t_sim_ref_vt) < 2, disp('Not enough sim timepoints for trigger plot.'); return; end

    n_vehicles_vt = size(runData_ref_vt.x, 2) / 2; % Rinomina
    
    if ~isfield(runData_ref_vt, 'opt_t') || ~isfield(runData_ref_vt, 'opt_d') || ...
       isempty(runData_ref_vt.opt_t) || length(runData_ref_vt.opt_t) < 2
        disp('No optimal trajectory in Run 1 for trigger plot.');
        return;
    end
    opt_t_l_ref_vt = runData_ref_vt.opt_t; % Rinomina
    opt_d_l_ref_vt = runData_ref_vt.opt_d; % Rinomina
    
    pos_opt_l_ref_interp_vt = interp1(opt_t_l_ref_vt, opt_d_l_ref_vt, t_sim_ref_vt, 'linear', 'extrap'); % Rinomina
    v_opt_l_ref_interp_vt = gradient(pos_opt_l_ref_interp_vt, t_sim_ref_vt); % Rinomina
    v_opt_l_ref_interp_vt = max(0, v_opt_l_ref_interp_vt);

    figure_name_vt = 'Velocity Trigger Analysis and Platoon Splits';
    existing_fig_vt = findobj('Type','Figure','Name',figure_name_vt);
    if ~isempty(existing_fig_vt), clf(existing_fig_vt); figure(existing_fig_vt);
    else, figure('Name', figure_name_vt, 'Position', [150, 100, 1200, max(400, 200*n_vehicles_vt)]);
    end

    
    for v_id_vt = 1:n_vehicles_vt % Rinomina v
        v_sim_veh_ref_vt = runData_ref_vt.x(:, n_vehicles_vt + v_id_vt); % Rinomina
        
        v_opt_veh_ref_interp_vt = zeros(size(t_sim_ref_vt)); % Rinomina
        if v_id_vt == runData_ref_vt.leader
            v_opt_veh_ref_interp_vt = v_opt_l_ref_interp_vt;
        else 
            vehicles_r1_vt = get_platoon_vehicles(1); % Rinomina
            v_targets_r1_vt = calculate_target_velocities(opt_t_l_ref_vt, opt_d_l_ref_vt); % Rinomina
            if ismember(v_id_vt, vehicles_r1_vt) && ~isempty(v_targets_r1_vt) && length(opt_t_l_ref_vt)>=2
                opt_d_foll_ref_vt = calculate_follower_trajectory(v_id_vt, runData_ref_vt.leader, opt_t_l_ref_vt, opt_d_l_ref_vt, v_targets_r1_vt, vehicles_r1_vt); % Rinomina
                if ~isempty(opt_d_foll_ref_vt)
                    pos_opt_foll_ref_interp_vt = interp1(opt_t_l_ref_vt, opt_d_foll_ref_vt, t_sim_ref_vt, 'linear', 'extrap'); % Rinomina
                    v_opt_veh_ref_interp_vt = gradient(pos_opt_foll_ref_interp_vt, t_sim_ref_vt);
                    v_opt_veh_ref_interp_vt = max(0, v_opt_veh_ref_interp_vt);
                end
            end
        end
        
        diff_v_ref_vt = v_opt_veh_ref_interp_vt - v_sim_veh_ref_vt; % Rinomina
        trigger_state_ref_vt = velocity_trigger(t_sim_ref_vt, diff_v_ref_vt, v_opt_veh_ref_interp_vt); % Rinomina
        
        if n_vehicles_vt > 0, subplot(n_vehicles_vt, 1, v_id_vt); end
        hold on; grid on;
        
        h_sim_vt = plot(t_sim_ref_vt, v_sim_veh_ref_vt, 'b-', 'LineWidth', 1.5); % Rinomina
        h_opt_vt = plot(t_sim_ref_vt, v_opt_veh_ref_interp_vt, 'g--', 'LineWidth', 1.5); % Rinomina
        h_diff_vt = plot(t_sim_ref_vt, diff_v_ref_vt, 'r-.', 'LineWidth', 1.0); % Rinomina
        
        max_abs_diff_vt = max(abs(diff_v_ref_vt)); if max_abs_diff_vt==0, max_abs_diff_vt=1; end % Rinomina
        h_trigger_vt = plot(t_sim_ref_vt, trigger_state_ref_vt * max_abs_diff_vt * 0.8, 'm-', 'LineWidth', 2); % Rinomina
        
        legend_h_sub_vt = [h_sim_vt, h_opt_vt, h_diff_vt, h_trigger_vt]; % Rinomina
        legend_t_sub_vt = {'Sim. Speed (Run 1)', 'Opt. Speed (Run 1)', 'Delta (v_{opt}-v_{sim})', 'Trigger State (Run 1 based)'}; % Rinomina
        
        y_lim_sub_vt = get(gca, 'YLim'); max_y_val_vt = max(abs(y_lim_sub_vt)); % Rinomina
        
        for si_plot = 1:length(split_info_list) % Rinomina si
            if split_info_list(si_plot).vehicle == v_id_vt 
                h_split_line_vt = plot([split_info_list(si_plot).time, split_info_list(si_plot).time], [-max_y_val_vt, max_y_val_vt], ...
                                    'k:', 'LineWidth', 2.5); % Rinomina e stile
                text(split_info_list(si_plot).time + 1, max_y_val_vt * 0.9, ...
                     sprintf('Split: V%d leads Plt %d', v_id_vt, split_info_list(si_plot).platoon_num_new), ...
                     'Color', 'black', 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.7], 'EdgeColor','k'); % Sfondo semi-trasparente
                
                if ~any(strcmp(legend_t_sub_vt, 'Platoon Split Event'))
                    legend_h_sub_vt(end+1) = h_split_line_vt;
                    legend_t_sub_vt{end+1} = 'Platoon Split Event';
                end
            end
        end
        
        legend(legend_h_sub_vt, legend_t_sub_vt, 'Location', 'best', 'FontSize', 8);
        xlabel('Time [s]'); ylabel('Speed [m/s]');
        title(sprintf('Vehicle %d: Velocity Trigger Analysis (Ref: Run 1 Optimal)', v_id_vt));
        if ~all(y_lim_sub_vt == 0) 
            ylim([-max_y_val_vt*1.1, max_y_val_vt*1.1]); % Aggiungi margine
        end
    end
    if n_vehicles_vt > 0
        sgtitle('Velocity Trigger Analysis and Indication of Platoon Splits', 'FontWeight','bold');
    end
end


function check_velocity_triggers(t_abs_curr, x_sim_curr, ~) % Rinomina
    persistent last_trigger_proc_time_cvt; 
    if isempty(last_trigger_proc_time_cvt), last_trigger_proc_time_cvt = -inf; end

    global SIM_RUNS % N_PLATOON non è usato qui direttamente, ma dalla funzione chiamata
    
    curr_run_idx_cvt = length(SIM_RUNS); % Rinomina
    curr_run_data_cvt = SIM_RUNS{curr_run_idx_cvt}; % Rinomina
    
    if isempty(t_abs_curr) || length(t_abs_curr) < 2, return; end

    n_veh_total_cvt = size(x_sim_curr, 2) / 2; % Rinomina
    
    inhibit_period_cvt = 15.0; % Rinomina
    if t_abs_curr(1) < last_trigger_proc_time_cvt + inhibit_period_cvt || ...
       (curr_run_data_cvt.offset > 0 && t_abs_curr(1) < curr_run_data_cvt.offset + inhibit_period_cvt/2) 
        return;
    end

    if ~isfield(curr_run_data_cvt, 'opt_t') || ~isfield(curr_run_data_cvt, 'opt_d') || ...
       isempty(curr_run_data_cvt.opt_t) || length(curr_run_data_cvt.opt_t) < 2
        return;
    end
    opt_t_l_curr_cvt = curr_run_data_cvt.opt_t; % Rinomina
    opt_d_l_curr_cvt = curr_run_data_cvt.opt_d; % Rinomina
    
    pos_opt_l_curr_interp_cvt = interp1(opt_t_l_curr_cvt, opt_d_l_curr_cvt, t_abs_curr, 'linear', 'extrap'); % Rinomina
    v_opt_l_curr_interp_cvt = gradient(pos_opt_l_curr_interp_cvt, t_abs_curr); % Rinomina
    v_opt_l_curr_interp_cvt = max(0, v_opt_l_curr_interp_cvt);

    vehicles_curr_plt_cvt = get_platoon_vehicles(curr_run_idx_cvt); % Rinomina

    for v_list_idx_cvt = 1:length(vehicles_curr_plt_cvt) % Rinomina
        v_id_cvt = vehicles_curr_plt_cvt(v_list_idx_cvt); % Rinomina
        if v_id_cvt == curr_run_data_cvt.leader, continue; end 
        if v_id_cvt < 1 || v_id_cvt > n_veh_total_cvt, continue; end 

        v_sim_veh_cvt = x_sim_curr(:, n_veh_total_cvt + v_id_cvt); % Rinomina
        
        v_opt_veh_curr_interp_cvt = zeros(size(t_abs_curr)); % Rinomina
        v_targets_l_curr_cvt = calculate_target_velocities(opt_t_l_curr_cvt, opt_d_l_curr_cvt); % Rinomina

        if ~isempty(v_targets_l_curr_cvt) && length(opt_t_l_curr_cvt) >=2
            opt_d_foll_curr_cvt = calculate_follower_trajectory(v_id_cvt, curr_run_data_cvt.leader, ...
                opt_t_l_curr_cvt, opt_d_l_curr_cvt, v_targets_l_curr_cvt, vehicles_curr_plt_cvt); % Rinomina
            if ~isempty(opt_d_foll_curr_cvt)
                pos_opt_foll_curr_interp_cvt = interp1(opt_t_l_curr_cvt, opt_d_foll_curr_cvt, t_abs_curr, 'linear', 'extrap'); % Rinomina
                v_opt_veh_curr_interp_cvt = gradient(pos_opt_foll_curr_interp_cvt, t_abs_curr);
                v_opt_veh_curr_interp_cvt = max(0, v_opt_veh_curr_interp_cvt);
            end
        end
        
        diff_v_veh_cvt = v_opt_veh_curr_interp_cvt - v_sim_veh_cvt; % Rinomina
        trigger_pts_cvt = velocity_trigger(t_abs_curr, diff_v_veh_cvt, v_opt_veh_curr_interp_cvt); % Rinomina
        
        first_trigger_sim_idx_cvt = find(trigger_pts_cvt, 1, 'first'); % Rinomina
        
        if ~isempty(first_trigger_sim_idx_cvt)
            trigger_time_abs_cvt = t_abs_curr(first_trigger_sim_idx_cvt); % Rinomina
            
            if trigger_time_abs_cvt < last_trigger_proc_time_cvt + inhibit_period_cvt
                continue; 
            end

            curr_pos_trigger_cvt = x_sim_curr(first_trigger_sim_idx_cvt, v_id_cvt); % Rinomina
            fprintf('\n[VELOCITY TRIGGER] Vehicle %d (Follower in Plt %d) triggered at t=%.2f s, pos=%.2f m.\n', ...
                    v_id_cvt, curr_run_idx_cvt, trigger_time_abs_cvt, curr_pos_trigger_cvt);
            
            idx_v_in_plt_cvt = find(vehicles_curr_plt_cvt == v_id_cvt); % Rinomina
            if isempty(idx_v_in_plt_cvt) , warning('CVT: Triggering vehicle not in its own platoon list.'); return; end
            
            vehicles_for_new_plt_cvt = vehicles_curr_plt_cvt(idx_v_in_plt_cvt:end); % Rinomina
            
            SIM_RUNS{curr_run_idx_cvt}.splittedVehicles = vehicles_for_new_plt_cvt;
            last_trigger_proc_time_cvt = trigger_time_abs_cvt;

            % Usa la funzione wrapper che gestisce N_PLATOON
            rerun_optimizer_for_velocity_trigger_EVENT(v_id_cvt, trigger_time_abs_cvt, curr_pos_trigger_cvt); % Rinominata per chiarezza
            
            return; 
        end
    end
end


%% MODIFICA NOTA: Rinominata la funzione wrapper
function rerun_optimizer_for_velocity_trigger_EVENT(trigger_vehicle_id_event, absolute_trigger_time_event, stop_pos_est_event) % Rinomina variabili
    global N_PLATOON SIM_RUNS % SIM_RUNS per check duplicati

    % Logica prevenzione duplicati (opzionale)
    % for k_run_check_vt = 1:length(SIM_RUNS)
    %     if SIM_RUNS{k_run_check_vt}.leader == trigger_vehicle_id_event && abs(SIM_RUNS{k_run_check_vt}.offset - absolute_trigger_time_event) < 1.5 % Tolleranza
    %         fprintf('[INFO] Velocity trigger split for V%d at t=%.1f already being processed. Skipping.\n', trigger_vehicle_id_event, absolute_trigger_time_event);
    %         return;
    %     end
    % end
    
    N_PLATOON = N_PLATOON + 1;
    new_platoon_num_event = N_PLATOON;

    disp(['[EVENT] VELOCITY TRIGGER SPLIT: Creating PLATOON ' num2str(new_platoon_num_event) ' for LEADER=' ...
          num2str(trigger_vehicle_id_event) '. EventTime=~' num2str(absolute_trigger_time_event, '%.1f') 's, EventPos=~' ...
          num2str(stop_pos_est_event, '%.1f') 'm.']);

    % Chiama la funzione di ottimizzazione, che NON deve toccare N_PLATOON.
    run_optimizer_from_traffic_light(trigger_vehicle_id_event, absolute_trigger_time_event, stop_pos_est_event, []); % Passa semaforo vuoto
end


function t_out_ng = next_green(light_ng, t_in_ng) % Rinomina variabili
    first_gs_abs_ng = light_ng.green_start; % Rinomina
    first_ge_abs_ng = light_ng.green_end;   % Rinomina
    cycle_ng        = light_ng.cycle_time;  % Rinomina
    green_dur_ng    = light_ng.green_duration; % Usa il campo precalcolato

    if green_dur_ng <= 0 || cycle_ng <= 0 % Sanity check
        % warning('next_green: Invalid light params (dur=%.1f, cyc=%.1f). Returning t_in.', green_dur_ng, cycle_ng);
        t_out_ng = t_in_ng; return; 
    end

    % Se t_in è già in un periodo verde
    if is_green(light_ng, t_in_ng)
        t_out_ng = t_in_ng;
        return;
    end
    
    % Altrimenti, trova l'inizio del prossimo periodo verde
    % Calcola quanti cicli completi sono passati o servono per superare t_in_ng partendo da first_gs_abs_ng
    if t_in_ng < first_gs_abs_ng % Se t_in è prima del primo verde in assoluto
        k_ng = 0; % Il prossimo verde è quello del primo ciclo (k=0)
    else
        % Tempo relativo all'inizio del primo periodo verde
        time_rel_first_green_ng = t_in_ng - first_gs_abs_ng;
        % Quanti cicli sono trascorsi completamente fino all'inizio del rosso in cui si trova t_in_ng
        k_ng = floor(time_rel_first_green_ng / cycle_ng);
        
        % Controlla se t_in_ng è nel verde del ciclo k_ng o successivo
        start_green_k_ng = first_gs_abs_ng + k_ng * cycle_ng;
        % end_green_k_ng   = start_green_k_ng + green_dur_ng;
        
        if t_in_ng >= start_green_k_ng + green_dur_ng % Se t_in è dopo la fine del verde del ciclo k_ng
            k_ng = k_ng + 1; % Passa al ciclo successivo
        end
        % Se t_in_ng è nel rosso prima del verde del ciclo k_ng, k_ng è corretto.
    end
    
    t_out_ng = first_gs_abs_ng + k_ng * cycle_ng;
end


function t_out_pg = prev_green(light_pg, t_in_pg) % Rinomina variabili
    first_gs_abs_pg = light_pg.green_start; % Rinomina
    first_ge_abs_pg = light_pg.green_end;   % Rinomina
    cycle_pg        = light_pg.cycle_time;  % Rinomina
    green_dur_pg    = light_pg.green_duration; % Usa campo precalcolato

    if green_dur_pg <= 0 || cycle_pg <= 0
        % warning('prev_green: Invalid light params. Returning t_in.');
        t_out_pg = t_in_pg; return; 
    end

    if is_green(light_pg, t_in_pg) % Se t_in è già verde, quello è il tempo di fine (o t_in stesso)
        t_out_pg = t_in_pg;
        return;
    end

    % Trova la fine dell'ultimo periodo verde <= t_in_pg
    if t_in_pg < first_gs_abs_pg % Se t_in è prima dell'inizio del primo verde
        % Non c'è un verde precedente valido in questo modello semplice.
        % Restituire un valore che causi t_max < t_min è un'opzione.
        t_out_pg = t_in_pg - cycle_pg; % Un valore sicuramente "sbagliato" per forzare pruning
        return;
    end

    time_rel_first_green_pg = t_in_pg - first_gs_abs_pg;
    k_pg = floor(time_rel_first_green_pg / cycle_pg); % Ciclo in cui o poco prima di cui cade t_in_pg

    % Fine del verde del ciclo k_pg
    end_green_k_pg_abs = first_gs_abs_pg + k_pg * cycle_pg + green_dur_pg;
    
    if t_in_pg >= end_green_k_pg_abs % Se t_in è dopo (o uguale a) la fine del verde del ciclo k_pg
        t_out_pg = end_green_k_pg_abs; % La fine di quel verde è il risultato
    else % t_in è nel rosso prima della fine del verde del ciclo k_pg, o prima dell'inizio del verde del ciclo k_pg
         % Significa che il verde rilevante è quello del ciclo k_pg-1
        t_out_pg = first_gs_abs_pg + (k_pg - 1) * cycle_pg + green_dur_pg;
    end
end


function flag_is_green = is_green(light_ig, t_query_ig) % Rinomina variabili
    first_gs_abs_ig = light_ig.green_start; % Rinomina
    % first_ge_abs_ig = light_ig.green_end; % Non serve qui direttamente se usiamo durata
    cycle_time_ig   = light_ig.cycle_time;  % Rinomina
    green_dur_ig    = light_ig.green_duration; % Usa campo precalcolato

    if green_dur_ig <= 0 || cycle_time_ig <= 0
        flag_is_green = false; 
        return;
    end

    time_rel_first_green_ig = t_query_ig - first_gs_abs_ig; % Rinomina
    
    if time_rel_first_green_ig < 0 
        flag_is_green = false;
        return;
    end
    
    phase_in_cycle_ig = mod(time_rel_first_green_ig, cycle_time_ig); % Rinomina
    
    % Il verde inizia a fase 0 del pattern (che è offsettato da first_gs_abs_ig)
    % e dura per green_dur_ig
    if phase_in_cycle_ig < green_dur_ig % Confronto stretto con durata
        flag_is_green = true;
    else
        flag_is_green = false;
    end
end


%%%% Helper functions %%%%

function [max_time_gs, max_distance_gs] = get_global_time_and_distance() % Rinomina output
    global SIM_RUNS
    max_time_gs     = 0;
    max_distance_gs = 0;
    if isempty(SIM_RUNS), return; end
    for idxRun_gs = 1:length(SIM_RUNS) % Rinomina
        runData_gs = SIM_RUNS{idxRun_gs}; % Rinomina
        if isfield(runData_gs,'t') && ~isempty(runData_gs.t)
            max_time_gs = max(max_time_gs, max(runData_gs.t(:))); 
        end
        if isfield(runData_gs,'x') && ~isempty(runData_gs.x)
            num_veh_gs = size(runData_gs.x,2)/2; % Rinomina
            if num_veh_gs > 0
                pos_matrix_gs = runData_gs.x(:,1:num_veh_gs); % Rinomina
                max_val_pos = max(pos_matrix_gs(:),[],'omitnan');
                if ~isempty(max_val_pos) % Assicura che non sia vuoto prima di max()
                    max_distance_gs = max(max_distance_gs, max_val_pos); 
                end
            end
        end
    end
    if max_time_gs == 0 && isempty(SIM_RUNS) == false, max_time_gs = 150; end % Fallback solo se SIM_RUNS non è vuoto ma t è vuoto
    if max_distance_gs == 0 && isempty(SIM_RUNS) == false, max_distance_gs = 1800; end 
end

function col_map_out = define_vehicle_colors() % Rinomina col_map
    col_map_out = [ 
        0.00, 0.45, 0.74; 
        0.85, 0.33, 0.10; 
        0.47, 0.67, 0.19; 
        0.93, 0.69, 0.13; 
        0.30, 0.75, 0.93; 
        0.64, 0.08, 0.18; 
        0.49, 0.18, 0.56; 
        1.00, 0.50, 0.05; 
        0.00, 0.00, 0.00; 
        0.50, 0.50, 0.50; 
    ];
end


function [all_t_plot, all_d_plot, all_c_plot] = prepare_traffic_light_data(tl_config_list_ptd) % Rinomina
    global SIM_RUNS
    all_t_plot = []; % Rinomina
    all_d_plot = []; % Rinomina
    all_c_plot = []; % Rinomina

    if isempty(tl_config_list_ptd) || isempty(SIM_RUNS)
        return;
    end
    
    max_sim_t_ptd = 0; % Rinomina
    for r_idx_ptd = 1:length(SIM_RUNS) % Rinomina
        if isfield(SIM_RUNS{r_idx_ptd},'t') && ~isempty(SIM_RUNS{r_idx_ptd}.t)
            max_sim_t_ptd = max(max_sim_t_ptd, SIM_RUNS{r_idx_ptd}.t(end));
        end
    end
    if max_sim_t_ptd == 0, max_sim_t_ptd = 150; end

    time_res_plot_ptd = 1.0; % Rinomina
    plot_t_vec_ptd = 0:time_res_plot_ptd:ceil(max_sim_t_ptd); % Rinomina
    
    if isempty(plot_t_vec_ptd), return; end

    num_lights_ptd = length(tl_config_list_ptd); % Rinomina
    num_t_points_ptd = length(plot_t_vec_ptd);   % Rinomina

    % Prealloca per efficienza
    all_t_plot = zeros(num_lights_ptd * num_t_points_ptd, 1);
    all_d_plot = zeros(num_lights_ptd * num_t_points_ptd, 1);
    all_c_plot = zeros(num_lights_ptd * num_t_points_ptd, 3);
    
    curr_idx_ptd = 1; % Rinomina
    for i_l_ptd = 1:num_lights_ptd % Rinomina
        light_i_cfg_ptd = tl_config_list_ptd(i_l_ptd); % Rinomina
        for t_idx_ptd = 1:num_t_points_ptd % Rinomina
            time_point_ptd = plot_t_vec_ptd(t_idx_ptd); % Rinomina
            
            all_t_plot(curr_idx_ptd) = time_point_ptd;
            all_d_plot(curr_idx_ptd) = light_i_cfg_ptd.distance;
            
            if is_green(light_i_cfg_ptd, time_point_ptd)
                all_c_plot(curr_idx_ptd,:) = [0.2, 0.8, 0.2]; 
            else
                all_c_plot(curr_idx_ptd,:) = [0.9, 0.1, 0.1]; 
            end
            curr_idx_ptd = curr_idx_ptd + 1;
        end
    end
end


function platoon_vehicles_out = get_platoon_vehicles(run_idx_gpv) % Rinomina
    global SIM_RUNS
    platoon_vehicles_out = []; % Rinomina
    if run_idx_gpv < 1 || run_idx_gpv > length(SIM_RUNS)
        return;
    end

    current_run_data_gpv = SIM_RUNS{run_idx_gpv}; % Rinomina
    leader_curr_run_gpv = current_run_data_gpv.leader; % Rinomina
    
    % Determina il numero totale di veicoli dalla prima esecuzione (run 1)
    if isempty(SIM_RUNS) || ~isfield(SIM_RUNS{1},'x') || isempty(SIM_RUNS{1}.x)
        total_num_veh_gpv = 0; % Non possiamo determinare il numero totale
         warning('get_platoon_vehicles: SIM_RUNS{1}.x is empty, cannot determine total vehicle count.');
    else
        total_num_veh_gpv = size(SIM_RUNS{1}.x, 2) / 2; % Rinomina
    end


    if run_idx_gpv == 1
        % Per il primo run, tutti i veicoli da 1 a N_total fanno parte del plotone.
        % 'splittedVehicles' in run 1 indica chi si staccherà ALLA FINE di run 1.
        if total_num_veh_gpv > 0
            platoon_vehicles_out = 1:total_num_veh_gpv;
        else
            platoon_vehicles_out = []; % Nessun veicolo se il conteggio è zero
        end
    else 
        prev_run_data_gpv = SIM_RUNS{run_idx_gpv - 1}; % Rinomina
        if isfield(prev_run_data_gpv, 'splittedVehicles') && ~isempty(prev_run_data_gpv.splittedVehicles)
            if ~isempty(prev_run_data_gpv.splittedVehicles) && prev_run_data_gpv.splittedVehicles(1) == leader_curr_run_gpv
                 platoon_vehicles_out = prev_run_data_gpv.splittedVehicles;
            else
                 % warning('get_platoon_vehicles: Leader mismatch for run %d. Expected L%d, got L%d. Using only current leader.', run_idx_gpv, prev_run_data_gpv.splittedVehicles(1), leader_curr_run_gpv);
                 platoon_vehicles_out = leader_curr_run_gpv; % Fallback
            end
        else
             % warning('get_platoon_vehicles: Run %d expected a split from run %d, but splittedVehicles was empty. Using only current leader.', run_idx_gpv, run_idx_gpv-1);
             platoon_vehicles_out = leader_curr_run_gpv; % Fallback
        end
    end
    
    % Assicura che il leader sia nel plotone e che gli ID siano validi
    if ~isempty(platoon_vehicles_out)
        if ~ismember(leader_curr_run_gpv, platoon_vehicles_out)
            platoon_vehicles_out = [leader_curr_run_gpv, platoon_vehicles_out]; % Aggiungi se mancante
            platoon_vehicles_out = unique(platoon_vehicles_out, 'stable'); % Rimuovi duplicati mantenendo ordine
        end
        if total_num_veh_gpv > 0 % Filtra solo se abbiamo un conteggio valido
            platoon_vehicles_out = platoon_vehicles_out(platoon_vehicles_out > 0 & platoon_vehicles_out <= total_num_veh_gpv);
        elseif isempty(platoon_vehicles_out) && leader_curr_run_gpv > 0 % Caso in cui total_num_veh_gpv era 0
            platoon_vehicles_out = leader_curr_run_gpv; % Almeno il leader se total_num_veh non definito
        end

    elseif leader_curr_run_gpv > 0 % Se platoon_vehicles_out era vuoto ma c'è un leader
        platoon_vehicles_out = leader_curr_run_gpv;
    end
end


function plot_real_trajectories()
    global SIM_RUNS
    if isempty(SIM_RUNS), disp('[plot_real_trajectories] No data in SIM_RUNS.'); return; end

    figure_name_prt = 'Real Trajectories of All Vehicles with Traffic Lights';
    existing_fig_prt = findobj('Type', 'Figure', 'Name', figure_name_prt);
    if ~isempty(existing_fig_prt), clf(existing_fig_prt); figure(existing_fig_prt);
    else, figure('Name',figure_name_prt, 'Position',[200,150,1000,650]);
    end
    hold on;  grid on;

    if ~isfield(SIM_RUNS{1}, 'traffic_lights')
        disp('[plot_real_trajectories] No traffic light data in SIM_RUNS{1}.');
        return;
    end
    tl_cfg_prt = SIM_RUNS{1}.traffic_lights; % Rinomina
    [tl_t_prt, tl_d_prt, tl_c_prt] = prepare_traffic_light_data(tl_cfg_prt); % Rinomina
    if ~isempty(tl_t_prt)
        scatter(tl_t_prt, tl_d_prt, 15, tl_c_prt, 'filled', 'Marker','s', 'DisplayName','Traffic Lights');
    end

    veh_colors_prt = define_vehicle_colors(); % Rinomina
    
    if ~isfield(SIM_RUNS{1}, 'x') || isempty(SIM_RUNS{1}.x)
        disp('[plot_real_trajectories] No vehicle state data in SIM_RUNS{1}.x.');
        title('Real Trajectories - ERROR: No vehicle data in Run 1');
        return;
    end
    num_total_veh_prt = size(SIM_RUNS{1}.x, 2) / 2; % Rinomina
    
    legend_h_prt = []; % Rinomina
    legend_t_prt = {}; % Rinomina
    plotted_veh_leg_prt = false(1, num_total_veh_prt); % Rinomina

    for v_id_prt = 1:num_total_veh_prt % Rinomina v
        color_v_prt = veh_colors_prt(mod(v_id_prt-1, size(veh_colors_prt,1))+1, :); % Rinomina
        
        veh_segments_t_prt = {}; % Rinomina
        veh_segments_x_prt = {}; % Rinomina
        
        for run_idx_prt = 1:length(SIM_RUNS) % Rinomina
            run_data_prt = SIM_RUNS{run_idx_prt}; % Rinomina
            if ~isfield(run_data_prt,'t') || isempty(run_data_prt.t) || ~isfield(run_data_prt,'x') || isempty(run_data_prt.x), continue; end

            vehicles_platoon_prt = get_platoon_vehicles(run_idx_prt); % Rinomina
            
            if ismember(v_id_prt, vehicles_platoon_prt) 
                t_run_prt = run_data_prt.t; % Rinomina
                if v_id_prt > size(run_data_prt.x,2)/2 % Verifica se v_id_prt è un indice valido per x
                     % warning('Vehicle ID %d out of bounds for run %d states.', v_id_prt, run_idx_prt);
                     continue;
                end
                x_run_v_prt = run_data_prt.x(:, v_id_prt); % Rinomina
                
                t_start_plot_prt = run_data_prt.offset; % Rinomina
                t_end_plot_prt = t_run_prt(end);       % Rinomina
                
                if isfield(run_data_prt, 'splittedVehicles') && ~isempty(run_data_prt.splittedVehicles) && ismember(v_id_prt, run_data_prt.splittedVehicles)
                    if run_idx_prt < length(SIM_RUNS) 
                        next_run_data_prt = SIM_RUNS{run_idx_prt+1}; % Rinomina
                        % Se questo veicolo è il leader del prossimo plotone (o parte del gruppo che si stacca)
                        if ~isempty(run_data_prt.splittedVehicles) && next_run_data_prt.leader == run_data_prt.splittedVehicles(1)
                             t_end_plot_prt = min(t_end_plot_prt, next_run_data_prt.offset);
                        end
                    end
                end
                
                mask_time_plot_prt = (t_run_prt >= t_start_plot_prt - 1e-3) & (t_run_prt <= t_end_plot_prt + 1e-3); % Rinomina, aggiungi tolleranza
                if sum(mask_time_plot_prt) < 2 && ~(sum(mask_time_plot_prt)==1 && t_start_plot_prt == t_end_plot_prt), continue; end 
                
                veh_segments_t_prt{end+1} = t_run_prt(mask_time_plot_prt);
                veh_segments_x_prt{end+1} = x_run_v_prt(mask_time_plot_prt);
            end
        end
        
        for seg_idx_prt = 1:length(veh_segments_t_prt) % Rinomina
            h_p_prt = plot(veh_segments_t_prt{seg_idx_prt}, veh_segments_x_prt{seg_idx_prt}, '-', 'Color', color_v_prt, 'LineWidth', 2); % Rinomina
            if ~plotted_veh_leg_prt(v_id_prt) 
                set(h_p_prt, 'DisplayName', sprintf('Vehicle %d', v_id_prt)); % Aggiungi DisplayName per legenda automatica
                % legend_h_prt(end+1) = h_p_prt; % Non necessario se si usa DisplayName
                % legend_t_prt{end+1} = sprintf('Vehicle %d', v_id_prt);
                plotted_veh_leg_prt(v_id_prt) = true;
            else % Per segmenti successivi dello stesso veicolo, non aggiungere alla legenda
                set(h_p_prt, 'HandleVisibility', 'off');
            end
        end
    end

    % if ~isempty(legend_h_prt) % Ora la legenda è gestita da DisplayName
    %     legend(legend_h_prt, legend_t_prt, 'Location','BestOutside', 'NumColumns',1);
    % end
    legend('show', 'Location','BestOutside', 'NumColumns',1); % Mostra legenda basata su DisplayName
    
    max_sim_t_overall_prt = get_global_time_and_distance(); % Rinomina
    if max_sim_t_overall_prt == 0 && ~isempty(SIM_RUNS) && isfield(SIM_RUNS{1},'t') && ~isempty(SIM_RUNS{1}.t)
        max_sim_t_overall_prt = SIM_RUNS{1}.t(end); % Fallback se get_global_time_and_distance fallisce ma ci sono dati
    elseif max_sim_t_overall_prt == 0
        max_sim_t_overall_prt = 150; % Ultimo fallback
    end
    xlim([0, max_sim_t_overall_prt]);
    xlabel('Time [s]');   ylabel('Position [m]');
    title('Real Vehicle Trajectories (All Segments) and Traffic Lights');
end



function plot_optimal_trajectories_and_lights()
    global SIM_RUNS
    if isempty(SIM_RUNS),  disp('[plot_optimal_trajectories_and_lights] No data.');  return; end

    figure_name_potl = 'Optimal Trajectories (All Platoons) and Traffic Lights'; % Rinomina
    existing_fig_potl = findobj('Type', 'Figure', 'Name', figure_name_potl);
    if ~isempty(existing_fig_potl), clf(existing_fig_potl); figure(existing_fig_potl);
    else, figure('Name',figure_name_potl, 'Position',[100,100,1000,650]);
    end
    hold on; grid on;

    if ~isfield(SIM_RUNS{1}, 'traffic_lights')
        disp('[plot_optimal_trajectories_and_lights] No traffic light data in SIM_RUNS{1}.');
        return;
    end
    tl_cfg_potl = SIM_RUNS{1}.traffic_lights; % Rinomina
    [tl_t_potl, tl_d_potl, tl_c_potl] = prepare_traffic_light_data(tl_cfg_potl); % Rinomina
    if ~isempty(tl_t_potl)
        scatter(tl_t_potl, tl_d_potl, 15, tl_c_potl, 'filled', 'Marker','s', 'DisplayName','Traffic Lights');
    end

    veh_colors_potl = define_vehicle_colors(); % Rinomina
    line_styles_run_potl = {'--', ':', '-.','--','-.',':'}; % Rinomina
    
    legend_h_potl = []; % Rinomina
    legend_t_potl = {}; % Rinomina
    
    max_opt_t_potl = 0; max_opt_d_potl = 0; % Rinomina

    for run_idx_potl = 1:length(SIM_RUNS) % Rinomina
        runData_potl = SIM_RUNS{run_idx_potl}; % Rinomina
        if ~isfield(runData_potl,'opt_t') || ~isfield(runData_potl,'opt_d') || isempty(runData_potl.opt_t)
            continue; 
        end
        
        opt_t_run_val = runData_potl.opt_t;  % Rinomina
        opt_d_leader_run_val = runData_potl.opt_d;   % Rinomina
        leader_id_run_val = runData_potl.leader; % Rinomina

        max_opt_t_potl = max(max_opt_t_potl, max(opt_t_run_val(:)));
        max_opt_d_potl = max(max_opt_d_potl, max(opt_d_leader_run_val(:)));

        vehicles_platoon_run_potl = get_platoon_vehicles(run_idx_potl); % Rinomina
        if isempty(vehicles_platoon_run_potl), continue; end
        
        v_targets_run_potl = calculate_target_velocities(opt_t_run_val, opt_d_leader_run_val); % Rinomina

        line_style_run_potl = line_styles_run_potl{mod(run_idx_potl-1, length(line_styles_run_potl))+1}; % Rinomina

        color_l_opt_potl = veh_colors_potl(mod(leader_id_run_val-1, size(veh_colors_potl,1))+1, :); % Rinomina
        h_plot_l_potl = plot(opt_t_run_val, opt_d_leader_run_val, line_style_run_potl, 'Color', color_l_opt_potl, 'LineWidth', 2.0); % Rinomina
        
        % Legenda per leader (una volta per leader ID)
        is_leader_in_legend = false;
        for k_leg_check = 1:length(legend_t_potl)
            if startsWith(legend_t_potl{k_leg_check}, sprintf('L%d Opt.', leader_id_run_val))
                is_leader_in_legend = true; break;
            end
        end
        if ~is_leader_in_legend
             set(h_plot_l_potl, 'DisplayName', sprintf('L%d Opt. (Style for Plt %d)', leader_id_run_val, run_idx_potl));
             % legend_h_potl(end+1) = h_plot_l_potl; % Non necessario con DisplayName
             % legend_t_potl{end+1} = sprintf('L%d Opt. (Style for Plt %d)', leader_id_run_val, run_idx_potl);
        else
             set(h_plot_l_potl, 'HandleVisibility','off');
        end

        for v_idx_potl = 1:length(vehicles_platoon_run_potl) % Rinomina
            v_id_potl = vehicles_platoon_run_potl(v_idx_potl); % Rinomina
            if v_id_potl == leader_id_run_val, continue; end 
            
            color_foll_opt_potl = veh_colors_potl(mod(v_id_potl-1, size(veh_colors_potl,1))+1, :); % Rinomina
            opt_d_foll_run_val = calculate_follower_trajectory(v_id_potl, leader_id_run_val, opt_t_run_val, opt_d_leader_run_val, v_targets_run_potl, vehicles_platoon_run_potl); % Rinomina
            
            if ~isempty(opt_d_foll_run_val)
                h_plot_f_potl = plot(opt_t_run_val, opt_d_foll_run_val, line_style_run_potl, 'Color', color_foll_opt_potl, 'LineWidth', 1.5); % Rinomina
                max_opt_d_potl = max(max_opt_d_potl, max(opt_d_foll_run_val(:)));
                
                is_foll_in_legend = false;
                 for k_leg_f_check = 1:length(legend_t_potl)
                    if startsWith(legend_t_potl{k_leg_f_check}, sprintf('V%d Opt.', v_id_potl)) 
                        is_foll_in_legend = true; break;
                    end
                end
                if ~is_foll_in_legend
                     set(h_plot_f_potl, 'DisplayName', sprintf('V%d Opt. (Style for Plt %d)', v_id_potl, run_idx_potl));
                     legend_t_potl{end+1} = sprintf('V%d Opt. (Style for Plt %d)', v_id_potl, run_idx_potl); % Aggiungi a legend_t_potl per il check futuro
                else
                     set(h_plot_f_potl, 'HandleVisibility','off');
                end
            end
        end
    end

    % if ~isempty(legend_h_potl) % Ora usa DisplayName
    %     legend(legend_h_potl, legend_t_potl, 'Location','BestOutside', 'NumColumns',1);
    % end
    legend('show', 'Location','BestOutside', 'NumColumns',1);
    
    if max_opt_t_potl == 0, max_opt_t_potl = 150; end
    if max_opt_d_potl == 0, max_opt_d_potl = 1800; end
    xlim([0, max_opt_t_potl]); 
    ylim([-max_opt_d_potl*0.05, max_opt_d_potl * 1.05]); % Permetti piccolo y negativo per offset
    xlabel('Time [s]'); ylabel('Position [m]');
    title('Optimal Trajectories (All Platoons) and Traffic Lights');
end


function save_plot_data(SIM_RUNS_arg) % Rinomina SIM_RUNS
    if isempty(SIM_RUNS_arg),  warning('SIM_RUNS_arg is empty: nothing to save for plot_data.mat'); return; end
    
    out_struct = struct(); % Rinomina out
    
    if isfield(SIM_RUNS_arg{1}, 'traffic_lights')
        out_struct.traffic_lights_config = SIM_RUNS_arg{1}.traffic_lights;
    else
        out_struct.traffic_lights_config = [];
    end
    
    out_struct.num_runs = length(SIM_RUNS_arg); % Rinomina
    out_struct.runs_data = cell(1, length(SIM_RUNS_arg)); % Rinomina

    for i_run_save = 1:length(SIM_RUNS_arg) % Rinomina i
        current_run_struct_save = SIM_RUNS_arg{i_run_save}; % Rinomina
        run_data_s = struct(); % Rinomina
        
        if isfield(current_run_struct_save, 't'), run_data_s.sim_time = current_run_struct_save.t; end
        if isfield(current_run_struct_save, 'x'), run_data_s.veh_states = current_run_struct_save.x; end
        if isfield(current_run_struct_save, 'opt_t'), run_data_s.opt_time = current_run_struct_save.opt_t; end
        if isfield(current_run_struct_save, 'opt_d'), run_data_s.opt_dist = current_run_struct_save.opt_d; end
        if isfield(current_run_struct_save, 'v_targets'), run_data_s.target_vel = current_run_struct_save.v_targets; end
        if isfield(current_run_struct_save, 'leader'), run_data_s.leader_id = current_run_struct_save.leader; end
        if isfield(current_run_struct_save, 'offset'), run_data_s.run_offset = current_run_struct_save.offset; end
        if isfield(current_run_struct_save, 'splittedVehicles')
            run_data_s.splitting_veh = current_run_struct_save.splittedVehicles;
        else
            run_data_s.splitting_veh = []; 
        end
        if isfield(current_run_struct_save, 'oldPlatoonTrajectory') && ~isempty(current_run_struct_save.oldPlatoonTrajectory)
            run_data_s.prev_platoon_traj = current_run_struct_save.oldPlatoonTrajectory;
        end
        
        out_struct.runs_data{i_run_save} = run_data_s; 
    end
    
    try
        save('plot_data.mat', 'out_struct'); 
        fprintf('[INFO] plot_data.mat saved successfully with data from %d run(s).\n', length(SIM_RUNS_arg));
    catch ME_save
        warning('[ERROR] Could not save plot_data.mat. Reason: %s\n', ME_save.message); % Rinomina ME
    end
end
