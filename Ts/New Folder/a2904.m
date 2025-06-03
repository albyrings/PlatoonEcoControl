
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% a) MAIN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
clc;
close all;

global SIM_RUNS N_PLATOON;
SIM_RUNS = {};
N_PLATOON = 1;

disp('=== Avvio prima simulazione con Leader=1 ===');
reset_persistent_variables();

% Richiama l'ottimizzatore con i parametri iniziali (leader=1, tempo=0, posizione=0, nessun set di semafori esterno)
ottimizzatore(1, 0, 0, []);

% Produzione dei plot finali
final_plot();
plot_delta_velocities();
plot_velocity_trigger_per_vehicle();


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% b) OTTIMIZZATORE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ottimizzatore(leader_vehicle, time_offset, start_position, external_traffic_lights)
    % Unico entry-point per ogni scenario (dal main o dai trigger).
    % Se external_traffic_lights è vuoto, costruisce i semafori come in
    % run_optimizer_and_plot; altrimenti, esegue una configurazione
    % parziale come in run_optimizer_from_traffic_light.
    
    global SIM_RUNS N_PLATOON
    
    % Pulizia variabili persistenti (necessaria per evitare conflitti)
    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear check_velocity_triggers
    clear get_current_v_target_indexed
    
    % Parametri base
    final_time       = 150;
    final_distance   = 1800;
    v_min            = 5;
    v_max            = 30;
    b1               = 0.1;  
    b2               = 0.01;
    b3               = 10;
    b4               = 4;
    T                = 30;  % Periodo utile per i cicli semaforici
    tf               = final_time;
    
    % Definizione funzione di disturbo (si può regolare a piacere)
    delta_func = @(t) -0 * (b1 + b2*(sin((1/b3)*t + b4) + 0.25*rand));
    
    fprintf('\n[INFO] ottimizzatore(Leader=%d, offset=%.2f, start_pos=%.2f)\n', ...
        leader_vehicle, time_offset, start_position);
    
    % Se non abbiamo semafori esterni, definiamo quelli standard
    if isempty(external_traffic_lights)
        traffic_lights = [
             create_traffic_light(300,   0, 10, T)
             create_traffic_light(600,  10, 20, T)
             create_traffic_light(900,  20, 30, T)
             create_traffic_light(1200,  0, 10, T)
             create_traffic_light(1550, 10, 20, T)
        ];
    else
        % Se sono forniti esternamente (ad es. da un trigger), filtriamo i semafori
        all_lights = external_traffic_lights;
        traffic_lights = [];
        for i = 1:length(all_lights)
            if all_lights(i).distance > start_position
                traffic_lights = [traffic_lights; all_lights(i)];
            end
        end
    end
    
    % Pruning veloce per i semafori
    [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    nIntersections = length(traffic_lights);
    
    % Costruzione dei nodi
    Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
    nodeId = 1;
    
    % Nodo iniziale
    Nodes(nodeId) = struct('id', nodeId, 't', time_offset, 'd', start_position, 'int', 0);
    nodeId = nodeId + 1;
    
    d = [traffic_lights.distance];
    
    % Creazione nodi semaforici (intervalli verdi)
    for i = 1:nIntersections
        light = traffic_lights(i);
        for k = 0:ceil(tf / light.cycle_time)
            cyc_start = k * light.cycle_time + light.offset;
            
            % Caso classico: green_start <= green_end
            if light.green_start <= light.green_end
                abs_start = cyc_start + light.green_start;
                abs_end   = cyc_start + light.green_end;
                
                if abs_start <= tf + time_offset
                    overlap_start = max(abs_start, t_min(i));
                    overlap_end   = min(abs_end,   t_max(i));
                    if overlap_start < overlap_end
                        mid_time = (overlap_start + overlap_end) / 2;
                        Nodes(nodeId) = struct('id', nodeId, 't', mid_time, 'd', d(i), 'int', i);
                        nodeId = nodeId + 1;
                    end
                end
            else
                % Verde a cavallo
                abs_startA = cyc_start + light.green_start;
                abs_endA   = cyc_start + light.cycle_time;
                if abs_startA <= tf + time_offset
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
                if abs_startB <= tf + time_offset
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
    Nodes(nodeId) = struct('id', nodeId, 't', time_offset + tf, 'd', final_distance, 'int', nIntersections + 1);
    nNodes = nodeId;
    
    % Creazione archi
    Edges = struct('from', {}, 'to', {}, 'w', {});
    edgeCount = 1;
    for i = 1:nNodes
        lvlA = Nodes(i).int;
        for j = 1:nNodes
            lvlB = Nodes(j).int;
            if lvlB == lvlA + 1
                if Nodes(j).t > Nodes(i).t && Nodes(j).d > Nodes(i).d
                    % Controllo semaforo sul nodo di arrivo
                    if lvlB > 0 && lvlB <= nIntersections
                        if ~is_green(traffic_lights(lvlB), Nodes(j).t)
                            continue;
                        end
                    end
                    delta_t = Nodes(j).t - Nodes(i).t;
                    delta_d = Nodes(j).d - Nodes(i).d;
                    v_link = delta_d / delta_t;
                    if v_link >= v_min && v_link <= v_max
                        E_link = delta_t * (b1 * v_link + b2 * v_link^2);
                        Edges(edgeCount) = struct('from', Nodes(i).id, 'to', Nodes(j).id, 'w', E_link);
                        edgeCount = edgeCount + 1;
                    end
                end
            end
        end
    end
    
    % Dijkstra
    [path, cost] = dijkstra(Nodes, Edges, 1, nNodes);
    if isinf(cost)
        error('[ERROR] Nessun percorso valido trovato nell''ottimizzatore.');
    end
    
    opt_nodes = Nodes(path);
    opt_t = arrayfun(@(n) n.t, opt_nodes);
    opt_d = arrayfun(@(n) n.d, opt_nodes);
    
    speeds = zeros(1, length(opt_t) - 1);
    for k = 1:(length(opt_t) - 1)
        dd = opt_d(k + 1) - opt_d(k);
        dt = opt_t(k + 1) - opt_t(k);
        speeds(k) = dd / dt;
    end
    
    fprintf('>>> Leader=%.1f, Costo ottimo=%.3f\n', leader_vehicle, cost);
    
    % Impostazioni per la simulazione
    n_vehicles = 5;
    m_vehicles = 1000 * ones(1, n_vehicles);
    
    % PID e parametri di distanza
    K_p_speed = 7000; K_i_speed = 0; K_d_speed = 0.1;
    K_p_dist  = 2000; K_i_dist = 0.8; K_d_dist = 0.4;
    t_CTH     = 1.5;
    d_init    = 4;
    
    % Stato iniziale: leader parte da start_position, follower dietro
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
        t, x, n_vehicles, m_vehicles, delta_func, traffic_lights, speeds, t_CTH, ...
        K_p_speed, K_i_speed, K_d_speed, K_p_dist, K_i_dist, K_d_dist, ...
        leader_vehicle, time_offset), t_span, x0);
    
    T_abs = t_sim; % t_sim include offset
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


%%%% filepath: /path/to/yourFile.m

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% c) SISTEMA DINAMICO
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dx = system_dynamics_new_platoon( ...
    t, x, n_vehicles, m_vehicles, delta_func, traffic_lights, speeds, t_CTH, ...
    K_p_speed, K_i_speed, K_d_speed, K_p_dist, K_i_dist, K_d_dist, ...
    leader_vehicle, offset)
    % Sistema dinamico di simulazione per un nuovo plotone di veicoli.
    %
    % x = [pos_1, ..., pos_n, vel_1, ..., vel_n]^T
    % leader_vehicle indica l'ID del veicolo leader.
    
    dx = zeros(size(x));
    
    % Estrai le posizioni e velocità
    pos = x(1:n_vehicles);
    vel = x(n_vehicles+1:end);
    
    % Tempo relativo al plotone
    t_rel = t - offset;
    
    % Interpola la velocità target ideale del leader (se disponibile)
    v_leader_opt = 0;
    if ~isempty(speeds)
        idx = min(floor(t_rel), length(speeds));
        if idx < 1, idx = 1; end
        v_leader_opt = speeds(idx);
    end
    
    % Aggiorna posizione e velocità di ogni veicolo
    for i = 1:n_vehicles
        current_pos = pos(i);
        current_vel = vel(i);
        
        % Differenzia posizioni
        dx(i) = current_vel;
        
        if i == leader_vehicle
            % Controller velocità per il leader
            v_error = v_leader_opt - current_vel;
            % Semplice PD (il termine integrale è disabilitato di default)
            a_leader = K_p_speed*v_error - K_d_speed*(0) + delta_func(t_rel);
            dx(n_vehicles + i) = a_leader;
        else
            % Controller distanza per i follower
            % Calcolo distanza e velocità relativa rispetto al veicolo di fronte
            leader_pos = pos(leader_vehicle);
            leader_vel = vel(leader_vehicle);
            desired_gap = 1 + t_CTH * leader_vel;
            gap_error = (leader_pos - current_pos) - desired_gap*(leader_vehicle - i);
            
            % PID semplificato per la distanza
            a_follower = K_p_dist*gap_error - K_d_dist*(0) + delta_func(t_rel);
            dx(n_vehicles + i) = a_follower;
        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% d) SISTEMA DI TRIGGER
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function check_red_light_violations(t_abs, x_sim, traffic_lights, T)
    % Verifica se il plotone ha oltrepassato un semaforo col rosso.
    %
    % Placeholder per la logica di rilevamento violazioni semaforo.
    % Implementare se necessario.
end


function check_velocity_triggers(t_abs, x_sim, traffic_lights)
    % Verifica scostamenti di velocità e potenziali split del plotone.
    %
    % Se viene rilevato un trigger, crea un nuovo plotone (chiamando ottimizzatore).
    
    persistent trigger_detected
    if isempty(trigger_detected), trigger_detected = false; end
    
    global SIM_RUNS N_PLATOON
    
    % Ottieni run corrente
    current_run = length(SIM_RUNS);
    if current_run < 1, return; end
    
    runData = SIM_RUNS{current_run};
    n_vehicles = size(x_sim, 2) / 2;
    
    % Per ogni veicolo controlla scostamenti o condizioni di split
    for v = 1:n_vehicles
        if v == runData.leader, continue; end
        
        % Qui potete inserire la logica di trigger specifica (ad es. scostamento velocità).
        % In caso di trigger:
        if false  % <-- Inserire la condizione reale
            if ~trigger_detected
                trigger_detected = true;
                fprintf('[TRIGGER] Veicolo %d diventa leader di un nuovo plotone.\n', v);
                SIM_RUNS{current_run}.splittedVehicles = v:n_vehicles;
                
                % Crea un nuovo plotone
                trigger_time = t_abs(end);
                stop_position = x_sim(end, v);
                
                rerun_optimizer_for_trigger(v, trigger_time, stop_position);
                trigger_detected = false;
                return;
            end
        end
    end
end


function trigger_events = velocity_trigger(t, diff_v, opt_v)
    % Restituisce un vettore 0/1 se lo scostamento di velocità supera una soglia.
    % Placeholder per logica di calcolo trigger.
    
    trigger_events = zeros(size(t));
    % Esempio di soglia da impostare:
    threshold = 5;  % m/s
    
    for i = 1:length(t)
        if abs(diff_v(i)) > threshold
            trigger_events(i) = 1;
        end
    end
end


function check_and_trigger(t_abs, x_sim, traffic_lights, leader_vehicle)
    % Funzione generica di controllo e possibile split del plotone.
    % Placeholder per logica personalizzata su semafori / velocità.
    
    % Esempio: se trova un problema, chiama run_new_platoon...
end


function [veh_red, time_red, pos_red] = detect_red_light(t_abs, x_sim, lights)
    % Placeholder per rilevare il primo veicolo che troverà rosso, se necessario.
    veh_red = [];
    time_red = [];
    pos_red = [];
end


function [veh_vel_diff, trigger_time, trigger_pos] = detect_speed_trigger(t_abs, x_sim, lights)
    % Placeholder per rilevare il primo veicolo con scostamento eccessivo.
    veh_vel_diff = [];
    trigger_time = [];
    trigger_pos  = [];
end


function run_new_platoon(leader_id, start_time, start_position, traffic_lights)
    % Crea un nuovo plotone a partire dal veicolo 'leader_id'.
    % Richiama l'ottimizzatore con i parametri attuali.
    
    global N_PLATOON
    N_PLATOON = N_PLATOON + 1;
    
    fprintf('[INFO] Creazione NUOVO PLOTONE %d con Leader=%d\n', N_PLATOON, leader_id);
    ottimizzatore(leader_id, start_time, start_position, traffic_lights);
end


function rerun_optimizer_for_trigger(trigger_vehicle, trigger_time, stop_position)
    % Richiamato quando si attiva un trigger di velocità.
    % Fa ripartire l'ottimizzatore dal veicolo segnalato come leader.
    
    global N_PLATOON
    N_PLATOON = N_PLATOON + 1;
    
    fprintf('[INFO] TRIGGER: Creazione NUOVO PLOTONE %d con LEADER=%d\n', ...
        N_PLATOON, trigger_vehicle);
    
    ottimizzatore(trigger_vehicle, trigger_time, stop_position, []);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% e) FUNZIONI RESTANTI
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plot_energy_consumption()
    global SIM_RUNS
    
    if isempty(SIM_RUNS)
        disp('Nessuna simulazione disponibile.');
        return;
    end
    
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
        
        % Calcolo del consumo energetico (approssimato)
        n_vehicles = size(x, 2)/2;
        total_energy = 0;
        
        % Determina i veicoli in questo plotone
        platoon_vehicles = get_platoon_vehicles(run_i);
        platoon_vehicle_count(run_i) = length(platoon_vehicles);
        
        for v = platoon_vehicles
            velocity = x(:, n_vehicles + v);
            dt = diff(t);
            segment_energy = sum(dt .* (b1*velocity(1:end-1) + b2*velocity(1:end-1).^2));
            total_energy = total_energy + segment_energy;
        end
        
        platoon_energy(run_i) = total_energy;
    end
    
    % Visualizza il consumo energetico per plotone
    bar(1:length(SIM_RUNS), platoon_energy);
    
    % Etichette
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
    
    if isempty(SIM_RUNS)
        disp('Nessuna simulazione disponibile.');
        return;
    end
    
    figure('Name', 'Distanze tra veicoli', 'Position', [350, 350, 1000, 600]);
    
    n_plots = length(SIM_RUNS);
    rows = ceil(sqrt(n_plots));
    cols = ceil(n_plots/rows);
    
    for run_i = 1:n_plots
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        leader = runData.leader;
        
        subplot(rows, cols, run_i);
        hold on;
        
        platoon_vehicles = get_platoon_vehicles(run_i);
        
        if length(platoon_vehicles) >= 2
            n_vehicles = size(x,2)/2;
            [~, order_idx] = sort(x(end, platoon_vehicles), 'descend');
            sorted_vehicles = platoon_vehicles(order_idx);
            
            distances = [];
            labels = {};
            
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
            
            if ~isempty(distances) && all(isfinite(distances))
                ylim([0, max(max(distances), 10)]);
            else
                ylim([0, 10]);
            end
        else
            text(0.5, 0.5, 'Veicoli insufficienti per calcolare le distanze', 'HorizontalAlignment', 'center');
            axis([0 1 0 1]);
        end
    end
end


function platoon_vehicles = get_platoon_vehicles(run_i)
    global SIM_RUNS
    n_vehicles = size(SIM_RUNS{1}.x, 2)/2;
    
    if run_i == 1
        platoon_vehicles = 1:n_vehicles;
        if isfield(SIM_RUNS{1}, 'splittedVehicles') && ~isempty(SIM_RUNS{1}.splittedVehicles)
            platoon_vehicles = setdiff(platoon_vehicles, SIM_RUNS{1}.splittedVehicles);
        end
        return;
    end
    
    current_leader = SIM_RUNS{run_i}.leader;
    prev_run = SIM_RUNS{run_i - 1};
    
    if isfield(prev_run, 'splittedVehicles') && ~isempty(prev_run.splittedVehicles)
        platoon_vehicles = prev_run.splittedVehicles;
        
        if isfield(SIM_RUNS{run_i}, 'splittedVehicles') && ~isempty(SIM_RUNS{run_i}.splittedVehicles)
            platoon_vehicles = setdiff(platoon_vehicles, SIM_RUNS{run_i}.splittedVehicles);
        end
    else
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
    
    t_CTH = 1.5;
    d_min = 1;
    
    initial_offset = abs(follower_id - leader_id);
    
    if follower_id > leader_id
        follower_opt_d(1) = opt_d(1) - initial_offset;
    else
        follower_opt_d(1) = opt_d(1) + initial_offset;
    end
    
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

    for i = 1:n
        dist_from_start = d(i) - start_position;
        if dist_from_start < 0
            t_min(i) = start_time;
            t_max(i) = start_time;
            continue;
        end

        t_min(i) = start_time + dist_from_start/v_max;
        t_max(i) = start_time + dist_from_start/v_min;
        t_max(i) = min(t_max(i), tf - (final_distance - d(i))/v_max);

        t_min(i) = next_green(traffic_lights(i), t_min(i));
        t_max(i) = prev_green(traffic_lights(i), t_max(i));

        if t_max(i) < t_min(i)
            t_max(i) = t_min(i);
        end
    end

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


function plot_velocity_trigger_per_vehicle()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('Nessun dato da plottare per i trigger delle velocità.');
        return;
    end
    
    runData = SIM_RUNS{1};
    t_sim = runData.t;
    n_vehicles = size(runData.x, 2)/2;
    
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
        
        h1 = plot(t_sim, v_sim, 'b-', 'LineWidth', 1.5);
        h2 = plot(t_sim, v_opt, 'g--', 'LineWidth', 1.5);
        h3 = plot(t_sim, diff_v, 'r-.', 'LineWidth', 1.5);
        
        h4 = plot(t_sim, trigger_state * (max(abs(diff_v))*0.8), 'm-', 'LineWidth', 2);
        
        y_lim = get(gca, 'YLim');
        max_y = y_lim(2);
        
        % (Eventuali linee verticali per split, da aggiungere se servono)
        
        legend_handles = [h1, h2, h3, h4];
        legend_names = {'Velocità Simulata', 'Velocità Ottimale', 'Delta Velocità', 'Trigger Attivo'};
        legend(legend_handles, legend_names, 'Location', 'best');
        
        xlabel('Tempo [s]');
        ylabel('Velocità [m/s]');
        title(['Veicolo ' num2str(v) ': Trigger e Split Detection']);
    end
end


function final_plot()
    % Richiamato dal main per produrre alcuni grafici
    plot_energy_consumption();
    plot_inter_vehicle_distances();
end


function next_time = next_green(light, t_in)
    % Allinea il tempo t_in al successivo periodo di verde del semaforo.
    cycle = light.cycle_time;
    offset = light.offset;
    green_s = light.green_start;
    green_e = light.green_end;

    dt = t_in - offset;
    k = floor(dt / cycle);
    phase = dt - k*cycle;
    if green_s <= green_e
        if phase < green_s
            next_time = offset + k*cycle + green_s;
        elseif phase > green_e
            next_time = offset + (k+1)*cycle + green_s;
        else
            next_time = t_in; 
        end
    else
        if phase > green_e && phase < green_s
            next_time = offset + (k+1)*cycle + green_s;
        else
            next_time = t_in;
        end
    end
end


function prev_time = prev_green(light, t_in)
    % Allinea t_in al termine del periodo di verde.
    cycle = light.cycle_time;
    offset = light.offset;
    green_s = light.green_start;
    green_e = light.green_end;

    dt = t_in - offset;
    k = floor(dt / cycle);
    phase = dt - k*cycle;

    if green_s <= green_e
        if phase > green_e
            prev_time = offset + k*cycle + green_e;
        elseif phase < green_s
            prev_time = offset + (k-1)*cycle + green_e;
        else
            prev_time = t_in;
        end
    else
        if phase < green_s && phase > green_e
            prev_time = offset + k*cycle + green_e;
        else
            prev_time = t_in;
        end
    end
end


function flag = is_green(light, t_in)
    % Ritorna vero se al tempo t_in il semaforo è in verde.
    cycle = light.cycle_time;
    offset = light.offset;
    green_s = light.green_start;
    green_e = light.green_end;

    dt = t_in - offset;
    if dt < 0
        flag = false;
        return;
    end
    k = floor(dt / cycle);
    phase = dt - k*cycle;

    if green_s <= green_e
        flag = (phase >= green_s) && (phase <= green_e);
    else
        if phase >= green_s || phase <= green_e
            flag = true;
        else
            flag = false;
        end
    end
end


