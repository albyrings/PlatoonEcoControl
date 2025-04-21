%% File: complete_code.m
% Questo file contiene la simulazione completa, la pianificazione, l'analisi e il plotting
% delle traiettorie ottimali/reali e dei trigger basati sulla differenza di posizione.

%% Inizializzazione
clear;
clearAllMemoizedCaches; 
clc; 
close all;
reset_persistent_variables();  % Resetta eventuali variabili persistenti

global SIM_RUNS;    % Contiene i dati delle simulazioni
global N_PLATOON;   % Conta il numero di plotoni

SIM_RUNS = {};
N_PLATOON = 1;

disp('=== Avvio prima simulazione con Leader=1 ===');
run_optimizer_and_plot(1, 0);   % Esegue la simulazione con leader = 1 e offset = 0

% Plot principali:
final_plot();
plot_position_triggers_per_vehicle();
plot_delta_values();
plot_all_vehicles_on_traffic_map()
plot_delta_velocities();
plot_velocity_trigger_per_vehicle();
%% --------------------------------------------------------------------
%% FUNZIONI DI SIMULAZIONE E DI PIANIFICAZIONE
%% --------------------------------------------------------------------
function run_optimizer_and_plot(leader_vehicle, time_offset)
    global SIM_RUNS

    % Parametri della simulazione
    final_time     = 150;          % Durata della simulazione [s]
    final_distance = 1800;         % Distanza finale [m]
    T  = 30;                      % Periodo dei semafori
    tf = final_time;
    v_min = 5;                   % Velocità minima [m/s]
    v_max = 30;                  % Velocità massima [m/s]
    b1 = 0.1;  
    b2 = 0.01;

    fprintf('\n[INFO] run_optimizer_and_plot(Leader=%d, offset=%.2f)\n', leader_vehicle, time_offset);

    %% Creazione dei semafori
    T_cycle = T;
    traffic_lights = [ ...
        create_traffic_light(300,   0, 10, T_cycle); ...
        create_traffic_light(600,  10, 20, T_cycle); ...
        create_traffic_light(900,  20, 30, T_cycle); ...
        create_traffic_light(1200,  0, 10, T_cycle); ...
        create_traffic_light(1550, 10, 20, T_cycle)  ...
        ];

    % Calcola i tempi minimi e massimi per passare i semafori
    [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance]; 
    nIntersections = length(traffic_lights);

    %% Costruzione dei nodi per la pianificazione
    Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
    nodeId = 1;
    Nodes(nodeId) = struct('id', nodeId, 't', 0, 'd', 0, 'int', 0); 
    nodeId = nodeId+1;
    for i = 1:nIntersections
        light = traffic_lights(i);
        for j = 0:ceil(tf / light.cycle_time)
            cycle_start = j * light.cycle_time + light.offset;
            if light.green_start <= light.green_end
                abs_green_start = cycle_start + light.green_start;
                abs_green_end   = cycle_start + light.green_end;
                if abs_green_start <= tf
                    overlap_start = max(abs_green_start, t_min(i));
                    overlap_end   = min(abs_green_end, t_max(i));
                    if overlap_start < overlap_end
                        middle_time = ceil((overlap_start + overlap_end)/2);
                        Nodes(nodeId) = struct('id', nodeId, 't', middle_time, 'd', d(i), 'int', i);
                        nodeId = nodeId + 1;
                    end
                end
            else
                % Ciclo "a cavallo" (verde attraversa il ciclo)
                abs_green_start_1 = cycle_start + light.green_start;
                abs_green_end_1   = cycle_start + light.cycle_time;
                if abs_green_start_1 <= tf
                    ov_start = max(abs_green_start_1, t_min(i));
                    ov_end   = min(abs_green_end_1, t_max(i));
                    if ov_start < ov_end
                        mid_t = (ov_start+ov_end)/2; 
                        Nodes(nodeId) = struct('id', nodeId, 't', mid_t, 'd', d(i), 'int', i);
                        nodeId = nodeId + 1;
                    end
                end
                abs_green_start_2 = cycle_start;
                abs_green_end_2   = cycle_start + light.green_end;
                if abs_green_end_2 <= tf
                    ov_start = max(abs_green_start_2, t_min(i));
                    ov_end   = min(abs_green_end_2, t_max(i));
                    if ov_start < ov_end
                        mid_t = (ov_start+ov_end)/2;
                        Nodes(nodeId) = struct('id', nodeId, 't', mid_t, 'd', d(i), 'int', i);
                        nodeId = nodeId + 1;
                    end
                end
            end
        end
    end
    % Nodo finale
    Nodes(nodeId) = struct('id', nodeId, 't', tf, 'd', final_distance, 'int', nIntersections+1);
    nNodes = nodeId;

    %% Creazione degli archi (collegamenti tra nodi)
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
                            continue;  % Salta collegamenti se il semaforo non è verde
                        end
                    end
                    delta_t = Nodes(j).t - Nodes(i).t;
                    delta_d = Nodes(j).d - Nodes(i).d;
                    v_link = delta_d / delta_t;
                    if v_link >= v_min && v_link <= v_max
                        E_link = delta_t*(b1*v_link + b2*v_link^2);
                        Edges(edgeCount) = struct('from', Nodes(i).id, 'to', Nodes(j).id, 'w', E_link);
                        edgeCount = edgeCount + 1;
                    end
                end
            end
        end
    end

    %% Calcolo del percorso ottimo con Dijkstra
    [path, cost] = dijkstra(Nodes, Edges, 1, nNodes);
    fprintf('>>> Leader=%.1f, Costo ottimo=%.3f\n', leader_vehicle, cost);
    opt_nodes = Nodes(path);
    opt_t = arrayfun(@(n) n.t, opt_nodes);
    opt_d = arrayfun(@(n) n.d, opt_nodes);

    % Stampa delle velocità medie tra i nodi ottimali
    speeds = zeros(1, length(path)-1);
    for k = 1:length(path)-1
        speeds(k) = (opt_d(k+1) - opt_d(k))/(opt_t(k+1) - opt_t(k));
    end
    disp('Velocità medie (m/s):');
    disp(num2str(speeds, '%.2f '));

    %% Simulazione ODE per il plotone
    n_vehicles = 5;
    m_vehicles = 1000 * ones(1, n_vehicles);
    v_targets = speeds;  

    % Parametri PID per il controllo (leader e follower)
    K_p_speed = 7000; K_i_speed = 0; K_d_speed = 0.1;
    K_p_dist  = 2000; K_i_dist  = 0.8; K_d_dist  = 0.4;
    t_CTH = 1.5;  
    d_init = 4;

    % Inizializzazione delle condizioni iniziali: le posizioni sono separate di d_init
    x0 = zeros(2 * n_vehicles, 1);
    for i = 1:n_vehicles
        if i == 1
            x0(i) = 0;
        else
            x0(i) = -d_init * (i - 1);
        end
    end

    t_span = [0 150];
    [t_sim, x_sim] = ode45(@(t, x) system_dynamics_new_platoon(t, x, n_vehicles, m_vehicles, @(tt) 0, traffic_lights, v_targets, t_CTH, ...
                            K_p_speed, K_i_speed, K_d_speed, K_p_dist, K_i_dist, K_d_dist, leader_vehicle, time_offset),...
                            t_span, x0);

    % Salvataggio dei risultati della simulazione
    T_abs = t_sim + time_offset;
    SIM_RUNS{end + 1} = struct( ...
        'leader', leader_vehicle, ...
        't', T_abs, ...
        'x', x_sim, ...
        'offset', time_offset, ...
        'traffic_lights', traffic_lights, ...
        'splittedVehicles', [], ...
        'v_targets', speeds, ...
        'opt_t', opt_t + time_offset, ... % tempi ottimali
        'opt_d', opt_d);                 % distanze ottimali

    % Verifica il rispetto dei semafori da parte dei veicoli
    check_red_light_violations(T_abs, x_sim, traffic_lights, T_cycle);
end

function dx = system_dynamics_new_platoon(t, x, n_vehicles, m, delta_func, traffic_lights, v_targets, t_CTH, ...
                                           K_p_speed, K_i_speed, K_d_speed, K_p_dist, K_i_dist, K_d_dist, ...
                                           leader_vehicle, time_offset)
    % Questa funzione definisce il sistema dinamico (ODE) per un plotone di veicoli
    dx = zeros(2 * n_vehicles, 1);
    
    persistent t_prev
    if isempty(t_prev)
        t_prev = t; 
    end
    dt = t - t_prev;
    if dt <= 0
        dt = 0.00001; 
    end  
    t_prev = t;

    % Variabili persistenti per i controlli PID
    persistent e_int_speed e_old_speed
    persistent e_int_dist  e_old_dist
    if isempty(e_int_speed)
        e_int_speed = 0; 
        e_old_speed = 0;
    end
    if isempty(e_int_dist)
        e_int_dist = zeros(n_vehicles, 1); 
        e_old_dist = zeros(n_vehicles, 1);
    end

    abs_t = t + time_offset;

    for i = 1:n_vehicles
        % La derivata della posizione è la velocità
        dx(i) = x(n_vehicles + i);
        
        if i == leader_vehicle
            % Controllo PID per il veicolo leader
            vt = get_current_v_target_indexed(x(leader_vehicle), traffic_lights, v_targets);
            vel_err = vt - x(n_vehicles + i);
            e_int_speed = e_int_speed + vel_err * dt;
            vel_deriv = (vel_err - e_old_speed) / dt;
            e_old_speed = vel_err;
            U_leader = K_p_speed * vel_err + K_i_speed * e_int_speed + K_d_speed * vel_deriv;
            dx(n_vehicles + i) = (U_leader + delta_func(abs_t)) / m(i);
            max_speed = 30;
            new_vel = x(n_vehicles + i) + dx(n_vehicles + i) * dt;
            new_vel = min(max(new_vel, 0), max_speed);
            dx(n_vehicles + i) = (new_vel - x(n_vehicles + i)) / dt;
        else
            % Controllo PID per i follower: mantenimento della distanza
            if i > 1
                dist = x(i - 1) - x(i);
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
            new_vel = x(n_vehicles + i) + dx(n_vehicles + i) * dt;
            new_vel = max(new_vel, 0);
            dx(n_vehicles + i) = (new_vel - x(n_vehicles + i)) / dt;
        end
    end
end

function vt = get_current_v_target_indexed(x_leader, traffic_lights, v_targets)
    % Seleziona il target di velocità in base alla posizione del leader e alla distanza dei semafori
    idx = find(x_leader < [traffic_lights.distance], 1);
    if isempty(idx)
        vt = v_targets(end);
    else
        vt = v_targets(min(idx, length(v_targets)));
    end
end

function check_red_light_violations(t_abs, x_sim, traffic_lights, T)
    % Verifica se un veicolo attraversa un semaforo col rosso e avvia il meccanismo di ri-ottimizzazione
    persistent new_leader_detected
    if isempty(new_leader_detected)
        new_leader_detected = false;
    end

    n_vehicles = size(x_sim, 2) / 2;
    for v = 1:n_vehicles
        pos_v = x_sim(:, v);
        for L = 1:length(traffic_lights)
            light_d = traffic_lights(L).distance;
            cross_idx = find(pos_v(1:end-1) < light_d & pos_v(2:end) >= light_d, 1);
            if ~isempty(cross_idx)
                cross_time = t_abs(cross_idx);
                if ~is_green(traffic_lights(L), cross_time)
                    fprintf('\n[WARNING] Veicolo %d passa col rosso a incrocio %d (t=%.2f s)\n', v, L, cross_time);
                    if ~new_leader_detected
                        new_leader_detected = true;
                        fprintf('>> Veicolo %d diventa leader di un nuovo plotone!\n', v);
                        global SIM_RUNS
                        last_idx = length(SIM_RUNS);
                        SIM_RUNS{last_idx}.splittedVehicles = v : n_vehicles;
                        rerun_optimizer_for_new_leader(v, T);
                        new_leader_detected = false;
                    end
                    return;
                end
            end
        end
    end
end

function rerun_optimizer_for_new_leader(violating_vehicle, T)
    % Cancella le funzioni ODE e riparte la simulazione con un nuovo leader
    clear system_dynamics_new_platoon
    clear get_current_v_target_indexed
    clear check_red_light_violations

    global SIM_RUNS
    global N_PLATOON
    
    disp(['[INFO] Ricalcolo con NUOVO LEADER = ', num2str(violating_vehicle), ', riparto da tempo assoluto = ', num2str(N_PLATOON*T)]);
    start_offset = N_PLATOON * T;  
    N_PLATOON = N_PLATOON + 1;
    run_optimizer_and_plot(violating_vehicle, start_offset);
end

%% --------------------------------------------------------------------
%% FUNZIONI PER IL PLOTTING DELLE TRAIETTORIE
%% --------------------------------------------------------------------
function final_plot()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[final_plot] Nessun dato da plottare.');
        return;
    end

    % FIGURA 1: Traiettorie ottimali e stato dei semafori
    figure('Name','Posizioni Ottimali e Semafori', 'Position', [100, 100, 1000, 600]);
    hold on;
    
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    
    % Costruzione di un vettore per il plotting dei semafori
    max_time = 0;
    for j = 1:length(SIM_RUNS)
        max_time = max(max_time, max(SIM_RUNS{j}.t));
    end
    times = 0:ceil(max_time);
    all_times = [];
    all_distances = [];
    all_colors = [];
    for i = 1:length(traffic_lights)
        for j = 1:length(times)
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
    scatter(all_times, all_distances, 10, all_colors, 'filled', 'DisplayName', 'Semafori');
    
    markers = {'o','s','d','^','v','>','<'};
    colors = {'b','r','g','m','c','k',[0.8 0.4 0],[0.5 0.5 0.5],[0.2 0.6 0.8]};
    line_styles = {'-', '--', ':', '-.'};
    legend_handles = [];
    legend_texts = {};
    
    n_vehicles = size(SIM_RUNS{1}.x, 2) / 2;
    % Plot delle traiettorie ottimali per ogni run
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        leader = runData.leader;
        if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
            opt_t = runData.opt_t;
            opt_d = runData.opt_d;
            
            % Determina l'ordine dei veicoli del plotone
            platoon_vehicles = [];
            if run_i == 1
                platoon_vehicles = 1:n_vehicles;
            else
                prev_run = SIM_RUNS{run_i - 1};
                if isfield(prev_run, 'splittedVehicles') && ~isempty(prev_run.splittedVehicles)
                    platoon_vehicles = prev_run.splittedVehicles;
                end
            end
            
            % Calcola velocità target tra i nodi ottimali
            v_targets = [];
            for j = 1:length(opt_t)-1
                v_targets(j) = (opt_d(j+1) - opt_d(j)) / (opt_t(j+1) - opt_t(j));
            end
            
            color_idx = mod(leader - 1, length(colors)) + 1;
            line_idx  = mod(run_i - 1, length(line_styles)) + 1;
            marker_idx = mod(run_i - 1, length(markers)) + 1;
            h = plot(opt_t, opt_d, [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 3);
            scatter(opt_t, opt_d, 70, colors{color_idx}, markers{marker_idx}, 'filled');
            legend_handles(end+1) = h;
            legend_texts{end+1} = ['Leader ' num2str(leader) ' (Plotone ' num2str(run_i) ')'];
            
            % Plot dei follower (utilizzo di un gap fisso)
            if isempty(platoon_vehicles)
                ordered_vehicles = leader;
            else
                x_initial = runData.x(1, 1:n_vehicles);
                [~, idx] = sort(x_initial, 'descend');
                ordered_vehicles = idx;
            end
            
            for vidx = 1:length(ordered_vehicles)
                v = ordered_vehicles(vidx);
                if v == leader
                    continue;
                end
                follower_opt_t = opt_t;
                follower_opt_d = zeros(size(opt_d));
                for j = 1:length(opt_t)
                    if j == 1
                        current_pos = opt_d(1);
                    else
                        current_pos = opt_d(j);
                    end
                    safety_gap = 1 + 1.5;  % d_min + t_CTH fisso
                    follower_opt_d(j) = current_pos - safety_gap;
                end
                if ~exist('follower_positions', 'var')
                    follower_positions = struct();
                end
                follower_positions.(['v' num2str(v)]) = follower_opt_d;
                
                follower_color_idx = mod(v - 1, length(colors)) + 1;
                follower_line_idx = mod(run_i - 1, length(line_styles)) + 1;
                follower_marker_idx = mod(v - 1, length(markers)) + 1;
                h_follower = plot(opt_t, follower_opt_d, [colors{follower_color_idx}, line_styles{follower_line_idx}], 'LineWidth', 2);
                scatter(opt_t, follower_opt_d, 40, colors{follower_color_idx}, markers{follower_marker_idx}, 'filled');
                legend_handles(end+1) = h_follower;
                legend_texts{end+1} = ['Follower ' num2str(v) ' (Plotone ' num2str(run_i) ')'];
            end
        end
    end
    
    legend(legend_handles, legend_texts, 'Location', 'Best');
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
    title('Traiettorie ottimali dei veicoli e stato semafori');
    grid on;
    
    % FIGURA 2: Traiettorie reali
    figure('Name','Grafico Traiettorie Reali', 'Position', [150, 150, 1000, 600]);
    hold on;
    scatter(all_times, all_distances, 10, all_colors, 'filled');
    
    colors = {'b', 'r', 'g', 'm', 'c', 'y', 'k'};
    line_styles = {'-', '-', ':', '-.'};
    plotted_vehicles = [];
    
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        if isfield(runData, 'splittedVehicles')
            splitted = runData.splittedVehicles;
        else
            splitted = [];
        end
        for v = 1:size(x, 2) / 2
            if ismember(v, splitted) || ismember(v, plotted_vehicles)
                continue;
            end
            color_idx = mod(v - 1, length(colors)) + 1;
            line_idx = mod(run_i - 1, length(line_styles)) + 1;
            plot(t, x(:, v), [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 2);
            plotted_vehicles = [plotted_vehicles, v];
        end
    end
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
    title('Traiettorie reali dei veicoli e stato semafori');
    grid on;
end

%% --------------------------------------------------------------------
%% FUNZIONI DI ANALISI DEI PROFILI DI POSIZIONE
%% --------------------------------------------------------------------

%% --------------------------------------------------------------------
%% FUNZIONI DI SUPPORTO PER I TRIGGER BASATI SULLA POSIZIONE
%% --------------------------------------------------------------------
% Le funzioni seguenti valutano il trigger in base alla differenza di posizione
function trigger_events = position_trigger(t, diff_pos, opt_pos)
    trigger_threshold = 30;      % Soglia trigger in metri
    disable_duration  = 10;     % Durata di disattivazione del trigger [s]
    rapid_change_thresh = 2;    % Soglia per variazioni rapide [m]
    initial_delay = 5;          % Tutti i trigger spenti nei primi 5 s
    
    trigger_events = zeros(size(t));
    disable_until = -inf;
    for i = 1:length(t)
        if t(i) < initial_delay
            trigger_events(i) = 0;
        elseif t(i) < disable_until
            trigger_events(i) = 0;
        else
            if abs(diff_pos(i)) > trigger_threshold
                trigger_events(i) = 1;
            else
                trigger_events(i) = 0;
            end
        end
        if i > 1
            delta_opt = abs(opt_pos(i) - opt_pos(i-1));
            if delta_opt > rapid_change_thresh
                disable_until = t(i) + disable_duration;
            end
        end
    end
end



function plot_position_triggers_per_vehicle()
    % Plot dei trigger per ogni veicolo con eventuale riattivazione dell'ottimizzatore
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[plot_position_triggers_per_vehicle] Nessun dato da plottare.');
        return;
    end

    t_CTH = 1.5;  d_min = 1;
    max_vehicle_id = 0;
    for run_i = 1:length(SIM_RUNS)
        n_vehicles = size(SIM_RUNS{run_i}.x, 2) / 2;
        max_vehicle_id = max(max_vehicle_id, n_vehicles);
    end

    vehicles_in_platoon2 = [];
    if length(SIM_RUNS) >= 2 && isfield(SIM_RUNS{1}, 'splittedVehicles')
        vehicles_in_platoon2 = SIM_RUNS{1}.splittedVehicles;
    end

    for v = 1:max_vehicle_id
        if ~ismember(v, vehicles_in_platoon2)
            valid_run_i = 1;
        else
            valid_run_i = [];
            for run_i = 2:length(SIM_RUNS)
                n_vehicles = size(SIM_RUNS{run_i}.x, 2) / 2;
                if v <= n_vehicles
                    valid_run_i = run_i;
                    break;
                end
            end
            if isempty(valid_run_i)
                continue;
            end
        end

        runData = SIM_RUNS{valid_run_i};
        t_sim = runData.t;
        n_vehicles = size(runData.x, 2) / 2;
        if v > n_vehicles, continue; end

        pos_sim = runData.x(:, v);
        if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
            opt_t = runData.opt_t;
            opt_d = runData.opt_d;
            pos_opt = interp1(opt_t, opt_d, t_sim, 'linear', 'extrap');
            
            % Calcola l'offset pari a (pos_opt(tf) - pos_sim(tf))
            offset_value = pos_opt(end) - pos_sim(end);
            % Trasla la differenza di posizione
            diff_pos = (pos_opt - pos_sim) - offset_value;
            trigger_state = position_trigger(t_sim, diff_pos, pos_opt - offset_value);
            check_and_reactivate_optimizer(t_sim, trigger_state, v);
            
            figure('Name', ['Trigger per Veicolo ' num2str(v) ' (Run ' num2str(valid_run_i) ')'], 'Position', [200+v*30, 200+v*30, 900, 500]);
            hold on; grid on;
            h_diff = plot(t_sim, diff_pos, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Δ Posizione traslata');
            h_trig = plot(t_sim, trigger_state * (max(diff_pos) * 0.8), 'r--', 'LineWidth', 2, 'DisplayName', 'Trigger Attivo');
            xlabel('Tempo [s]');
            ylabel('Differenza di Posizione [m]');
            title(['Veicolo ' num2str(v) ' - Δ Posizione e Trigger']);
            legend([h_diff, h_trig], 'Location', 'best');
        end
    end
end

%% --------------------------------------------------------------------
%% FUNZIONI DI SUPPORTO GENERALE
%% --------------------------------------------------------------------
function light = create_traffic_light(distance, green_start, green_end, cycle_time)
    % Crea una struttura contenente i parametri di un semaforo
    light.distance      = distance;
    light.green_start   = green_start;
    light.green_end     = green_end;
    light.cycle_time    = cycle_time;
    light.green_duration= green_end - green_start;
    light.offset        = mod(green_start, cycle_time);
end

function st = is_green(light, time)
    % Ritorna true se il semaforo è verde al tempo "time"
    t_in_cycle = mod(time - light.offset, light.cycle_time);
    if light.green_start <= light.green_end
        st = (t_in_cycle >= light.green_start && t_in_cycle < light.green_end);
    else
        st = (t_in_cycle >= light.green_start || t_in_cycle < light.green_end);
    end
end

function [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max)
    % Calcola i tempi minimi e massimi consentiti per attraversare i semafori
    n = length(traffic_lights);
    d = [traffic_lights.distance];
    t_min = zeros(1, n); 
    t_max = zeros(1, n);
    t_min(1) = d(1) / v_max; 
    t_max(1) = d(1) / v_min;
    t_min(1) = next_green(traffic_lights(1), t_min(1));
    t_max(1) = prev_green(traffic_lights(1), t_max(1));
    
    for i = 2:n
        dist_inc = d(i) - d(i - 1);
        t_min(i) = t_min(i - 1) + dist_inc / v_max;
        t_max(i) = t_max(i - 1) + dist_inc / v_min;
        t_max(i) = min(t_max(i), tf - (final_distance - d(i)) / v_max);
        t_min(i) = next_green(traffic_lights(i), t_min(i));
        t_max(i) = prev_green(traffic_lights(i), t_max(i));
    end
    
    for i = n:-1:2
        needed_t = (d(i) - d(i - 1)) / v_max;
        if t_max(i) > t_max(i - 1) + needed_t
            t_max(i - 1) = t_max(i) - needed_t;
            t_max(i - 1) = prev_green(traffic_lights(i - 1), t_max(i - 1));
        end
    end
end

function t_next = next_green(light, t)
    % Ritorna il prossimo istante di verde a partire da t
    if is_green(light, t)
        t_next = t;
    else
        cyc = mod(t - light.offset, light.cycle_time);
        if cyc < light.green_start
            t_next = t + (light.green_start - cyc);
        else
            t_next = t + (light.cycle_time - cyc) + light.green_start;
        end
    end
end

function t_prev = prev_green(light, t)
    % Ritorna l'istante di verde precedente a partire da t
    if is_green(light, t)
        t_prev = t;
    else
        cyc = mod(t - light.offset, light.cycle_time);
        if cyc >= light.green_end
            t_prev = t - (cyc - light.green_end);
        else
            t_prev = t - cyc - (light.cycle_time - light.green_end);
        end
    end
end

function [path, cost] = dijkstra(Nodes, Edges, source, target)
    % Implementazione dell'algoritmo di Dijkstra per il calcolo del percorso ottimo
    nNodes = length(Nodes);
    cost = inf(1, nNodes);
    prev = nan(1, nNodes);
    cost(source) = 0;
    Q = 1:nNodes;
    while ~isempty(Q)
        [~, idx] = min(cost(Q));
        u = Q(idx);
        Q(Q == u) = [];
        if u == target, break; end
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

function reset_persistent_variables()
    % Rimuove le funzioni persistenti usate nel calcolo della dinamica e altri metodi
    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear get_current_v_target_indexed
    clear next_green
    clear prev_green
    clear dijkstra
    disp('Persistent variables reset.');
end

function check_and_reactivate_optimizer(t, trigger_state, vehicle_id)
    % Controlla se il trigger rimane attivo per un intervallo sufficiente a riattivare l'ottimizzatore
    duration_threshold = 10; % Soglia minima in secondi
    active_indices = find(trigger_state == 1);
    if isempty(active_indices)
        return;
    end
    
    segments = {};
    seg_start = active_indices(1);
    for i = 2:length(active_indices)
        if active_indices(i) - active_indices(i - 1) > 1
            segments{end+1} = [seg_start, active_indices(i - 1)];
            seg_start = active_indices(i);
        end
    end
    segments{end+1} = [seg_start, active_indices(end)];
    
    for i = 1:length(segments)
        seg = segments{i};
        seg_duration = t(seg(2)) - t(seg(1));
        if seg_duration >= duration_threshold
            fprintf('Veicolo %d: Trigger attivo per %.2f secondi. Riattivazione ottimizzatore.\n', vehicle_id, seg_duration);
            reactivate_optimizer(vehicle_id, seg_duration);
            break;
        end
    end
end

function reactivate_optimizer(vehicle_id, seg_duration)
    % Funzione di riattivazione dell'ottimizzatore
    fprintf('Riattivazione dell''ottimizzatore per il veicolo %d (durata trigger: %.2f s).\n', vehicle_id, seg_duration);
    % Eventuali operazioni aggiuntive possono essere inserite qui.
end


% filepath: complete_code.m
function plot_delta_values()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('Nessun dato disponibile per il plot dei delta.');
        return;
    end
    
    % Seleziona il primo run per il plot
    runData = SIM_RUNS{1};
    t_sim = runData.t;
    n_vehicles = size(runData.x,2) / 2;
    
    figure('Name','Valori per Calcolo Delta', 'Position', [100, 100, 1200, 800]);
    for v = 1:n_vehicles
        % Posizione simulata del veicolo v
        pos_sim = runData.x(:, v);
        
        % Verifica che siano presenti il profilo ottimale
        if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d')
            continue;
        end
        opt_t = runData.opt_t;
        opt_d = runData.opt_d;
        % Interpola il profilo ottimale alla griglia temporale della simulazione
        pos_opt = interp1(opt_t, opt_d, t_sim, 'linear', 'extrap');
        
        % Calcola l'offset come differenza finale tra pos_opt e pos_sim
        offset_value = pos_opt(end) - pos_sim(end);
        % Calcola diff_pos come differenza tra posizioni sottraendo l'offset
        diff_pos = (pos_opt - pos_sim) - offset_value;
        
        % Plotta i dati per il veicolo v in un subplot
        subplot(n_vehicles, 1, v);
        hold on; grid on;
        plot(t_sim, pos_sim, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulazione');
        plot(t_sim, pos_opt, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Ottimale');
        plot(t_sim, diff_pos, 'r-.', 'LineWidth', 1.5, 'DisplayName', 'Delta (calcolato)');
        xlabel('Tempo [s]');
        ylabel('Posizione [m]');
        title(['Veicolo ' num2str(v) ': pos_sim, pos_opt e Delta']);
        legend('Location', 'best');
    end
end

% filepath: complete_code.m
function plot_optimal_trajectories_on_traffic_map()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('Nessun dato da plottare.');
        return;
    end
    
    % Recupera i semafori dal primo run
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    
    % Calcola il tempo massimo registrato da tutti i run
    max_time = 0;
    for i = 1:length(SIM_RUNS)
        max_time = max(max_time, max(SIM_RUNS{i}.opt_t));
    end
    T = 0:ceil(max_time);
    
    % Prepara vettori per il plotting dei semafori
    light_times = [];
    light_positions = [];
    light_colors = [];
    for j = 1:length(traffic_lights)
        d = traffic_lights(j).distance;
        for t = T
            light_times = [light_times, t];
            light_positions = [light_positions, d];
            if is_green(traffic_lights(j), t)
                light_colors = [light_colors; [0, 1, 0]]; % verde
            else
                light_colors = [light_colors; [1, 0, 0]]; % rosso
            end
        end
    end
    
    % Inizializza il plot
    figure('Name','Ottimali sulla Mappa dei Semafori','Position',[100,100,1000,600]);
    hold on;
    scatter(light_times, light_positions, 10, light_colors, 'filled','DisplayName','Semafori');
    
    % Impostazione colori, marker e line style per i vari run
    colors = {'b','r','g','m','c','k',[0.8 0.4 0],[0.5 0.5 0.5]};
    line_styles = {'-', '--', ':', '-.'};
    markers = {'o','s','d','^','v','>','<'};
    
    % Plotta per ogni run i profili ottimali
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d')
            continue;
        end
        opt_t = runData.opt_t;
        opt_d = runData.opt_d;
        color_idx = mod(run_i-1, length(colors)) + 1;
        line_idx = mod(run_i-1, length(line_styles)) + 1;
        marker_idx = mod(run_i-1, length(markers)) + 1;
        
        plot(opt_t, opt_d, [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 2, 'DisplayName', ['Run ' num2str(run_i)]);
        scatter(opt_t, opt_d, 70, colors{color_idx}, markers{marker_idx}, 'filled');
    end
    
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
    title('Profili Ottimali sulla Mappa dei Semafori');
    legend('Location','best');
    grid on;
end

% filepath: complete_code.m
function plot_all_vehicles_on_traffic_map()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('Nessun dato da plottare.');
        return;
    end

    %% Recupera la "mappa" dei semafori dal primo run
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    
    % Calcola il tempo massimo tra i profili ottimali di tutti i run
    max_time = 0;
    for i = 1:length(SIM_RUNS)
        if isfield(SIM_RUNS{i}, 'opt_t')
            max_time = max(max_time, max(SIM_RUNS{i}.opt_t));
        end
    end
    T = 0:ceil(max_time);
    
    % Costruisce i vettori per il plotting dello stato dei semafori
    light_times = [];
    light_positions = [];
    light_colors = [];
    for j = 1:length(traffic_lights)
        d = traffic_lights(j).distance;
        for t = T
            light_times = [light_times, t];
            light_positions = [light_positions, d];
            if is_green(traffic_lights(j), t)
                light_colors = [light_colors; [0, 1, 0]]; % verde
            else
                light_colors = [light_colors; [1, 0, 0]]; % rosso
            end
        end
    end
    
    %% Crea il plot principale
    figure('Name','Traiettorie Ottimali del Plotone sui Semafori','Position',[100,100,1000,600]);
    hold on;
    scatter(light_times, light_positions, 10, light_colors, 'filled','DisplayName','Semafori');
    
    % Imposta opzioni per colori, marker e utilizza solo linea piena ('-')
    colors = {'b','r','g','m','c','k',[0.8 0.4 0],[0.5 0.5 0.5]};
    line_style = '-';  % linea piena
    markers = {'o','s','d','^','v','>','<'};
    
    % Safety gap base per il posizionamento dei follower
    base_gap = 1 + 1.5;  
    
    % Loop su ciascun run (plotone)
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        if ~isfield(runData,'opt_t') || ~isfield(runData,'opt_d')
            continue;
        end
        opt_t = runData.opt_t;
        opt_d = runData.opt_d;
        n_tot = size(runData.x,2) / 2;
        
        % Determina i veicoli da plottare in base al run corrente
        if run_i == 1
            if isfield(runData, 'splittedVehicles') && ~isempty(runData.splittedVehicles)
                vehicles = setdiff(1:n_tot, runData.splittedVehicles);
            else
                vehicles = 1:n_tot;
            end
        elseif run_i == 2
            if isfield(SIM_RUNS{1}, 'splittedVehicles') && ~isempty(SIM_RUNS{1}.splittedVehicles)
                vehicles = SIM_RUNS{1}.splittedVehicles;
            else
                vehicles = 1:n_tot;
            end
        else
            vehicles = 1:n_tot;
        end
        
        % Ordina i veicoli in base alla posizione iniziale per gestire l'offset progressivo
        x_initial = runData.x(1, 1:n_tot);
        [~, ordered_idx] = sort(x_initial, 'descend');
        vehicles = intersect(ordered_idx, vehicles, 'stable');
        
        % Conta per l'applicazione dell'offset progressivo
        follower_count = 0;
        
        for pos = 1:length(vehicles)
            v = vehicles(pos);
            if pos == 1
                offset_value = 0;
            else
                follower_count = follower_count + 1;
                offset_value = base_gap * follower_count;
            end
            veh_traj = opt_d - offset_value;
            
            color_idx = mod(v-1, length(colors)) + 1;
            marker_idx = mod(v-1, length(markers)) + 1;
            
            plot(opt_t, veh_traj, [colors{color_idx} line_style], 'LineWidth', 2, ...
                'DisplayName', ['Run ' num2str(run_i) ', Veicolo ' num2str(v)]);
            scatter(opt_t, veh_traj, 40, colors{color_idx}, markers{marker_idx}, 'filled');
        end
    end
    
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
    title('Traiettorie Ottimali del Plotone sui Semafori');
    legend('Location','best');
    grid on;
end

% filepath: complete_code.m
function plot_delta_velocities()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('Nessun dato disponibile per il plot dei delta delle velocità.');
        return;
    end
    
    % Seleziona il primo run per il plot
    runData = SIM_RUNS{1};
    t_sim = runData.t;
    n_vehicles = size(runData.x, 2) / 2;
    
    figure('Name','Valori per Calcolo Delta Velocità', 'Position', [100, 100, 1200, 800]);
    for v = 1:n_vehicles
        % Velocità simulata del veicolo v (parte seconda dello stato)
        v_sim = runData.x(:, n_vehicles + v);
        
        % Verifica che siano presenti il profilo ottimale
        if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d')
            continue;
        end
        opt_t = runData.opt_t;
        opt_d = runData.opt_d;
        % Interpola il profilo ottimale (posizione) sulla griglia temporale della simulazione
        pos_opt = interp1(opt_t, opt_d, t_sim, 'linear', 'extrap');
        
        % Calcola la velocità ottimale come derivata della posizione ottimale
        v_opt = gradient(pos_opt, t_sim);
        
        % Calcola l'offset come differenza finale tra v_opt e v_sim
        offset_value = v_opt(end) - v_sim(end);
        % Calcola diff_v come differenza tra velocità ottimale e simulata, sottraendo l'offset
        diff_v = (v_opt - v_sim) - offset_value;
        
        % Plotta i dati per il veicolo v in un subplot
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
% filepath: complete_code.m
function trigger_events = velocity_trigger(t, diff_v, opt_v)
    % Meccanismo di trigger basato sui delta di velocità.
    % t      : vettore dei tempi
    % diff_v : differenza tra la velocità ottimale e quella simulata, con offset già rimosso
    % opt_v  : profilo della velocità ottimale (ad es. derivata della posizione ottimale)
    
    trigger_threshold = 5;       % Soglia per attivazione trigger (m/s)
    disable_duration  = 10;      % Durata di disattivazione del trigger (s)
    rapid_change_thresh = 0.5;   % Soglia per variazioni rapide nella velocità (m/s)
    initial_delay = 5;           % Nessun trigger nei primi 5 s
    
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
% filepath: complete_code.m
function plot_velocity_trigger_per_vehicle()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('Nessun dato da plottare per i trigger delle velocità.');
        return;
    end
    
    % Seleziona il primo run per il plot
    runData = SIM_RUNS{1};
    t_sim = runData.t;
    n_vehicles = size(runData.x, 2) / 2;
    
    figure('Name','Trigger per Velocità per Veicolo', 'Position',[100,100,1200,800]);
    for v = 1:n_vehicles
        % Velocità simulata del veicolo v
        v_sim = runData.x(:, n_vehicles + v);
        
        % Verifica che esista il profilo ottimale (in termini di posizione)
        if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d')
            continue;
        end
        opt_t = runData.opt_t;
        opt_d = runData.opt_d;
        % Interpola il profilo ottimale sulla griglia temporale della simulazione
        pos_opt = interp1(opt_t, opt_d, t_sim, 'linear', 'extrap');
        % Calcola la velocità ottimale come derivata della posizione ottimale
        v_opt = gradient(pos_opt, t_sim);
        
        % Calcola offset: differenza finale tra v_opt e v_sim
        offset_value = v_opt(end) - v_sim(end);
        % Calcola diff_v come delta delle velocità (con offset rimosso)
        diff_v = (v_opt - v_sim) - offset_value;
        
        % Calcola il trigger basato sul delta di velocità
        trigger_state = velocity_trigger(t_sim, diff_v, v_opt);
        
        % Plotta i dati per il veicolo v in un subplot
        subplot(n_vehicles, 1, v);
        hold on; grid on;
        plot(t_sim, v_sim, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Velocità Simulata');
        plot(t_sim, v_opt, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Velocità Ottimale');
        plot(t_sim, diff_v, 'r-.', 'LineWidth', 1.5, 'DisplayName', 'Delta Velocità');
        % Il trigger lo moltiplico per un fattore per renderlo visibile sul grafico
        plot(t_sim, trigger_state * (max(diff_v) * 0.8), 'm-', 'LineWidth', 2, 'DisplayName', 'Trigger Attivo');
        xlabel('Tempo [s]');
        ylabel('Velocità [m/s]');
        title(['Veicolo ' num2str(v) ': Vel. Simulata, v_{ottimale} e Delta']);
        legend('Location','best');
    end
end