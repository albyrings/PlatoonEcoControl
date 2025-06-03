% filepath: complete_code_with_triggers.m
% Script principale con sistema di trigger per riottimizzazione basata su delta velocità
% e previsione di violazioni dei semafori

%% Inizializzazione
clear;
clc; 
close all;

%% Pulizia delle cache e reset delle variabili
clearAllMemoizedCaches(); 
reset_persistent_variables();

%% Variabili globali
global SIM_RUNS;    % Contiene i dati delle simulazioni
global N_PLATOON;   % Conta il numero di plotoni

SIM_RUNS = {};
N_PLATOON = 1;

%% Avvio simulazione iniziale
disp('=== Avvio prima simulazione con Leader=1 ===');
run_optimizer_and_plot(1, 0);   % Esegue la simulazione con leader = 1 e offset = 0

%% Plot principali
final_plot();
plot_delta_velocities();
plot_velocity_trigger_per_vehicle();

%% --------------------------------------------------------------------
%% DEFINIZIONI DELLE FUNZIONI
%% --------------------------------------------------------------------

%% Funzione che pulisce tutte le cache memorizzate
function clearAllMemoizedCaches()
    % Pulisce tutte le cache memorizzate nelle funzioni del progetto
    % Questa funzione dovrebbe essere chiamata quando si vuole ricominciare
    % una simulazione da zero, eliminando tutti i dati persistenti
    
    % Reset di tutte le funzioni che utilizzano variabili persistenti
    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear get_current_v_target_indexed
    clear velocity_trigger
    clear next_green
    clear prev_green
    clear dijkstra
    
    % Pulisce anche le cache di MATLAB per interpolazione e altre funzioni
    clear interp1
    clear gradient
    
    % Reimposta generatore numeri casuali per riproducibilità
    rng('default');
    
    disp('Tutte le cache memorizzate sono state pulite.');
end

%% Reset delle variabili persistenti
function reset_persistent_variables()
    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear get_current_v_target_indexed
    clear next_green
    clear prev_green
    clear dijkstra
    disp('Persistent variables reset.');
end

%% Funzione principale di ottimizzazione e simulazione
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
    b3 = 10;
    b4 = 4;
    delta_func = @(t) 30000 * (b1 + b2*(sin((1/b3)*t+b4) + 0.25*rand));

    fprintf('\n[INFO] run_optimizer_and_plot(Leader=%d, offset=%.2f)\n', leader_vehicle, time_offset);

    %% Creazione dei semafori
    T_cycle = T;
    traffic_lights = [ ...
        create_traffic_light(300,   0, 10, T_cycle); 
        create_traffic_light(600,  10, 20, T_cycle); 
        create_traffic_light(900,  20, 30, T_cycle); 
        create_traffic_light(1200,  0, 10, T_cycle); 
        create_traffic_light(1550, 10, 20, T_cycle)  
        ];

    % Calcola i tempi minimi e massimi per passare i semafori
    [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance]; 
    nIntersections = length(traffic_lights);

    %% Costruzione dei nodi per la pianificazione
    Nodes = struct('id', num2cell(1), 't', {0}, 'd', {0}, 'int', {0});
    nodeId = 1;
    Nodes(1) = struct('id', 1, 't', 0, 'd', 0, 'int', 0);
    nodeId = nodeId + 1;
    for i = 1:nIntersections
        light = traffic_lights(i);
        nCycles = ceil(tf / light.cycle_time);
        for j = 0:nCycles
            cycle_start = j * light.cycle_time + light.offset;
            % Caso verde "classico"
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

    %% Creazione degli archi
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
    
    % Stampa velocità medie
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

    % Parametri PID
    K_p_speed = 7000; K_i_speed = 0; K_d_speed = 0.1;
    K_p_dist  = 2000; K_i_dist  = 0.8; K_d_dist  = 0.4;
    t_CTH = 1.5;  
    d_init = 4;

    % Condizioni iniziali separate da d_init
    x0 = zeros(2 * n_vehicles, 1);
    for i = 1:n_vehicles
        if i == 1
            x0(i) = 0;
        else
            x0(i) = -d_init * (i - 1);
        end
    end

    t_span = [0 150];
    [t_sim, x_sim] = ode45(@(t, x) system_dynamics_new_platoon(t, x, n_vehicles, m_vehicles, delta_func, traffic_lights, v_targets, t_CTH, ...
                                K_p_speed, K_i_speed, K_d_speed, K_p_dist, K_i_dist, K_d_dist, leader_vehicle, time_offset),...
                                t_span, x0);

    % Salvataggio risultati
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

    % Verifica semafori tradizionale
    check_red_light_violations(T_abs, x_sim, traffic_lights, T_cycle);
    
    % Verifica trigger velocità e anticipa eventuali problemi
    check_velocity_triggers_and_reoptimize(leader_vehicle, T_abs, x_sim, traffic_lights, T_cycle);
end

%% Dinamica del sistema per il plotone
function dx = system_dynamics_new_platoon(t, x, n_vehicles, m, delta_func, traffic_lights, v_targets, t_CTH, ...
                                           K_p_speed, K_i_speed, K_d_speed, K_p_dist, K_i_dist, K_d_dist, ...
                                           leader_vehicle, time_offset)
    dx = zeros(2 * n_vehicles, 1);
    
    persistent t_prev
    if isempty(t_prev), t_prev = t; end
    dt = t - t_prev;
    if dt <= 0, dt = 1e-5; end  
    t_prev = t;

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
        dx(i) = x(n_vehicles + i);
        if i == leader_vehicle
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

%% Funzione per ottenere la velocità target per il leader
function vt = get_current_v_target_indexed(x_leader, traffic_lights, v_targets)
    idx = find(x_leader < [traffic_lights.distance], 1);
    if isempty(idx)
        vt = v_targets(end);
    else
        vt = v_targets(min(idx, length(v_targets)));
    end
end

%% Controlla violazioni semaforo rosso (meccanismo tradizionale)
function check_red_light_violations(t_abs, x_sim, traffic_lights, T)
    persistent new_leader_detected
    if isempty(new_leader_detected), new_leader_detected = false; end

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

%% Verifica trigger velocità e anticipa problemi (meccanismo nuovo)
function check_velocity_triggers_and_reoptimize(leader_vehicle, t_abs, x_sim, traffic_lights, T_cycle)
    % Questa funzione controlla i trigger di velocità e avvia una nuova
    % ottimizzazione quando necessario, dividendo il plotone
    global SIM_RUNS
    global N_PLATOON
    
    % Ottiene l'indice del run corrente
    current_run_idx = length(SIM_RUNS);
    runData = SIM_RUNS{current_run_idx};
    
    n_vehicles = size(x_sim, 2) / 2;
    
    % Calcolo dei dati necessari per il trigger
    if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d')
        disp('[check_velocity_triggers] Profili ottimali mancanti, impossibile procedere.');
        return;
    end
    
    opt_t = runData.opt_t;
    opt_d = runData.opt_d;
    abs_time_offset = runData.offset;
    
    % Analizziamo veicolo per veicolo
    for v = 1:n_vehicles
        pos_v = x_sim(:, v);
        vel_v = x_sim(:, n_vehicles + v);
        
        % Se questo veicolo è il leader, controlliamo solo i semafori
        if v == leader_vehicle
            continue; % Il leader segue già la traiettoria ottimale
        end
        
        % Calcola i dati per il trigger
        t_sim = t_abs - abs_time_offset; % Tempo relativo alla simulazione
        pos_opt = interp1(opt_t, opt_d, t_abs, 'linear', 'extrap');
        v_opt = gradient(pos_opt, t_abs);
        offset_value = v_opt(end) - vel_v(end);
        diff_v = (v_opt - vel_v) - offset_value;
        
        % Calcola lo stato del trigger
        trigger_state = velocity_trigger(t_sim, diff_v, v_opt, pos_v, vel_v, traffic_lights, abs_time_offset);
        
        % Trova il primo indice in cui il trigger si attiva (se c'è)
        trigger_idx = find(trigger_state == 1, 1);
        
        if ~isempty(trigger_idx)
            % Trigger attivato! Dobbiamo dividere il plotone
            trigger_time = t_abs(trigger_idx);
            fprintf('\n[INFO] Trigger attivato per veicolo %d al tempo %.2f\n', v, trigger_time);
            
            % Se questo veicolo non è già stato scelto come nuovo leader
            if ~ismember(v, runData.splittedVehicles)
                fprintf('>> Veicolo %d diventa leader di un nuovo plotone a causa del trigger!\n', v);
                
                % Aggiorna il record dei veicoli splittati
                SIM_RUNS{current_run_idx}.splittedVehicles = v : n_vehicles;
                
                % Avvia una nuova ottimizzazione con questo veicolo come leader
                start_offset = N_PLATOON * T_cycle;
                N_PLATOON = N_PLATOON + 1;
                
                % Resetta le variabili persistenti per sicurezza
                clear velocity_trigger
                clear system_dynamics_new_platoon
                
                % Avvia la nuova ottimizzazione
                disp(['[INFO] Riottimizzazione con NUOVO LEADER = ', num2str(v), ...
                      ', tempo trigger = ', num2str(trigger_time), ...
                      ', ripartenza da tempo assoluto = ', num2str(start_offset)]);
                
                %run_optimizer_and_plot(v, start_offset);
                return; % Usciamo dopo aver avviato la prima riottimizzazione
            end
        end
    end
end

%% Riavvia l'ottimizzazione con un nuovo leader (chiamata da check_red_light_violations)
function rerun_optimizer_for_new_leader(violating_vehicle, T)
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

%% Funzione di plot finale delle traiettorie
function final_plot()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[final_plot] Nessun dato da plottare.');
        return;
    end

    %% FIGURA 1: Traiettorie ottimali e stato dei semafori
    figure('Name','Posizioni Ottimali e Semafori', 'Position', [100, 100, 1000, 600]);
    hold on;
    
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    
    % Preallocazione per vettori semafori (evita concatenazioni ripetute)
    max_time = 0;
    for j = 1:length(SIM_RUNS)
        max_time = max(max_time, max(SIM_RUNS{j}.t));
    end
    times = 0:ceil(max_time);
    N = length(times) * length(traffic_lights);
    all_times = zeros(1, N);
    all_distances = zeros(1, N);
    all_colors = zeros(N, 3);
    cnt = 1;
    for i = 1:length(traffic_lights)
        for j = 1:length(times)
            all_times(cnt) = times(j);
            all_distances(cnt) = traffic_lights(i).distance;
            if is_green(traffic_lights(i), times(j))
                all_colors(cnt, :) = [0, 1, 0];
            else
                all_colors(cnt, :) = [1, 0, 0];
            end
            cnt = cnt + 1;
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
            
            % Calcola velocità target tra i nodi ottimali (per completezza)
            v_targets = zeros(1, length(opt_t)-1);
            for j = 1:length(v_targets)
                v_targets(j) = (opt_d(j+1) - opt_d(j)) / (opt_t(j+1) - opt_t(j));
            end
            
            color_idx = mod(leader - 1, length(colors)) + 1;
            line_idx  = mod(run_i - 1, length(line_styles)) + 1;
            marker_idx = mod(run_i - 1, length(markers)) + 1;
            h = plot(opt_t, opt_d, [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 3);
            scatter(opt_t, opt_d, 70, colors{color_idx}, markers{marker_idx}, 'filled');
            legend_handles(end+1) = h;
            legend_texts{end+1} = ['Leader ' num2str(leader) ' (Plotone ' num2str(run_i) ')'];
            
            % Plot dei follower con gap fisso
            if isempty(platoon_vehicles)
                ordered_vehicles = leader;
            else
                x_initial = runData.x(1, 1:n_vehicles);
                [~, idx] = sort(x_initial, 'descend');
                ordered_vehicles = intersect(idx, platoon_vehicles, 'stable');
            end
            
            for vidx = 1:length(ordered_vehicles)
                v = ordered_vehicles(vidx);
                if v == leader, continue; end
                follower_opt_t = opt_t;
                follower_opt_d = zeros(size(opt_d));
                for j = 1:length(opt_t)
                    current_pos = opt_d(j);
                    safety_gap = 1 + 1.5;
                    follower_opt_d(j) = current_pos - safety_gap;
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
    
    legend(legend_handles, legend_texts, 'Location','Best');
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
    title('Traiettorie ottimali dei veicoli e stato semafori');
    grid on;
    
    %% FIGURA 2: Traiettorie reali
    figure('Name','Grafico Traiettorie Reali', 'Position', [150, 150, 1000, 600]);
    hold on;
    scatter(all_times, all_distances, 10, all_colors, 'filled');
    
    colorsReal = {'b', 'r', 'g', 'm', 'c', 'y', 'k'};
    line_stylesReal = {'-', '-', ':', '-.'};
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
            color_idx = mod(v - 1, length(colorsReal)) + 1;
            line_idx = mod(run_i - 1, length(line_stylesReal)) + 1;
            plot(t, x(:, v), [colorsReal{color_idx}, line_stylesReal{line_idx}], 'LineWidth', 2);
            plotted_vehicles = [plotted_vehicles, v];
        end
    end
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
    title('Traiettorie reali dei veicoli e stato semafori');
    grid on;
end

%% Plot dei delta delle velocità
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

%% Plot dei trigger di velocità per veicolo
function plot_velocity_trigger_per_vehicle()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('Nessun dato da plottare per i trigger delle velocità.');
        return;
    end
    
    % Determina quali veicoli sono in quali plotoni
    platoon_assignments = cell(length(SIM_RUNS), 1);
    n_vehicles = size(SIM_RUNS{1}.x, 2) / 2;
    
    % Assegna i veicoli al primo plotone di default
    platoon_assignments{1} = 1:n_vehicles;
    
    % Aggiorna le assegnazioni in base ai veicoli divisi
    for run_i = 1:length(SIM_RUNS)-1
        if isfield(SIM_RUNS{run_i}, 'splittedVehicles') && ~isempty(SIM_RUNS{run_i}.splittedVehicles)
            platoon_assignments{run_i} = setdiff(platoon_assignments{run_i}, SIM_RUNS{run_i}.splittedVehicles);
            platoon_assignments{run_i+1} = SIM_RUNS{run_i}.splittedVehicles;
        end
    end
    
    % Crea una figura per ogni veicolo
    figure('Name','Trigger per Velocità per Veicolo', 'Position',[100,100,1200,800]);
    for v = 1:n_vehicles
        % Determina a quale plotone appartiene questo veicolo
        platoon_idx = 0;
        for p = 1:length(platoon_assignments)
            if ismember(v, platoon_assignments{p})
                platoon_idx = p;
                break;
            end
        end
        
        if platoon_idx == 0
            disp(['Veicolo ' num2str(v) ' non assegnato a nessun plotone, salto.']);
            continue;
        end
        
        % Usa i dati del plotone corretto
        runData = SIM_RUNS{platoon_idx};
        t_sim = runData.t;
        x_sim = runData.x;
        traffic_lights = runData.traffic_lights;
        abs_time_offset = runData.offset;
        
        % Verifica che il veicolo sia presente in questo dataset
        if v > size(x_sim, 2) / 2
            disp(['Veicolo ' num2str(v) ' non presente nel dataset del plotone ' num2str(platoon_idx) ', salto.']);
            continue;
        end
        
        % Posizione e velocità del veicolo
        pos_sim = x_sim(:, v);
        v_sim = x_sim(:, n_vehicles + v);
        
        if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d')
            continue;
        end
        
        opt_t = runData.opt_t;
        opt_d = runData.opt_d;
        pos_opt = interp1(opt_t, opt_d, t_sim, 'linear', 'extrap');
        v_opt = gradient(pos_opt, t_sim);
        offset_value = v_opt(end) - v_sim(end);
        diff_v = (v_opt - v_sim) - offset_value;
        
        % Usa la versione del trigger con controllo semaforo rosso
        trigger_state = velocity_trigger(t_sim, diff_v, v_opt, pos_sim, v_sim, traffic_lights, abs_time_offset);
        
        subplot(n_vehicles, 1, v);
        hold on; grid on;
        plot(t_sim, v_sim, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Velocità Simulata');
        plot(t_sim, v_opt, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Velocità Ottimale');
        plot(t_sim, diff_v, 'r-.', 'LineWidth', 1.5, 'DisplayName', 'Delta Velocità');
        
        % Visualizza il trigger con un fattore di scala
        max_value = max(max(v_sim), max(v_opt));
        plot(t_sim, trigger_state * max_value * 0.8, 'm-', 'LineWidth', 2, 'DisplayName', 'Trigger Attivo');
        
        % Aggiungi indicatori visivi per i semafori e il loro stato
        for lt = 1:length(traffic_lights)
            % Trova quando il veicolo attraversa il semaforo
            cross_idx = find(pos_sim(1:end-1) < traffic_lights(lt).distance & pos_sim(2:end) >= traffic_lights(lt).distance, 1);
            if ~isempty(cross_idx)
                cross_time = t_sim(cross_idx) + abs_time_offset;
                if is_green(traffic_lights(lt), cross_time)
                    xline(t_sim(cross_idx), '--g', sprintf('Semaforo %d (VERDE)', lt));
                else
                    xline(t_sim(cross_idx), '--r', sprintf('Semaforo %d (ROSSO!)', lt));
                end
            end
        end
        
        % Se c'è un trigger attivo, mostra dove
        trigger_idx = find(trigger_state == 1, 1);
        if ~isempty(trigger_idx)
            xline(t_sim(trigger_idx), '-m', 'Riottimizzazione');
        end
        
        xlabel('Tempo [s]');
        ylabel('Velocità [m/s]');
        title(['Veicolo ' num2str(v) ' (Plotone ' num2str(platoon_idx) '): Velocità e Trigger']);
        legend('Location','best');
    end
end

%% Funzione di trigger basata su delta velocità e previsione semaforo rosso
function trigger_events = velocity_trigger(t, diff_v, opt_v, pos_v, vel_v, traffic_lights, abs_time_offset)
    % Meccanismo di trigger basato sui delta di velocità e previsione di semaforo rosso.
    % t               : vettore dei tempi
    % diff_v          : differenza tra la velocità ottimale e quella simulata
    % opt_v           : profilo della velocità ottimale 
    % pos_v           : posizione attuale del veicolo
    % vel_v           : velocità attuale del veicolo
    % traffic_lights  : array di strutture con i semafori
    % abs_time_offset : offset temporale assoluto (per calcolare il tempo corretto)
    
    trigger_threshold = 5;       % Soglia per attivazione trigger (m/s)
    disable_duration  = 10;      % Durata di disattivazione del trigger (s)
    rapid_change_thresh = 0.5;   % Soglia per variazioni rapide nella velocità (m/s)
    initial_delay = 5;           % Nessun trigger nei primi 5 s
    look_ahead_time = 5;         % Secondi di "previsione" per semaforo rosso
    
    trigger_events = zeros(size(t));
    disable_until = -inf;
    
    for i = 1:length(t)
        % Trigger standard basato su delta velocità
        if t(i) < initial_delay || t(i) < disable_until
            trigger_active = false;
        else
            trigger_active = abs(diff_v(i)) > trigger_threshold;
        end
        
        % Trigger basato su previsione semaforo rosso
        red_light_trigger = false;
        if i < length(t) && vel_v(i) > 0.5  % Se il veicolo è in movimento
            % Posizione prevista nei prossimi look_ahead_time secondi
            predicted_pos = pos_v(i) + vel_v(i) * look_ahead_time;
            
            % Controlla se c'è un semaforo nel raggio di previsione
            for light_idx = 1:length(traffic_lights)
                light = traffic_lights(light_idx);
                
                % Se il veicolo sta per attraversare questo semaforo
                if pos_v(i) < light.distance && predicted_pos >= light.distance
                    % Stima il tempo di attraversamento
                    time_to_crossing = (light.distance - pos_v(i)) / vel_v(i);
                    crossing_time = t(i) + time_to_crossing + abs_time_offset;
                    
                    % Verifica se il semaforo sarà rosso
                    if ~is_green(light, crossing_time) && time_to_crossing < look_ahead_time
                        red_light_trigger = true;
                        fprintf('Trigger Preventivo: Veicolo in arrivo a semaforo rosso (t=%0.2f, attraversamento a t=%0.2f)\n', ...
                               t(i) + abs_time_offset, crossing_time);
                        break;
                    end
                end
            end
        end
        
        % Attiva il trigger se una delle condizioni è vera
        trigger_events(i) = trigger_active || red_light_trigger;
        
        % Gestione della disabilitazione temporanea
        if i > 1
            delta_opt = abs(opt_v(i) - opt_v(i-1));
            if delta_opt > rapid_change_thresh
                disable_until = t(i) + disable_duration;
            end
        end
    end
end

%% Funzione per creare un semaforo
function light = create_traffic_light(distance, green_start, green_end, cycle_time)
    light.distance      = distance;
    light.green_start   = green_start;
    light.green_end     = green_end;
    light.cycle_time    = cycle_time;
    light.green_duration= green_end - green_start;
    light.offset        = mod(green_start, cycle_time);
end

%% Funzione per verificare se un semaforo è verde a un determinato tempo
function st = is_green(light, time)
    t_in_cycle = mod(time - light.offset, light.cycle_time);
    if light.green_start <= light.green_end
        st = (t_in_cycle >= light.green_start && t_in_cycle < light.green_end);
    else
        st = (t_in_cycle >= light.green_start || t_in_cycle < light.green_end);
    end
end

%% Funzione per trovare il prossimo tempo di verde
function t_next = next_green(light, t)
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

%% Funzione per trovare il precedente tempo di verde
function t_prev = prev_green(light, t)
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

%% Algoritmo di Dijkstra
function [path, cost] = dijkstra(Nodes, Edges, source, target)
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

%% Funzione per potatura velocità tra semafori
function [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max)
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