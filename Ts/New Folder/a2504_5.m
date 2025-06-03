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

%% =========================================================================
%% Funzioni locali
%% =========================================================================

function run_optimizer_and_plot(leader_vehicle, time_offset, varargin)
    global SIM_RUNS N_PLATOON

    % Controllo se ci sono condizioni iniziali personalizzate
    use_custom_init = false;
    if length(varargin) >= 4
        use_custom_init = true;
        init_time = varargin{1};
        init_state = varargin{2};
        init_t_abs = varargin{3};
        violation_idx = varargin{4}; % Indice temporale della violazione
    end
    
    % Pulizia variabili persistent
    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear get_current_v_target_indexed
    
    % Parametri e soglie
    final_time     = 150;         
    final_distance = 1800;    
    T              = 30;                   
    tf             = final_time;
    v_min          = 5;   
    v_max          = 30;  
    b1             = 0.1;  
    b2             = 0.01;
    
    % Stampa debug
    fprintf('\n[INFO] run_optimizer_and_plot(Leader=%d, offset=%.2f)\n', ...
        leader_vehicle, time_offset);

    % Definizione semafori
    traffic_lights = [
        create_traffic_light(300,   0, 10, T)
        create_traffic_light(600,  10, 20, T)
        create_traffic_light(900,  20, 30, T)
        create_traffic_light(1200,  0, 10, T)
        create_traffic_light(1550, 10, 20, T)
    ];

    % Esempio di pruning
    [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance]; 
    nIntersections = length(traffic_lights);
    
    % Se stiamo usando condizioni iniziali personalizzate, ottimizza da quel punto
    if use_custom_init
        % Trova la posizione attuale del leader
        leader_pos = init_state(violation_idx, leader_vehicle);
        
        % Trova il semaforo più vicino al leader che sia davanti a lui
        next_light_idx = find(d > leader_pos, 1);
        if isempty(next_light_idx)
            next_light_idx = length(traffic_lights) + 1;
            next_light_pos = final_distance;
        else
            next_light_pos = d(next_light_idx);
        end
        
        % Inizializza i nodi da questo punto
        Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
        nodeId = 1;
        current_time = init_time;
        Nodes(nodeId) = struct('id', nodeId, 't', current_time, 'd', leader_pos, 'int', next_light_idx-1); 
        nodeId = nodeId + 1;
        
        % Continua con la costruzione dei nodi da qui in poi
        for i = next_light_idx:nIntersections
            light = traffic_lights(i);
            for k = 0:ceil(tf / light.cycle_time)
                cycle_start = k*light.cycle_time + light.offset;
                if light.green_start <= light.green_end
                    abs_green_start = cycle_start + light.green_start;
                    abs_green_end   = cycle_start + light.green_end;
                    if abs_green_start <= tf
                        overlap_start = max(abs_green_start, t_min(i));
                        overlap_end   = min(abs_green_end, t_max(i));
                        if overlap_start < overlap_end && overlap_start > current_time
                            middle_time = ceil((overlap_start+overlap_end)/2);
                            Nodes(nodeId) = struct('id',nodeId,'t',middle_time,'d',d(i),'int',i);
                            nodeId = nodeId + 1;
                        end
                    end
                else
                    abs_green_start_1 = cycle_start + light.green_start;
                    abs_green_end_1   = cycle_start + light.cycle_time;
                    if abs_green_start_1 <= tf
                        ov_start = max(abs_green_start_1, t_min(i));
                        ov_end   = min(abs_green_end_1,   t_max(i));
                        if ov_start < ov_end && ov_start > current_time
                            mid_t = (ov_start+ov_end)/2; 
                            Nodes(nodeId) = struct('id',nodeId,'t',mid_t,'d',d(i),'int',i);
                            nodeId = nodeId + 1;
                        end
                    end
                    abs_green_start_2 = cycle_start;
                    abs_green_end_2   = cycle_start + light.green_end;
                    if abs_green_end_2 <= tf
                        ov_start = max(abs_green_start_2, t_min(i));
                        ov_end   = min(abs_green_end_2,   t_max(i));
                        if ov_start < ov_end && ov_start > current_time
                            mid_t = (ov_start+ov_end)/2;
                            Nodes(nodeId) = struct('id',nodeId,'t',mid_t,'d',d(i),'int',i);
                            nodeId = nodeId + 1;
                        end
                    end
                end
            end
        end
        Nodes(nodeId) = struct('id',nodeId,'t',tf,'d',final_distance,'int',nIntersections+1);
    else
        % Costruzione nodi standard dall'inizio
        Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
        nodeId = 1;
        Nodes(nodeId) = struct('id', nodeId, 't', 0, 'd', 0, 'int', 0); 
        nodeId = nodeId + 1;

        for i = 1:nIntersections
            light = traffic_lights(i);
            for k = 0:ceil(tf / light.cycle_time)
                cycle_start = k*light.cycle_time + light.offset;
                if light.green_start <= light.green_end
                    abs_green_start = cycle_start + light.green_start;
                    abs_green_end   = cycle_start + light.green_end;
                    if abs_green_start <= tf
                        overlap_start = max(abs_green_start, t_min(i));
                        overlap_end   = min(abs_green_end, t_max(i));
                        if overlap_start < overlap_end
                            middle_time = ceil((overlap_start+overlap_end)/2);
                            Nodes(nodeId) = struct('id',nodeId,'t',middle_time,'d',d(i),'int',i);
                            nodeId = nodeId + 1;
                        end
                    end
                else
                    abs_green_start_1 = cycle_start + light.green_start;
                    abs_green_end_1   = cycle_start + light.cycle_time;
                    if abs_green_start_1 <= tf
                        ov_start = max(abs_green_start_1, t_min(i));
                        ov_end   = min(abs_green_end_1,   t_max(i));
                        if ov_start < ov_end
                            mid_t = (ov_start+ov_end)/2; 
                            Nodes(nodeId) = struct('id',nodeId,'t',mid_t,'d',d(i),'int',i);
                            nodeId = nodeId + 1;
                        end
                    end
                    abs_green_start_2 = cycle_start;
                    abs_green_end_2   = cycle_start + light.green_end;
                    if abs_green_end_2 <= tf
                        ov_start = max(abs_green_start_2, t_min(i));
                        ov_end   = min(abs_green_end_2,   t_max(i));
                        if ov_start < ov_end
                            mid_t = (ov_start+ov_end)/2;
                            Nodes(nodeId) = struct('id',nodeId,'t',mid_t,'d',d(i),'int',i);
                            nodeId = nodeId + 1;
                        end
                    end
                end
            end
        end
        Nodes(nodeId) = struct('id',nodeId,'t',tf,'d',final_distance,'int',nIntersections+1);
    end
    nNodes = nodeId;

    % Costruzione archi
    Edges = struct('from',{},'to',{},'w',{});
    edgeCount = 1;
    for i = 1:nNodes
        lvlA = Nodes(i).int;
        for j = 1:nNodes
            lvlB = Nodes(j).int;
            if lvlB == lvlA+1
                if Nodes(j).t > Nodes(i).t && Nodes(j).d > Nodes(i).d
                    if Nodes(j).int > 0 && Nodes(j).int <= nIntersections
                        if ~is_green(traffic_lights(Nodes(j).int), Nodes(j).t)
                            continue;
                        end
                    end
                    delta_t = Nodes(j).t - Nodes(i).t;
                    delta_d = Nodes(j).d - Nodes(i).d;
                    v_link = delta_d/delta_t;
                    if v_link >= v_min && v_link <= v_max
                        E_link = delta_t*(b1*v_link + b2*v_link^2);
                        Edges(edgeCount) = struct('from',Nodes(i).id,'to',Nodes(j).id,'w',E_link);
                        edgeCount = edgeCount + 1;
                    end
                end
            end
        end
    end

    [path, cost] = dijkstra(Nodes, Edges, 1, nNodes);
    fprintf('>>> Leader=%.1f, Costo ottimo=%.3f\n', leader_vehicle, cost);
    % Aggiungi questo codice per gestire il caso di percorso non trovato
if isempty(path)
    disp(['[AVVISO] Non è stato trovato un percorso ottimale per il leader ' num2str(leader_vehicle)]);
    disp('         Utilizzerò una velocità costante per il resto del percorso.');
    
    % Crea un percorso semplice con velocità costante
    opt_t = [0; 150]; % Tempo iniziale e finale
    
    % Se siamo partiti da una violazione, usa la posizione attuale
    if use_custom_init
        opt_d = [init_state(violation_idx, leader_vehicle); final_distance];
    else
        opt_d = [0; final_distance];
    end
    
    speeds = [(opt_d(2) - opt_d(1)) / (opt_t(2) - opt_t(1))];
else
    % Codice esistente per estrarre opt_t, opt_d e calcolare speeds
    opt_nodes = Nodes(path);
    opt_t = arrayfun(@(n)n.t, opt_nodes);
    opt_d = arrayfun(@(n)n.d, opt_nodes);

    speeds = zeros(1, length(path)-1);
    for k = 1:(length(path)-1)
        d_ = opt_nodes(k+1).d - opt_nodes(k).d;
        t_ = opt_nodes(k+1).t - opt_nodes(k).t;
        speeds(k) = d_/t_;
    end
end
    opt_nodes = Nodes(path);
    opt_t = arrayfun(@(n)n.t, opt_nodes);
    opt_d = arrayfun(@(n)n.d, opt_nodes);

    speeds = zeros(1, length(path)-1);
    for k = 1:(length(path)-1)
        d_ = opt_nodes(k+1).d - opt_nodes(k).d;
        t_ = opt_nodes(k+1).t - opt_nodes(k).t;
        speeds(k) = d_/t_;
    end

    n_vehicles = 7;
    m_vehicles = 1000*ones(1,n_vehicles);
    v_targets = speeds;  

    % PID
    K_p_speed = 7000; K_i_speed = 0; K_d_speed = 0.1;
    K_p_dist = 2000;  K_i_dist = 0.8; K_d_dist = 0.4;
    t_CTH = 1.5;  
    d_init = 4;

    if use_custom_init
        % Usa stato iniziale dal punto di violazione
        x0 = init_state(violation_idx, :)';
        t_start = init_time;
        t_span = [t_start tf-t_start];
    else
        % Condizioni iniziali standard
        x0 = zeros(2*n_vehicles,1);
        for i = 1:n_vehicles
            if i == 1, x0(i) = 0;
            else, x0(i) = -d_init*(i-1);
            end
        end
        t_span = [0 150];
    end

    [t_sim, x_sim] = ode45(@(t,x)system_dynamics_new_platoon( ...
        t,x,n_vehicles,m_vehicles,@(tt)0, traffic_lights,v_targets,t_CTH, ...
        K_p_speed,K_i_speed,K_d_speed,K_p_dist,K_i_dist,K_d_dist, ...
        leader_vehicle, time_offset), t_span, x0);

    if use_custom_init
        T_abs = t_sim + init_t_abs(violation_idx);
    else
        T_abs = t_sim + time_offset;
    end
    
    if use_custom_init
    SIM_RUNS{end+1} = struct( ...
        'leader', leader_vehicle, ...
        't', T_abs, ...
        'x', x_sim, ...
        'offset', time_offset, ...
        'traffic_lights', traffic_lights, ...
        'splittedVehicles', [], ...
        'v_targets', speeds, ...
        'opt_t', opt_t + init_t_abs(violation_idx), ...
        'opt_d', opt_d, ...
        'from_violation', use_custom_init, ...
        'violation_time', init_t_abs(violation_idx));
    else
    SIM_RUNS{end+1} = struct( ...
        'leader', leader_vehicle, ...
        't', T_abs, ...
        'x', x_sim, ...
        'offset', time_offset, ...
        'traffic_lights', traffic_lights, ...
        'splittedVehicles', [], ...
        'v_targets', speeds, ...
        'opt_t', opt_t + time_offset, ...
        'opt_d', opt_d, ...
        'from_violation', use_custom_init, ...
        'violation_time', 0);
    end
    check_red_light_violations(T_abs, x_sim, traffic_lights, T);
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

    persistent e_int_speed e_old_speed
    persistent e_int_dist  e_old_dist
    if isempty(e_int_speed), e_int_speed = 0; e_old_speed = 0; end
    if isempty(e_int_dist),  e_int_dist = zeros(n_vehicles, 1); e_old_dist = zeros(n_vehicles, 1); end

    abs_t = t + time_offset;

    for i = 1:n_vehicles
        dx(i) = x(n_vehicles + i);
        if i == leader_vehicle
            vt = get_current_v_target_indexed(x(leader_vehicle), traffic_lights, v_targets);
            vel_err = vt - x(n_vehicles + i);
            e_int_speed = e_int_speed + vel_err * dt;
            vel_deriv = (vel_err - e_old_speed) / dt;
            e_old_speed = vel_err;
            U_leader = K_p_speed*vel_err + K_i_speed*e_int_speed + K_d_speed*vel_deriv;
            dx(n_vehicles + i) = (U_leader + delta_func(abs_t)) / m(i);
            max_speed = 30;
            new_vel = x(n_vehicles + i) + dx(n_vehicles + i)*dt;
            if new_vel < 0, new_vel = 0; end
            if new_vel > max_speed, new_vel = max_speed; end
            dx(n_vehicles + i) = (new_vel - x(n_vehicles + i)) / dt;
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
            U_dist = K_p_dist*dist_err + K_i_dist*e_int_dist(i) + K_d_dist*dist_deriv;
            dx(n_vehicles + i) = U_dist / m(i);
            new_vel = x(n_vehicles + i) + dx(n_vehicles + i)*dt;
            if new_vel < 0, new_vel = 0; end
            dx(n_vehicles + i) = (new_vel - x(n_vehicles + i)) / dt;
        end
    end
end


function vt = get_current_v_target_indexed(x_leader, traffic_lights, v_targets)
    % Se v_targets è vuoto, imposta una velocità di default
    if isempty(v_targets)
        vt = 20; % Velocità target di default (m/s)
        return;
    end
    
    idx = find(x_leader < [traffic_lights.distance], 1);
    if isempty(idx)
        vt = v_targets(end);
    else
        idx = min(idx, length(v_targets));
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
    
    % Controlla ogni veicolo
    for v = 1:n_vehicles
        % Salta i veicoli che non fanno parte di questo plotone
        if ~is_in_current_platoon(v, current_run)
            continue;
        end
        
        pos_v = x_sim(:, v);
        for L = 1:length(traffic_lights)
            light_d = traffic_lights(L).distance;
            cross_idx = find(pos_v(1:end-1) < light_d & pos_v(2:end) >= light_d, 1);
            if ~isempty(cross_idx)
                cross_time = t_abs(cross_idx);
                if ~is_green(traffic_lights(L), cross_time)
                    % Verifica se questa violazione è già stata rilevata
                    if any(violations_detected == v)
                        continue;
                    end
                    
                    fprintf('\n[WARNING] Veicolo %d passa col rosso a incrocio %d (t=%.2f s)\n', v, L, cross_time);
                    
                    % Salva questa violazione per non rilevarla di nuovo
                    violations_detected = [violations_detected, v];
                    
                    fprintf('>> Veicolo %d diventa leader di un nuovo plotone dal punto di violazione!\n', v);

                    % Aggiorna i veicoli nel plotone corrente
                    SIM_RUNS{current_run}.splittedVehicles = v:n_vehicles;
                    
                    % Salva anche l'indice della violazione
                    SIM_RUNS{current_run}.violation_idx = cross_idx;
                    SIM_RUNS{current_run}.violation_time = cross_time;

                    % Rilancia la simulazione con il nuovo leader dal punto di violazione
                    t_local = t_abs(cross_idx) - SIM_RUNS{current_run}.offset;
                    rerun_optimizer_from_violation(v, T, t_local, x_sim, t_abs, cross_idx);
                    return;
                end
            end
        end
    end
end

function rerun_optimizer_from_violation(violating_vehicle, T, violation_time, x_state, t_abs, violation_idx)
    clear system_dynamics_new_platoon
    clear get_current_v_target_indexed
    
    global SIM_RUNS
    global N_PLATOON
    
    % Mostra informazioni sulla creazione del nuovo plotone
    current_platoon = N_PLATOON + 1;
    disp(['[INFO] Creazione NUOVO PLOTONE ' num2str(current_platoon) ' con LEADER=' ...
          num2str(violating_vehicle) ', dal punto di violazione (t=' ...
          num2str(t_abs(violation_idx)) ')']);

    % Incrementa il contatore di plotoni
    N_PLATOON = N_PLATOON + 1;
    
    % Determina l'offset temporale (utilizziamo il tempo attuale)
    start_offset = 0;  % Non serve un offset aggiuntivo dato che stiamo usando il tempo assoluto
    
    % Esegui la simulazione con il nuovo leader dal punto di violazione
    run_optimizer_and_plot(violating_vehicle, start_offset, ...
                          violation_time, x_state, t_abs, violation_idx);
    
    % Pulisci lo stato delle violazioni per il nuovo plotone
    clear check_red_light_violations
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

function final_plot()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[final_plot] Nessun dato da plottare.');
        return;
    end
    
    % Creazione dei singoli grafici
    plot_optimal_trajectories_and_lights();
    plot_real_trajectories();
    %plot_comparison();
    plot_leader_velocities();
    plot_energy_consumption();
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
    if length(opt_t) > 1
        v_targets = calculate_target_velocities(opt_t, opt_d);
    else
        v_targets = []; % Caso di traiettoria vuota o singolo punto
    end
    
    % Disegna la traiettoria del leader
    color_idx = mod(leader-1, length(colors))+1;
    line_idx = mod(run_i-1, length(line_styles))+1;
    marker_idx = mod(run_i-1, length(markers))+1;
    
    % Gestione sicura per il plottaggio - verifica che ci siano abbastanza punti
    if length(opt_t) >= 2
        h = plot(opt_t, opt_d, [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 3);
        scatter(opt_t, opt_d, 70, colors{color_idx}, markers{marker_idx}, 'filled');
        
        % Prendi solo il primo handle se è un array
        if length(h) > 1
            h = h(1);
        end
        
        legend_handles(end+1) = h;
        legend_texts{end+1} = ['Leader ' num2str(leader) ' (Plotone ' num2str(run_i) ')'];
        
        % Disegna le traiettorie ottimali dei follower solo se abbiamo velocità target
        if ~isempty(v_targets)
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
                    
                    % Prendi solo il primo handle se è un array
                    if length(h_follower) > 1
                        h_follower = h_follower(1);
                    end
                    
                    legend_handles(end+1) = h_follower;
                    legend_texts{end+1} = ['Follower ' num2str(v) ' (Plotone ' num2str(run_i) ')'];
                end
            end
        end
    else
        % Se ci sono meno di 2 punti, non possiamo disegnare una linea
        disp(['[AVVISO] Non abbastanza punti per disegnare la traiettoria ottimale del plotone ' num2str(run_i)]);
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
    
    % Combinazione delle traiettorie complete
    % Per ogni veicolo, combina le sue traiettorie da tutti i plotoni pertinenti
    n_vehicles = size(SIM_RUNS{1}.x, 2)/2;
    
    for v = 1:n_vehicles
        % Raccogli tutte le porzioni di traiettoria per questo veicolo
        all_times = [];
        all_positions = [];
        all_run_labels = [];
        
        for run_i = 1:length(SIM_RUNS)
            runData = SIM_RUNS{run_i};
            is_leader = (v == runData.leader);
            
            % Verifica se il veicolo è in questo plotone e quando
            if run_i == 1
                % Nel primo plotone, tutti i veicoli iniziano insieme
                if ~isfield(runData, 'violation_time') || isempty(runData.violation_time)
                    % Nessuna violazione registrata, usa tutta la traiettoria
                    all_times = [all_times; runData.t];
                    all_positions = [all_positions; runData.x(:, v)];
                    all_run_labels = [all_run_labels; run_i*ones(size(runData.t))];
                else
                    % Usa la traiettoria fino al punto di violazione
                    violation_idx = runData.violation_idx;
                    if ismember(v, runData.splittedVehicles)
                        all_times = [all_times; runData.t(1:violation_idx)];
                        all_positions = [all_positions; runData.x(1:violation_idx, v)];
                        all_run_labels = [all_run_labels; run_i*ones(size(runData.t(1:violation_idx)))];
                    else
                        all_times = [all_times; runData.t];
                        all_positions = [all_positions; runData.x(:, v)];
                        all_run_labels = [all_run_labels; run_i*ones(size(runData.t))];
                    end
                end
            else
                % Nei plotoni successivi, verifica se il veicolo è stato splittato dal plotone precedente
                prev_run = SIM_RUNS{run_i-1};
                if isfield(prev_run, 'splittedVehicles') && ismember(v, prev_run.splittedVehicles)
                    % Questo veicolo è stato diviso nel plotone corrente
                    if isfield(runData, 'from_violation') && runData.from_violation
                        % La traiettoria parte dal punto di violazione
                        all_times = [all_times; runData.t];
                        all_positions = [all_positions; runData.x(:, v)];
                        all_run_labels = [all_run_labels; run_i*ones(size(runData.t))];
                    end
                end
            end
        end
        
        % Ordina i dati per tempo crescente
        [sorted_times, idx] = sort(all_times);
        sorted_positions = all_positions(idx);
        sorted_run_labels = all_run_labels(idx);
        
        % Disegna la traiettoria completa
        if ~isempty(sorted_times)
            color_idx = mod(v-1, length(colors))+1;
            h = plot(sorted_times, sorted_positions, [colors{color_idx}, '-'], 'LineWidth', 2);
            
            % Aggiungi punti di cambio plotone
            run_changes = find(diff(sorted_run_labels) ~= 0);
            if ~isempty(run_changes)
                for change_i = 1:length(run_changes)
                    change_idx = run_changes(change_i);
                    scatter(sorted_times(change_idx), sorted_positions(change_idx), 100, 'k', 'x', 'LineWidth', 2);
                end
            end
            
            % Determina se è un leader in qualsiasi plotone
            is_leader_anywhere = false;
            leader_of_platoons = [];
            for run_i = 1:length(SIM_RUNS)
                if SIM_RUNS{run_i}.leader == v
                    is_leader_anywhere = true;
                    leader_of_platoons = [leader_of_platoons, run_i];
                end
            end
            
            if is_leader_anywhere
                legend_texts{end+1} = ['Veicolo ' num2str(v) ' (Leader nei plotoni ' num2str(leader_of_platoons) ')'];
            else
                legend_texts{end+1} = ['Veicolo ' num2str(v)];
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
    
    % Combina le traiettorie reali e ottimali per ogni veicolo
    n_vehicles = size(SIM_RUNS{1}.x, 2)/2;
    
    % Prima disegna tutte le traiettorie reali
    for v = 1:n_vehicles
        % Raccogli tutte le porzioni di traiettoria reale
        all_times = [];
        all_positions = [];
        all_run_labels = [];
        
        for run_i = 1:length(SIM_RUNS)
            runData = SIM_RUNS{run_i};
            is_leader = (v == runData.leader);
            
            % Verifica se il veicolo è in questo plotone e quando
            if run_i == 1
                % Nel primo plotone, tutti i veicoli iniziano insieme
                if ~isfield(runData, 'violation_time') || isempty(runData.violation_time)
                    % Nessuna violazione registrata, usa tutta la traiettoria
                    all_times = [all_times; runData.t];
                    all_positions = [all_positions; runData.x(:, v)];
                    all_run_labels = [all_run_labels; run_i*ones(size(runData.t))];
                else
                    % Usa la traiettoria fino al punto di violazione
                    violation_idx = runData.violation_idx;
                    if ismember(v, runData.splittedVehicles)
                        all_times = [all_times; runData.t(1:violation_idx)];
                        all_positions = [all_positions; runData.x(1:violation_idx, v)];
                        all_run_labels = [all_run_labels; run_i*ones(size(runData.t(1:violation_idx)))];
                    else
                        all_times = [all_times; runData.t];
                        all_positions = [all_positions; runData.x(:, v)];
                        all_run_labels = [all_run_labels; run_i*ones(size(runData.t))];
                    end
                end
            else
                % Nei plotoni successivi, verifica se il veicolo è stato splittato dal plotone precedente
                prev_run = SIM_RUNS{run_i-1};
                if isfield(prev_run, 'splittedVehicles') && ismember(v, prev_run.splittedVehicles)
                    % Questo veicolo è stato diviso nel plotone corrente
                    if isfield(runData, 'from_violation') && runData.from_violation
                        % La traiettoria parte dal punto di violazione
                        all_times = [all_times; runData.t];
                        all_positions = [all_positions; runData.x(:, v)];
                        all_run_labels = [all_run_labels; run_i*ones(size(runData.t))];
                    end
                end
            end
        end
        
        % Ordina i dati per tempo crescente
        [sorted_times, idx] = sort(all_times);
        sorted_positions = all_positions(idx);
        sorted_run_labels = all_run_labels(idx);
        
        % Disegna la traiettoria completa
        if ~isempty(sorted_times)
            color_idx = mod(v-1, length(colors_real))+1;
            h_real = plot(sorted_times, sorted_positions, [colors_real{color_idx}, '-'], 'LineWidth', 2);
            
            % Determina se è un leader in qualsiasi plotone
            is_leader_anywhere = false;
            leader_of_platoons = [];
            for run_i = 1:length(SIM_RUNS)
                if SIM_RUNS{run_i}.leader == v
                    is_leader_anywhere = true;
                    leader_of_platoons = [leader_of_platoons, run_i];
                end
            end
            
            if is_leader_anywhere
                legend_texts{end+1} = ['Veicolo ' num2str(v) ' (reale, Leader nei plotoni ' num2str(leader_of_platoons) ')'];
            else
                legend_texts{end+1} = ['Veicolo ' num2str(v) ' (reale)'];
            end
            legend_handles(end+1) = h_real;
        end
    end
    
    % Poi aggiungi le traiettorie ottimali
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
            opt_t = runData.opt_t;
            opt_d = runData.opt_d;
            leader = runData.leader;
            v_targets = calculate_target_velocities(opt_t, opt_d);
            
            % Disegna la traiettoria ottimale del leader
            color_idx = mod(leader-1, length(colors_opt))+1;
            h_opt = plot(opt_t, opt_d, [colors_opt{color_idx}, '--'], 'LineWidth', 2);
            legend_texts{end+1} = ['Leader ' num2str(leader) ' (ottimale, Plotone ' num2str(run_i) ')'];
            legend_handles(end+1) = h_opt;
            
            % Disegna traiettorie ottimali dei follower
            platoon_vehicles = get_platoon_vehicles(run_i);
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
                    
                    % Usa un approccio sicuro per plottare le velocità target
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
        labels = [];
        
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
        legend(labels, 'Location', 'Best');
        grid on;
        ylim([0, max(max(distances), 10)]);
    end
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

function st=is_green(light, time)
    t_in_cycle= mod(time - light.offset, light.cycle_time);
    if light.green_start <= light.green_end
        st= (t_in_cycle >= light.green_start && t_in_cycle < light.green_end);
    else
        st= (t_in_cycle >= light.green_start || t_in_cycle < light.green_end);
    end
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

function t_next=next_green(light, t)
    if is_green(light,t), t_next=t; return; end
    cyc= mod(t- light.offset, light.cycle_time);
    if cyc< light.green_start
        t_next= t+(light.green_start- cyc);
    else
        t_next= t+(light.cycle_time- cyc)+ light.green_start;
    end
end

function t_prev=prev_green(light, t)
    if is_green(light,t), t_prev=t; return; end
    cyc= mod(t- light.offset, light.cycle_time);
    if cyc>= light.green_end
        t_prev= t-(cyc- light.green_end);
    else
        t_prev= t- cyc- (light.cycle_time- light.green_end);
    end
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