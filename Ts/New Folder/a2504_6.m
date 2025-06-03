
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
        violation_time = init_t_abs(violation_idx); % Definizione necessaria
    else
        violation_time = 0; % Valore predefinito quando non fornito
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
    
    % Gestione caso in cui non viene trovato un percorso ottimale
    if isempty(path)
        disp(['[AVVISO] Non è stato trovato un percorso ottimale per il leader ' num2str(leader_vehicle)]);
        disp('         Utilizzerò una traiettoria lineare per il resto del percorso.');
        
        % Crea un percorso semplice con più punti
        if use_custom_init
            start_position = init_state(violation_idx, leader_vehicle);
            start_time = violation_time;
        else
            start_position = 0;
            start_time = 0;
        end
        
        % Genera 10 punti equidistanti per avere una traiettoria più fluida
        num_points = 10;
        opt_t = linspace(start_time, start_time + 150, num_points)';
        total_distance = final_distance - start_position;
        opt_d = start_position + (opt_t - start_time) * (total_distance / 150);
        
        speeds = (total_distance / 150) * ones(1, length(opt_t)-1);
    else
        % Estrai i nodi del percorso ottimale
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
    
    % Salva i risultati della simulazione usando if-else invece dell'operatore ternario
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
            'violation_time', violation_time);
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
            'violation_time', violation_time);
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
                    
                    % Ignora le violazioni vicino alla fine del percorso
                    if cross_time > 120 && light_d > 1400  
                        fprintf('[INFO] Violazione vicino alla fine del percorso, ignoro la creazione di un nuovo plotone\n');
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
    plot_comparison();
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
    
    % Verifica che ci siano dati da plottare
    if isempty(opt_t) || isempty(opt_d)
        disp(['[AVVISO] Nessun dato disponibile per la traiettoria ottimale del plotone ' num2str(run_i)]);
        return;
    end
    
    % Anche se c'è solo un punto, traccialo come marker
    if length(opt_t) == 1
        color_idx = mod(leader-1, length(colors))+1;
        marker_idx = mod(run_i-1, length(markers))+1;
        h = scatter(opt_t, opt_d, 100, colors{color_idx}, markers{marker_idx}, 'filled');
        legend_handles(end+1) = h;
        legend_texts{end+1} = ['Leader ' num2str(leader) ' (Plotone ' num2str(run_i) ', punto singolo)'];
        return;
    end
    
    % Disegna la traiettoria del leader
    color_idx = mod(leader-1, length(colors))+1;
    line_idx = mod(run_i-1, length(line_styles))+1;
    marker_idx = mod(run_i-1, length(markers))+1;
    
    h = plot(opt_t, opt_d, [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 3);
    scatter(opt_t, opt_d, 70, colors{color_idx}, markers{marker_idx}, 'filled');
    
    % Prendi il primo handle se ne viene restituito più di uno
    if length(h) > 1
        h = h(1);
    end
    
    legend_handles(end+1) = h;
    legend_texts{end+1} = ['Leader ' num2str(leader) ' (Plotone ' num2str(run_i) ')'];
    
    % Calcola velocità target
    if length(opt_t) > 1
        v_targets = calculate_target_velocities(opt_t, opt_d);
        
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
                
                % Prendi il primo handle se ne viene restituito più di uno
                if length(h_follower) > 1
                    h_follower = h_follower(1);
                end
                
                legend_handles(end+1) = h_follower;
                legend_texts{end+1} = ['Follower ' num2str(v) ' (Plotone ' num2str(run_i) ')'];
            end
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
    
    % Combinazione delle traiettorie complete
    % Per ogni veicolo, combina le sue traiettorie da tutti i plotoni pertinenti
    n_vehicles = size(SIM_RUNS{1}.x, 2)/2;
    
    for v = 1:n_vehicles
        % Raccogli tutte le porzioni di traiettoria per questo veicolo
        all_times = [];
        all_positions = [];
        all_run_labels = [];
        
        for run_i = 1:length(SIM_RUNS)
            runData = SIM