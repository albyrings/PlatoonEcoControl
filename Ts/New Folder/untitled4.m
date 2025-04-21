% filepath: /path/to/file.m
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
diff_speed_plot_per_vehicle();
diff_speed_trigger_plot(5);  % Soglia impostata a 5
diff_speed_trigger_line_plot(5); % Soglia impostata a 5

diff_pos_plot_per_vehicle();
diff_pos_trigger_plot(5);         % soglia impostata a 5
diff_pos_trigger_line_plot(5);      % soglia impostata a 5




%% =========================================================================
%% Funzioni locali
%% =========================================================================

function run_optimizer_and_plot(leader_vehicle, time_offset)
    global SIM_RUNS N_PLATOON

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
    delta_func = @(t) 0 * (b1 + b2*(sin((1/b3)*t+b4) + 0.25*rand));
    
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

    % Costruzione nodi
    Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
    nodeId = 1;
    Nodes(nodeId) = struct('id', nodeId, 't', 0, 'd', 0, 'int', 0); 
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
                if abs_green_end_2 <= tf
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
    Nodes(nodeId) = struct('id', nodeId, 't', tf, 'd', final_distance, 'int', nIntersections + 1);
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

    [path, cost] = dijkstra(Nodes, Edges, 1, nNodes);
    fprintf('>>> Leader=%.1f, Costo ottimo=%.3f\n', leader_vehicle, cost);
    opt_nodes = Nodes(path);
    opt_t = arrayfun(@(n) n.t, opt_nodes);
    opt_d = arrayfun(@(n) n.d, opt_nodes);

    speeds = zeros(1, length(path) - 1);
    for k = 1:(length(path)-1)
        d_ = opt_nodes(k+1).d - opt_nodes(k).d;
        t_ = opt_nodes(k+1).t - opt_nodes(k).t;
        speeds(k) = d_ / t_;
    end

    n_vehicles = 5;
    m_vehicles = 1000 * ones(1, n_vehicles);
    v_targets = speeds;  

    % PID
    K_p_speed = 7000; K_i_speed = 0; K_d_speed = 0.1;
    K_p_dist = 2000; K_i_dist = 0.8; K_d_dist = 0.4;
    t_CTH = 1.5;  
    d_init = 4;

    x0 = zeros(2*n_vehicles, 1);
    for i = 1:n_vehicles
        if i == 1
            x0(i) = 0;
        else
            x0(i) = -d_init * (i - 1);
        end
    end

    t_span = [0 150];
    [t_sim, x_sim] = ode45(@(t, x) system_dynamics_new_platoon( ...
        t, x, n_vehicles, m_vehicles, @(tt) 0, traffic_lights, v_targets, t_CTH, ...
        K_p_speed, K_i_speed, K_d_speed, K_p_dist, K_i_dist, K_d_dist, ...
        leader_vehicle, time_offset), t_span, x0);

    T_abs = t_sim + time_offset;
    SIM_RUNS{end+1} = struct( ...
        'leader', leader_vehicle, ...
        't', T_abs, ...
        'x', x_sim, ...
        'offset', time_offset, ...
        'traffic_lights', traffic_lights, ...
        'splittedVehicles', [], ...
        'v_targets', speeds, ...
        'opt_t', opt_t + time_offset, ...
        'opt_d', opt_d);

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
    persistent e_int_dist e_old_dist
    if isempty(e_int_speed), e_int_speed = 0; e_old_speed = 0; end
    if isempty(e_int_dist), e_int_dist = zeros(n_vehicles, 1); e_old_dist = zeros(n_vehicles, 1); end

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
            U_dist = K_p_dist * dist_err + K_i_dist * e_int_dist(i) + K_d_dist * dist_deriv;
            dx(n_vehicles + i) = U_dist / m(i);
            new_vel = x(n_vehicles + i) + dx(n_vehicles + i) * dt;
            if new_vel < 0, new_vel = 0; end
            dx(n_vehicles + i) = (new_vel - x(n_vehicles + i)) / dt;
        end
    end
end

function vt = get_current_v_target_indexed(x_leader, traffic_lights, v_targets)
    idx = find(x_leader < [traffic_lights.distance], 1);
    if isempty(idx)
        vt = v_targets(end);
    else
        idx = min(idx, length(v_targets));
        vt = v_targets(idx);
    end
end

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
                    fprintf('\n[WARNING] Veicolo %d passa col rosso a incrocio %d (t=%.2f s)\n',...
                        v, L, cross_time);
                    if ~new_leader_detected
                        new_leader_detected = true;
                        fprintf('>> Veicolo %d diventa leader di un nuovo plotone!\n', v);
                        
                        global SIM_RUNS
                        last_idx = length(SIM_RUNS);
                        SIM_RUNS{last_idx}.splittedVehicles = v:n_vehicles;
                        
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
    clear system_dynamics_new_platoon
    clear get_current_v_target_indexed
    clear check_red_light_violations
    
    global SIM_RUNS
    global N_PLATOON
    
    disp(['[INFO] Ricalcolo con NUOVO LEADER=', num2str(violating_vehicle), ...
          ', riparto da tempo assoluto=', num2str(N_PLATOON * T)]);

    start_offset = N_PLATOON * T;  
    N_PLATOON = N_PLATOON + 1;
    run_optimizer_and_plot(violating_vehicle, start_offset);
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

function st = is_green(light, time)
    t_in_cycle = mod(time - light.offset, light.cycle_time);
    if light.green_start <= light.green_end
        st = (t_in_cycle >= light.green_start && t_in_cycle < light.green_end);
    else
        st = (t_in_cycle >= light.green_start || t_in_cycle < light.green_end);
    end
end

function [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max)
    n = length(traffic_lights);
    d = [traffic_lights.distance];
    t_min = zeros(1, n); t_max = zeros(1, n);
    t_min(1) = d(1) / v_max; t_max(1) = d(1) / v_min;
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

function t_next = next_green(light, t)
    if is_green(light, t)
        t_next = t;
        return;
    end
    cyc = mod(t - light.offset, light.cycle_time);
    if cyc < light.green_start
        t_next = t + (light.green_start - cyc);
    else
        t_next = t + (light.cycle_time - cyc) + light.green_start;
    end
end

function t_prev = prev_green(light, t)
    if is_green(light, t)
        t_prev = t;
        return;
    end
    cyc = mod(t - light.offset, light.cycle_time);
    if cyc >= light.green_end
        t_prev = t - (cyc - light.green_end);
    else
        t_prev = t - cyc - (light.cycle_time - light.green_end);
    end
end

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

function final_plot()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[final_plot] Nessun dato da plottare.');
        return;
    end
    
    % =========================================================================
    % Fig. 1: Posizioni Ottimali e Semafori
    % =========================================================================
    figure('Name', 'Posizioni Ottimali e Semafori');
    hold on;
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    max_time = 0;
    for i = 1:length(SIM_RUNS)
        max_time = max(max_time, max(SIM_RUNS{i}.t));
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
    
    markers = {'o', 's', 'd', '^', 'v', '>', '<'};
    colors = {'b', 'r', 'g', 'm', 'c', 'k', [0.8 0.4 0], [0.5 0.5 0.5], [0.2 0.6 0.8]};
    line_styles = {'-', '--', ':', '-.'};
    legend_handles = [];
    legend_texts = {};
    n_vehicles = size(SIM_RUNS{1}.x, 2) / 2;

    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        leader = runData.leader;
        if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
            opt_t = runData.opt_t;
            opt_d = runData.opt_d;
            platoon_vehicles = [];
            if run_i == 1
                platoon_vehicles = 1:n_vehicles;
            else
                prev_run = SIM_RUNS{run_i-1};
                if isfield(prev_run, 'splittedVehicles') && ~isempty(prev_run.splittedVehicles)
                    platoon_vehicles = prev_run.splittedVehicles;
                end
            end
            
            if isfield(runData, 'splittedVehicles') && ~isempty(runData.splittedVehicles)
                platoon_vehicles = setdiff(platoon_vehicles, runData.splittedVehicles);
            end

            % Calcolo velocità target se necessario
            v_targets = [];
            for i = 1:length(opt_t)-1
                v = (opt_d(i+1) - opt_d(i)) / (opt_t(i+1) - opt_t(i));
                v_targets(i) = v;
            end

            color_idx = mod(leader-1, length(colors)) + 1;
            line_idx = mod(run_i-1, length(line_styles)) + 1;
            marker_idx = mod(run_i-1, length(markers)) + 1;
            h = plot(opt_t, opt_d, [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 3);
            scatter(opt_t, opt_d, 70, colors{color_idx}, markers{marker_idx}, 'filled');
            legend_handles(end+1) = h;
            legend_texts{end+1} = ['Leader ' num2str(leader) ' (Plotone ' num2str(run_i) ')'];

            % Tracciamento ottimale dei follower
            for v = platoon_vehicles
                if v ~= leader
                    follower_opt_d = zeros(size(opt_d));
                    t_CTH = 1.5; 
                    d_min = 1;
                    initial_offset = abs(v - leader);
                    if v > leader
                        follower_opt_d(1) = opt_d(1) - initial_offset;
                    else
                        follower_opt_d(1) = opt_d(1) + initial_offset;
                    end
                    for idx_opt = 2:length(opt_t)
                        if v > leader
                            desired_gap = d_min + t_CTH * v_targets(idx_opt-1);
                            follower_opt_d(idx_opt) = opt_d(idx_opt) - desired_gap * (v - leader);
                        else
                            desired_gap = d_min + t_CTH * v_targets(idx_opt-1);
                            follower_opt_d(idx_opt) = opt_d(idx_opt) + desired_gap * (leader - v);
                        end
                    end
                    
                    follower_color_idx = mod(v-1, length(colors)) + 1;
                    follower_line_idx = mod(run_i-1, length(line_styles)) + 1;
                    follower_marker_idx = mod(v-1, length(markers)) + 1;
                    h_follower = plot(opt_t, follower_opt_d, ...
                        [colors{follower_color_idx}, line_styles{follower_line_idx}], 'LineWidth', 2);
                    scatter(opt_t, follower_opt_d, 40, colors{follower_color_idx}, ...
                        markers{follower_marker_idx}, 'filled');
                    legend_handles(end+1) = h_follower;
                    legend_texts{end+1} = ['Follower ' num2str(v) ' (Plotone ' num2str(run_i) ')'];
                end
            end
        end
    end
    
    legend(legend_handles, legend_texts, 'Location', 'Best');
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
    title('Traiettorie ottimali e semafori');
    grid on;

    % =========================================================================
    % Fig. 2: Grafico Traiettorie Reali
    % =========================================================================
    figure('Name', 'Grafico Traiettorie Reali');
    hold on;
    scatter(all_times, all_distances, 10, all_colors, 'filled');
    colors = {'b','r','g','m','c','y','k'};
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
            color_idx = mod(v-1, length(colors)) + 1;
            line_idx = mod(run_i-1, length(line_styles)) + 1;
            plot(t, x(:, v), [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 2);
            plotted_vehicles = [plotted_vehicles, v];
        end
    end
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
    title('Traiettorie reali e semafori');
    grid on;

    % =========================================================================
    % Fig. 3: Confronto Reale vs Ottimali + Semafori (tutti i veicoli)
    % =========================================================================
    figure('Name', 'Confronto Reale vs Ottimali + Semafori');
    hold on;
    scatter(all_times, all_distances, 10, all_colors, 'filled');  % Semafori

    % Posizioni reali (linea continua)
    colors_real = {'b','r','g','m','c','y','k'};
    line_styles_real = {'-'};
    plotted_vehicles = [];
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        splitted = [];
        if isfield(runData, 'splittedVehicles')
            splitted = runData.splittedVehicles;
        end
        for v = 1:size(x, 2) / 2
            if ismember(v, splitted) || ismember(v, plotted_vehicles)
                continue;
            end
            color_idx = mod(v-1, length(colors_real)) + 1;
            plot(t, x(:, v), [colors_real{color_idx}, line_styles_real{1}], 'LineWidth', 2);
            plotted_vehicles = [plotted_vehicles, v];
        end
    end

    % Posizioni ottimali (linea tratteggiata) per tutti i veicoli
    colors_opt = {'b','r','g','m','c','k',[0.8 0.4 0],[0.5 0.5 0.5],[0.2 0.6 0.8]};
    line_style_opt = '--';
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d')
            continue;
        end
        opt_t = runData.opt_t;
        opt_d = runData.opt_d;
        leader = runData.leader;

        if run_i == 1
            platoon_vehicles = 1:n_vehicles;
        else
            platoon_vehicles = [];
            prev_run = SIM_RUNS{run_i-1};
            if isfield(prev_run, 'splittedVehicles') && ~isempty(prev_run.splittedVehicles)
                platoon_vehicles = prev_run.splittedVehicles;
            end
        end
        if isfield(runData, 'splittedVehicles') && ~isempty(runData.splittedVehicles)
            platoon_vehicles = setdiff(platoon_vehicles, runData.splittedVehicles);
        end
        if isempty(platoon_vehicles) && run_i == 1
            platoon_vehicles = 1:n_vehicles;
        end
        
        % Calcola velocità 'v_targets'
        v_targets = [];
        for idx_opt = 1:length(opt_t) - 1
            v_ = (opt_d(idx_opt+1) - opt_d(idx_opt)) / (opt_t(idx_opt+1) - opt_t(idx_opt));
            v_targets(idx_opt) = v_;
        end

        % Veicoli
        for v = platoon_vehicles
            color_idx = mod(v-1, length(colors_opt)) + 1;
            if v == leader
                plot(opt_t, opt_d, [colors_opt{color_idx}, line_style_opt], 'LineWidth', 2);
            else
                follower_opt_d = zeros(size(opt_d));
                t_CTH = 1.5; 
                d_min = 1;
                offset = abs(v - leader);
                if v > leader
                    follower_opt_d(1) = opt_d(1) - offset;
                else
                    follower_opt_d(1) = opt_d(1) + offset;
                end
                for idx_opt = 2:length(opt_t)
                    if v > leader
                        desired_gap = d_min + t_CTH * v_targets(idx_opt-1);
                        follower_opt_d(idx_opt) = opt_d(idx_opt) - desired_gap * (v - leader);
                    else
                        desired_gap = d_min + t_CTH * v_targets(idx_opt-1);
                        follower_opt_d(idx_opt) = opt_d(idx_opt) + desired_gap * (leader - v);
                    end
                end
                plot(opt_t, follower_opt_d, [colors_opt{color_idx}, line_style_opt], 'LineWidth', 2);
            end
        end
    end

    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
    title('Confronto Traiettorie (Reali vs Ottimali) + Semafori');
    grid on;

    % =========================================================================
    % Grafici di velocità dei Leader
    % =========================================================================
    leaders = [];
    for run_i = 1:length(SIM_RUNS)
        if ~ismember(SIM_RUNS{run_i}.leader, leaders)
            leaders = [leaders, SIM_RUNS{run_i}.leader];
        end
    end

    for l = 1:length(leaders)
        current_leader = leaders(l);
        figure('Name', ['Velocità Leader ' num2str(current_leader)]);
        hold on;
        for run_i = 1:length(SIM_RUNS)
            runData = SIM_RUNS{run_i};
            if runData.leader == current_leader
                t = runData.t;
                x = runData.x;
                nv = size(x, 2) / 2;
                leader_velocity = x(:, nv + current_leader);
                plot(t, leader_velocity, 'b-', 'LineWidth', 2);
                if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
                    opt_t = runData.opt_t;
                    opt_d = runData.opt_d;
                    opt_v = [];
                    for i = 1:length(opt_t)-1
                        vv = (opt_d(i+1) - opt_d(i)) / (opt_t(i+1) - opt_t(i));
                        opt_v = [opt_v, vv];
                    end
                    plot([opt_t(1:end-1); opt_t(2:end)], [opt_v; opt_v], 'r--', 'LineWidth', 2);
                    scatter(opt_t(1:end-1), opt_v, 50, 'r', 'filled');
                end
            end
        end
        xlabel('Tempo [s]');
        ylabel('Velocità [m/s]');
        title(['Velocità Leader ' num2str(current_leader)]);
        grid on;
        ylim([0, 35]);
    end
end


function diff_plot_per_vehicle()
    % Questo grafico mostra, per ogni veicolo, in un subplot, la differenza
    % tra la traiettoria ottimizzata (shiftata per avere valore 0 al tempo 0)
    % e quella simulata, interpolata sui tempi di simulazione.
    
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[diff_plot_per_vehicle] Nessun dato da plottare.');
        return;
    end
    
    % Utilizziamo il primo run per il confronto
    runData = SIM_RUNS{1};
    t_sim   = runData.t;           % tempi della simulazione
    x_sim   = runData.x;           % posizioni simulate per i veicoli (colonne 1:n_vehicles)
    opt_t   = runData.opt_t;       % tempi ottimizzati (per il leader)
    opt_d_l = runData.opt_d;       % traiettoria ottimizzata del leader
    leader  = runData.leader;      % numero del leader
    
    n_vehicles = size(x_sim, 2) / 2;
    
    figure('Name', 'Differenza (Ottimizzatore - Modello) per Veicolo');
    for v = 1:n_vehicles        
        % Calcolo della traiettoria ottimizzata per il veicolo
        if v == leader
            opt_traj = opt_d_l;
        else
            follower_opt = zeros(size(opt_d_l));
            t_CTH = 1.5;
            d_min = 1;
            initial_offset = abs(v - leader);
            if v > leader
                follower_opt(1) = opt_d_l(1) - initial_offset;
            else
                follower_opt(1) = opt_d_l(1) + initial_offset;
            end
            for idx = 2:length(opt_t)
                dt = opt_t(idx) - opt_t(idx-1);
                desired_gap = d_min + t_CTH * ((opt_d_l(idx) - opt_d_l(idx-1)) / dt);
                if v > leader
                    follower_opt(idx) = opt_d_l(idx) - desired_gap * (v - leader);
                else
                    follower_opt(idx) = opt_d_l(idx) + desired_gap * (leader - v);
                end
            end
            opt_traj = follower_opt;
        end
        
        % SHIFT: portiamo l'istante 0 a 0
        opt_traj = opt_traj - opt_traj(1);
        
        % Interpola la traiettoria ottimizzata sui tempi della simulazione
        opt_interp = interp1(opt_t, opt_traj, t_sim, 'linear', 'extrap');
        sim_traj = x_sim(:, v);
        diff_traj = opt_interp - sim_traj;
        
        subplot(n_vehicles, 1, v);
        plot(t_sim, diff_traj, 'LineWidth', 2);
        grid on;
        ylabel(['Veic. ' num2str(v)]);
        if v == 1
            title('Differenza (Ottimizzatore - Modello) per ciascun veicolo');
        end
        if v == n_vehicles
            xlabel('Tempo [s]');
        end            
    end
end

function diff_speed_plot_per_vehicle()
    % Il grafico mostra, per ciascun veicolo, in un subplot, la differenza
    % tra la velocità derivata dalla traiettoria ottimizzata e quella simulata.
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[diff_speed_plot_per_vehicle] Nessun dato da plottare.');
        return;
    end
    
    % Utilizziamo il primo run per il confronto
    runData = SIM_RUNS{1};
    t_sim = runData.t;             % tempi della simulazione
    x_sim = runData.x;             % posizioni simulate (colonne 1:n_vehicles)
    opt_t = runData.opt_t;         % tempi ottimizzati
    opt_d_leader = runData.opt_d;  % traiettoria ottimizzata del leader
    leader = runData.leader;       % numero del leader
    
    n_vehicles = size(x_sim, 2) / 2;
    
    % Calcola le velocità simulate (utilizzando gradient per avere lunghezze uguali)
    sim_speed = zeros(length(t_sim), n_vehicles);
    for v = 1:n_vehicles
        sim_speed(:, v) = gradient(x_sim(:, v), t_sim);
    end
    
    % Calcola la velocità ottimizzata per il leader
    opt_speed_leader = gradient(opt_d_leader, opt_t);
    
    % Prepara il grafico
    figure('Name', 'Differenza Velocità (Ottimizzatore - Modello) per Veicolo');
    for v = 1:n_vehicles
        subplot(n_vehicles, 1, v);
        
        % Calcola la traiettoria ottimizzata per il veicolo v
        if v == leader
            opt_traj = opt_d_leader;
            opt_speed = opt_speed_leader;
        else
            follower_opt = zeros(size(opt_d_leader));
            t_CTH = 1.5;
            d_min = 1;
            initial_offset = abs(v - leader);
            if v > leader
                follower_opt(1) = opt_d_leader(1) - initial_offset;
            else
                follower_opt(1) = opt_d_leader(1) + initial_offset;
            end
            for idx = 2:length(opt_t)
                dt = opt_t(idx) - opt_t(idx-1);
                % Calcola il gap richiesto in modo simile a quanto fatto per le posizioni
                desired_gap = d_min + t_CTH * ((opt_d_leader(idx) - opt_d_leader(idx-1)) / dt);
                if v > leader
                    follower_opt(idx) = opt_d_leader(idx) - desired_gap * (v - leader);
                else
                    follower_opt(idx) = opt_d_leader(idx) + desired_gap * (leader - v);
                end
            end
            opt_traj = follower_opt;
            opt_speed = gradient(opt_traj, opt_t);
        end
        
        % Interpola la velocità ottimizzata sui tempi della simulazione
        opt_speed_interp = interp1(opt_t, opt_speed, t_sim, 'linear', 'extrap');
        
        % Calcola la differenza (Ottimizzatore - Simulato)
        diff_speed = opt_speed_interp - sim_speed(:, v);
        
        plot(t_sim, diff_speed, 'LineWidth', 2);
        grid on;
        ylabel(['Veic. ' num2str(v)]);
        if v == 1
            title('Differenza Velocità (Ottimizzatore - Modello) per ciascun veicolo');
        end
        if v == n_vehicles
            xlabel('Tempo [s]');
        end
    end
end


function diff_speed_trigger_plot(threshold)
    % Questa funzione crea un grafico a subplot per ogni veicolo, tracciando la differenza
    % tra la velocità calcolata dalla traiettoria ottimizzata (derivata numericamente)
    % e quella simulata. Verrà evidenziato (trigger) con marker rossi quando la differenza
    % supera "threshold", ignorando i primi 5 secondi di movimento.
    
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[diff_speed_trigger_plot] Nessun dato da plottare.');
        return;
    end
    
    % Utilizziamo il primo run per il confronto
    runData = SIM_RUNS{1};
    t_sim   = runData.t;             % tempi della simulazione
    x_sim   = runData.x;             % posizioni simulate per i veicoli nelle colonne 1:n_vehicles
    opt_t   = runData.opt_t;         % tempi ottimizzati
    opt_d_l = runData.opt_d;         % traiettoria ottimizzata del leader
    leader  = runData.leader;        % numero del leader
    n_vehicles = size(x_sim, 2) / 2;
    
    % Calcola le velocità simulate (usando gradient per avere lo stesso campionamento)
    sim_speed = zeros(length(t_sim), n_vehicles);
    for v = 1:n_vehicles
        sim_speed(:, v) = gradient(x_sim(:, v), t_sim);
    end
    
    % Calcola la velocità ottimizzata per il leader
    opt_speed_leader = gradient(opt_d_l, opt_t);
    
    figure('Name','Differenza Velocità (Opt - Sim) con Trigger');
    for v = 1:n_vehicles
        subplot(n_vehicles,1,v);
        
        % Calcola la traiettoria ottimizzata per il veicolo: se è leader è uguale, 
        % altrimenti viene calcolata una traiettoria "virtuale" per il follower.
        if v == leader
            opt_traj = opt_d_l;
            opt_speed = opt_speed_leader;
        else
            follower_opt = zeros(size(opt_d_l));
            t_CTH = 1.5;
            d_min = 1;
            initial_offset = abs(v - leader);
            if v > leader
                follower_opt(1) = opt_d_l(1) - initial_offset;
            else
                follower_opt(1) = opt_d_l(1) + initial_offset;
            end
            for idx = 2:length(opt_t)
                dt = opt_t(idx) - opt_t(idx-1);
                % Calcola il gap richiesto in base alla velocità media del leader
                desired_gap = d_min + t_CTH * ((opt_d_l(idx) - opt_d_l(idx-1)) / dt);
                if v > leader
                    follower_opt(idx) = opt_d_l(idx) - desired_gap * (v - leader);
                else
                    follower_opt(idx) = opt_d_l(idx) + desired_gap * (leader - v);
                end
            end
            opt_traj = follower_opt;
            opt_speed = gradient(opt_traj, opt_t);
        end
        
        % Interpola la velocità ottimizzata sui tempi della simulazione
        opt_speed_interp = interp1(opt_t, opt_speed, t_sim, 'linear', 'extrap');
        % Calcola la differenza (ottimizzatore - simulato)
        diff_speed = opt_speed_interp - sim_speed(:, v);
        
        % Traccia il grafico della differenza
        plot(t_sim, diff_speed, 'LineWidth', 1.5);
        hold on;
        
        % Trigger: considera solo istanti t > 5 secondi e segnala i punti in cui
        % la differenza supera la soglia
        trigger_idx = find(t_sim > 5 & abs(diff_speed) > threshold);
        if ~isempty(trigger_idx)
            plot(t_sim(trigger_idx), diff_speed(trigger_idx), 'r*', 'MarkerSize', 8);
        end
        
        hold off;
        grid on;
        ylabel(['Veic. ' num2str(v)]);
        if v == 1
            title(['Diff. Velocità (Opt - Sim) con trigger, soglia = ' num2str(threshold)]);
        end
        if v == n_vehicles
            xlabel('Tempo [s]');
        end
    end
end

function diff_speed_trigger_line_plot(threshold)
    % Questa funzione crea un grafico a subplot per ogni veicolo, mostrando un segnale
    % binario a linee: 0 quando il trigger è OFF (differenza di velocità <= soglia)
    % e 1 quando il trigger è ON (differenza di velocità > soglia). 
    % Il trigger viene attivato solo per t > 5 s.
    
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[diff_speed_trigger_line_plot] Nessun dato da plottare.');
        return;
    end
    
    % Utilizziamo i dati del primo run per il confronto
    runData = SIM_RUNS{1};
    t_sim   = runData.t;             % tempi della simulazione
    x_sim   = runData.x;             % posizioni simulate per i veicoli (colonne 1:n_vehicles)
    opt_t   = runData.opt_t;         % tempi ottimizzati
    opt_d_l = runData.opt_d;         % traiettoria ottimizzata del leader
    leader  = runData.leader;        % numero del leader
    n_vehicles = size(x_sim, 2) / 2;
    
    % Calcola le velocità simulate per ciascun veicolo (usando gradient)
    sim_speed = zeros(length(t_sim), n_vehicles);
    for v = 1:n_vehicles
        sim_speed(:, v) = gradient(x_sim(:, v), t_sim);
    end
    
    % Calcola la velocità ottimizzata per il leader
    opt_speed_leader = gradient(opt_d_l, opt_t);
    
    figure('Name','Trigger Velocità (0 = off, 1 = on)');
    for v = 1:n_vehicles
        subplot(n_vehicles, 1, v);
        
        % Calcola la traiettoria ottimizzata (e quindi la velocità) per il veicolo
        if v == leader
            opt_traj = opt_d_l;
            opt_speed = opt_speed_leader;
        else
            follower_opt = zeros(size(opt_d_l));
            t_CTH = 1.5;
            d_min = 1;
            initial_offset = abs(v - leader);
            if v > leader
                follower_opt(1) = opt_d_l(1) - initial_offset;
            else
                follower_opt(1) = opt_d_l(1) + initial_offset;
            end
            for idx = 2:length(opt_t)
                dt = opt_t(idx) - opt_t(idx-1);
                % Calcola il gap richiesto in base alla velocità media del leader
                desired_gap = d_min + t_CTH * ((opt_d_l(idx) - opt_d_l(idx-1)) / dt);
                if v > leader
                    follower_opt(idx) = opt_d_l(idx) - desired_gap * (v - leader);
                else
                    follower_opt(idx) = opt_d_l(idx) + desired_gap * (leader - v);
                end
            end
            opt_traj = follower_opt;
            opt_speed = gradient(opt_traj, opt_t);
        end
        
        % Interpola la velocità ottimizzata sui tempi della simulazione
        opt_speed_interp = interp1(opt_t, opt_speed, t_sim, 'linear', 'extrap');
        
        % Calcola la differenza di velocità (ottimizzatore - simulato)
        diff_speed = opt_speed_interp - sim_speed(:, v);
        
        % Crea il segnale trigger: per t > 5 s, 1 se la differenza supera la soglia, 0 altrimenti
        trigger = zeros(size(t_sim));
        valid_idx = t_sim > 5;
        trigger(valid_idx) = abs(diff_speed(valid_idx)) > threshold;
        
        % Traccia il grafico a linea del segnale (0/1)
        plot(t_sim, trigger, 'LineWidth', 1.5);
        ylim([-0.2, 1.2]);
        grid on;
        ylabel(['Veic. ' num2str(v)]);
        if v == 1
            title(['Trigger Velocità (0 = off, 1 = on), soglia = ' num2str(threshold)]);
        end
        if v == n_vehicles
            xlabel('Tempo [s]');
        end
    end
end


% filepath: /path/to/position_diff_functions.m

function diff_pos_plot_per_vehicle()
    % Questo grafico mostra, per ogni veicolo, in un subplot, la differenza
    % tra la traiettoria ottimizzata (shiftata per avere valore 0 al tempo 0)
    % e quella simulata, interpolata sui tempi della simulazione.
    
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[diff_pos_plot_per_vehicle] Nessun dato da plottare.');
        return;
    end
    
    % Utilizziamo il primo run per il confronto
    runData = SIM_RUNS{1};
    t_sim   = runData.t;           % tempi della simulazione
    x_sim   = runData.x;           % posizioni simulate per i veicoli (colonne 1:n_vehicles)
    opt_t   = runData.opt_t;       % tempi ottimizzati (per il leader)
    opt_d_l = runData.opt_d;       % traiettoria ottimizzata del leader
    leader  = runData.leader;      % numero del leader
    
    n_vehicles = size(x_sim, 2) / 2;
    
    figure('Name', 'Differenza Posizione (Ottimizzatore - Modello) per Veicolo');
    for v = 1:n_vehicles        
        % Calcola la traiettoria ottimizzata per il veicolo
        if v == leader
            opt_traj = opt_d_l;
        else
            follower_opt = zeros(size(opt_d_l));
            t_CTH = 1.5;
            d_min = 1;
            initial_offset = abs(v - leader);
            if v > leader
                follower_opt(1) = opt_d_l(1) - initial_offset;
            else
                follower_opt(1) = opt_d_l(1) + initial_offset;
            end
            for idx = 2:length(opt_t)
                dt = opt_t(idx) - opt_t(idx-1);
                desired_gap = d_min + t_CTH * ((opt_d_l(idx) - opt_d_l(idx-1)) / dt);
                if v > leader
                    follower_opt(idx) = opt_d_l(idx) - desired_gap * (v - leader);
                else
                    follower_opt(idx) = opt_d_l(idx) + desired_gap * (leader - v);
                end
            end
            opt_traj = follower_opt;
        end
        
        % SHIFT: porta il valore iniziale a 0
        opt_traj = opt_traj - opt_traj(1);
        
        % Interpola la traiettoria ottimizzata sui tempi della simulazione
        opt_interp = interp1(opt_t, opt_traj, t_sim, 'linear', 'extrap');
        sim_traj = x_sim(:, v);
        diff_traj = opt_interp - sim_traj;
        
        subplot(n_vehicles, 1, v);
        plot(t_sim, diff_traj, 'LineWidth', 2);
        grid on;
        ylabel(['Veic. ' num2str(v)]);
        if v == 1
            title('Differenza Posizione (Opt - Sim) per ciascun veicolo');
        end
        if v == n_vehicles
            xlabel('Tempo [s]');
        end            
    end
end

function diff_pos_trigger_plot(threshold)
    % Questa funzione crea un grafico a subplot per ogni veicolo,
    % tracciando la differenza tra la posizione ottimizzata e quella simulata.
    % Vengono evidenziati con marker rossi i punti in cui la differenza (in valore assoluto) supera 'threshold'
    % (considerando solo t > 5 s).
    
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[diff_pos_trigger_plot] Nessun dato da plottare.');
        return;
    end
    
    runData = SIM_RUNS{1};
    t_sim   = runData.t;
    x_sim   = runData.x;
    opt_t   = runData.opt_t;
    opt_d_l = runData.opt_d;
    leader  = runData.leader;
    n_vehicles = size(x_sim, 2) / 2;
    
    figure('Name','Differenza Posizione (Opt - Sim) con Trigger');
    for v = 1:n_vehicles
        subplot(n_vehicles, 1, v);
        
        % Calcola la traiettoria ottimizzata per il veicolo
        if v == leader
            opt_traj = opt_d_l;
        else
            follower_opt = zeros(size(opt_d_l));
            t_CTH = 1.5; d_min = 1;
            initial_offset = abs(v - leader);
            if v > leader
                follower_opt(1) = opt_d_l(1) - initial_offset;
            else
                follower_opt(1) = opt_d_l(1) + initial_offset;
            end
            for idx = 2:length(opt_t)
                dt = opt_t(idx) - opt_t(idx-1);
                desired_gap = d_min + t_CTH * ((opt_d_l(idx) - opt_d_l(idx-1)) / dt);
                if v > leader
                    follower_opt(idx) = opt_d_l(idx) - desired_gap * (v - leader);
                else
                    follower_opt(idx) = opt_d_l(idx) + desired_gap * (leader - v);
                end
            end
            opt_traj = follower_opt;
        end
        
        % SHIFT
        opt_traj = opt_traj - opt_traj(1);
        
        % Interpola la traiettoria sui tempi della simulazione
        opt_interp = interp1(opt_t, opt_traj, t_sim, 'linear', 'extrap');
        sim_traj = x_sim(:, v);
        diff_traj = opt_interp - sim_traj;
        
        plot(t_sim, diff_traj, 'b-', 'LineWidth', 1.5);
        hold on;
        
        % Trigger solo per t > 5 s
        trigger_idx = find(t_sim > 5 & abs(diff_traj) > threshold);
        if ~isempty(trigger_idx)
            plot(t_sim(trigger_idx), diff_traj(trigger_idx), 'ro', 'MarkerSize', 8);
        end
        
        hold off;
        grid on;
        ylabel(['Veic. ' num2str(v)]);
        if v==1
            title(['Diff. Posizione (Opt - Sim) con trigger, soglia = ' num2str(threshold)]);
        end
        if v==n_vehicles, xlabel('Tempo [s]'); end
    end
end

function diff_pos_trigger_line_plot(threshold)
    % Questa funzione mostra, per ogni veicolo, un segnale binario a linea continua:
    % 0 se la differenza in posizione (Opt - Sim) è minore o uguale a 'threshold'
    % (o per t <= 5 s) e 1 se la supera.
    
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[diff_pos_trigger_line_plot] Nessun dato da plottare.');
        return;
    end

    runData = SIM_RUNS{1};
    t_sim   = runData.t;
    x_sim   = runData.x;
    opt_t   = runData.opt_t;
    opt_d_l = runData.opt_d;
    leader  = runData.leader;
    n_vehicles = size(x_sim, 2) / 2;
    
    figure('Name','Trigger Posizione (0 = off, 1 = on)');
    for v = 1:n_vehicles
        subplot(n_vehicles, 1, v);
        
        if v == leader
            opt_traj = opt_d_l;
        else
            follower_opt = zeros(size(opt_d_l));
            t_CTH = 1.5; d_min = 1;
            initial_offset = abs(v - leader);
            if v > leader
                follower_opt(1) = opt_d_l(1) - initial_offset;
            else
                follower_opt(1) = opt_d_l(1) + initial_offset;
            end
            for idx = 2:length(opt_t)
                dt = opt_t(idx) - opt_t(idx-1);
                desired_gap = d_min + t_CTH * ((opt_d_l(idx) - opt_d_l(idx-1)) / dt);
                if v > leader
                    follower_opt(idx) = opt_d_l(idx) - desired_gap * (v - leader);
                else
                    follower_opt(idx) = opt_d_l(idx) + desired_gap * (leader - v);
                end
            end
            opt_traj = follower_opt;
        end
        
        % SHIFT
        opt_traj = opt_traj - opt_traj(1);
        
        % Interpolazione
        opt_interp = interp1(opt_t, opt_traj, t_sim, 'linear', 'extrap');
        sim_traj = x_sim(:, v);
        diff_traj = opt_interp - sim_traj;
        
        % Genera il segnale trigger binario: 1 se |diff| > threshold per t > 5 s, 0 altrimenti
        trigger = zeros(size(t_sim));
        valid = t_sim > 5;
        trigger(valid) = abs(diff_traj(valid)) > threshold;
        
        plot(t_sim, trigger, 'LineWidth', 1.5);
        ylim([-0.2, 1.2]);
        grid on;
        ylabel(['Veic. ' num2str(v)]);
        if v == 1
            title(['Trigger Posizione (0 = off, 1 = on), soglia = ' num2str(threshold)]);
        end
        if v == n_vehicles
            xlabel('Tempo [s]');
        end
    end
end

function leader_pos_diff_plot()
    % Questa funzione calcola e plottà la differenza fra la posizione simulata del leader
    % e quella ottimizzata (interpolata) – la differenza viene calcolata come (Sim - Opt).
    
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[leader_pos_diff_plot] Nessun dato da plottare.');
        return;
    end
    runData = SIM_RUNS{1};
    leader = runData.leader;
    t_sim  = runData.t;
    x_sim  = runData.x;
    opt_t  = runData.opt_t;
    opt_d  = runData.opt_d;
    
    % La posizione simulata del leader si assume sia nella colonna 'leader'
    leader_sim_pos = x_sim(:, leader);
    % Per il leader la traiettoria ottimizzata è la stessa (shiftata a 0)
    opt_traj = opt_d - opt_d(1);
    leader_opt_interp = interp1(opt_t, opt_traj, t_sim, 'linear', 'extrap');
    
    diff_pos = leader_sim_pos - leader_opt_interp;
    
    figure('Name',['Differenza Posizione Leader ' num2str(leader)]);
    plot(t_sim, diff_pos, 'm-', 'LineWidth', 2);
    xlabel('Tempo [s]');
    ylabel('Diff. posizione (m)');
    title(['Differenza (Sim - Opt) posizione del Leader ' num2str(leader)]);
    grid on;
end
