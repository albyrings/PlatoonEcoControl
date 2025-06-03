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
    SIM_RUNS = {};
    N_PLATOON = 1;
    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear check_velocity_triggers
    clear get_current_v_target_indexed
    
    % Reset persistent variables
    reset_persistent_variables();
    
    % Parameters and thresholds
    final_time     = 150;         
    final_distance = 1800;    
    T              = 30;                   
    tf             = final_time;
    v_min          = 3;   
    v_max          = 30;  
    b1             = 0.1;  
    b2             = 0.01;
    b3             = 10;
    b4             = 4;
    
    delta_func = @(t) 10000 * (b1 + b2*(sin((1/b3)*t+b4) + 0.25*rand));
    
    fprintf('\n[INFO] run_optimizer_and_plot(Leader=%d, offset=%.2f, start_pos=%.2f)\n', ...
        leader_vehicle, time_offset, start_position);
    
    % Traffic lights definition
    traffic_lights = [
         create_traffic_light(300,   0, 10, T)
         create_traffic_light(600,  10, 20, T)
         create_traffic_light(900,  20, 30, T)
         create_traffic_light(1200,  0, 10, T)
         create_traffic_light(1500, 10, 20, T)
    ];
     
    % Example pruning
    [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance]; 
    nIntersections = length(traffic_lights);
    
    % Nodes construction (the initial node uses start_position and time_offset)
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
    % Final node
    Nodes(nodeId) = struct('id', nodeId, 't', time_offset + tf, 'd', final_distance, 'int', nIntersections + 1);
    nNodes = nodeId;
    
    % Edges construction
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
        error('[ERROR] No valid path found by optimization.');
    end
    
    opt_nodes = Nodes(path);
    opt_t = arrayfun(@(n) n.t, opt_nodes);
    opt_d = arrayfun(@(n) n.d, opt_nodes);
    
    fprintf('>>> Leader=%.1f, Optimal cost=%.3f\n', leader_vehicle, cost);
    
    speeds = zeros(1, length(opt_t) - 1);
    for k = 1:(length(opt_t) - 1)
        d_ = opt_d(k + 1) - opt_d(k);
        t_ = opt_t(k + 1) - opt_t(k);
        speeds(k) = d_ / t_;
    end
    
    n_vehicles = 5;
    m_vehicles = 1000 * ones(1, n_vehicles);
    v_targets = speeds;  
    
    % PID parameters
    K_p_speed = 7000; K_i_speed = 0; K_d_speed = 0.1;
    K_p_dist  = 2000; K_i_dist  = 0.8; K_d_dist  = 0.4;
    t_CTH     = 1.5;  
    d_init    = 4;
    
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
    
    T_abs = t_sim;  
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
    
    % Check lights and triggers
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
        dx(i) = x(n_vehicles + i);   % update position
        if i == leader_vehicle
            vt = get_current_v_target_indexed(x(leader_vehicle), traffic_lights, v_targets);
            vel_err = vt - x(n_vehicles + i);
            e_int_speed = e_int_speed + vel_err * dt;
            vel_deriv = (vel_err - e_old_speed) / dt;
            e_old_speed = vel_err;
            U_leader = K_p_speed * vel_err + K_i_speed * e_int_speed + K_d_speed * vel_deriv;
            dx(n_vehicles + i) = (U_leader + delta_func(abs_t)) / m(i);
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
        end
    end
end

function vt = get_current_v_target_indexed(x_leader, traffic_lights, v_targets)
    if isempty(v_targets)
        fprintf('[WARNING] v_targets is empty, using default speed 10 m/s\n');
        vt = 10;
        return;
    end
    
    idx = find(x_leader < [traffic_lights.distance], 1);
    if isempty(idx)
        vt = v_targets(end);
    else
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
    
    % Get current leader
    current_leader = SIM_RUNS{current_run}.leader;
    
    % Distance margin for detection
    look_ahead_distance = 30;  
    
    for v = 1:n_vehicles
        if ~is_in_current_platoon(v, current_run)
            continue;
        end
        
        pos_v = x_sim(:, v);
        v_vel = x_sim(:, n_vehicles + v);
        
        for L = 1:length(traffic_lights)
            light_d = traffic_lights(L).distance;
            
            approaching_idx = find(pos_v >= light_d - look_ahead_distance & pos_v < light_d, 1, 'last');
            
            if ~isempty(approaching_idx)
                approach_time = t_abs(approaching_idx);
                current_pos = pos_v(approaching_idx);
                current_vel = v_vel(approaching_idx);
                
                time_to_light = (light_d - current_pos) / max(current_vel, 0.1);
                estimated_crossing_time = approach_time + time_to_light;
                
                if ~is_green(traffic_lights(L), estimated_crossing_time)
                    if any(violations_detected == v)
                        continue;
                    end
                    
                    fprintf('\n[WARNING] Vehicle %d is about to pass the red light at intersection %d (t=%.2f s)\n', v, L, approach_time);
                    fprintf('          Current position: %.2f m, Speed: %.2f m/s\n', current_pos, current_vel);
                    fprintf('          Light at: %.2f m, Estimated crossing time: %.2f s\n', light_d, estimated_crossing_time);
                    
                    violations_detected = [violations_detected, v];
                    
                    fprintf('>> Vehicle %d stops at the traffic light and becomes leader of a new platoon!\n', v);

                    SIM_RUNS{current_run}.splittedVehicles = v:n_vehicles;

                    next_green_time = next_green(traffic_lights(L), estimated_crossing_time);
                    
                    rerun_optimizer_from_traffic_light(v, next_green_time, light_d - 5, traffic_lights(L));
                    return;
                end
            end
        end
    end
end

function run_optimizer_from_traffic_light(leader_vehicle, start_time, start_position, ~)
    global SIM_RUNS N_PLATOON

    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear get_current_v_target_indexed
    
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
    
    fprintf('\n[INFO] run_optimizer_from_traffic_light(Leader=%d, time=%.2f, pos=%.2f)\n', ...
        leader_vehicle, start_time, start_position);

    traffic_lights = SIM_RUNS{1}.traffic_lights;
    
    remaining_lights = [];
    for i = 1:length(traffic_lights)
        if traffic_lights(i).distance > start_position
            remaining_lights = [remaining_lights; traffic_lights(i)];
        end
    end
    
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
        d = [remaining_lights.distance];
        nIntersections = length(remaining_lights);
        t_min = zeros(1, nIntersections);
        t_max = zeros(1, nIntersections);

        for i = 1:nIntersections
            dist_from_start = d(i) - start_position;
            if dist_from_start < 0
                t_min(i) = start_time;
                t_max(i) = start_time;
            else
                t_min(i) = start_time + dist_from_start / v_max;
                t_max(i) = start_time + dist_from_start / v_min;
                t_max(i) = min(t_max(i), start_time + tf - (final_distance - d(i)) / v_max);
                t_min(i) = next_green(remaining_lights(i), t_min(i));
                t_max(i) = prev_green(remaining_lights(i), t_max(i));
                if t_max(i) < t_min(i)
                    t_max(i) = t_min(i);
                end
            end
        end
        
        Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
        nodeId = 1;
        Nodes(nodeId) = struct('id', nodeId, 't', start_time, 'd', start_position, 'int', 0);
        nodeId = nodeId + 1;
        
        for i = 1:nIntersections
            light = remaining_lights(i);
            for k = 0:ceil(tf / light.cycle_time)
                cyc_start = k * light.cycle_time + light.offset;
                
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

        Nodes(nodeId) = struct('id', nodeId, 't', start_time + tf, 'd', final_distance, 'int', nIntersections+1);
        nNodes = nodeId;
        
        Edges = struct('from', {}, 'to', {}, 'w', {});
        edgeCount = 1;
        for i = 1:nNodes
            lvlA = Nodes(i).int;
            for j = 1:nNodes
                lvlB = Nodes(j).int;
                if lvlB == lvlA + 1
                    if (Nodes(j).t > Nodes(i).t) && (Nodes(j).d > Nodes(i).d)
                        if lvlB > 0 && lvlB <= nIntersections
                            if ~is_green(remaining_lights(lvlB), Nodes(j).t)
                                continue;
                            end
                        end
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

    [path, cost] = dijkstra(Nodes, Edges, 1, nNodes);
    fprintf('>>> Leader=%d, Optimal cost=%.3f\n', leader_vehicle, cost);

    invalid_cost = isscalar(cost) && isinf(cost);
    invalid_path = isempty(path) || (length(path) < 2);
    if invalid_cost || invalid_path
        fprintf('[ERROR] No valid path found. Creating a safe default path.\n');
        opt_t = [start_time; start_time + 100];
        opt_d = [start_position; final_distance];
        speeds = [10];
    else
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
        if isfield(SIM_RUNS{1}, 'splittedVehicles') && ...
           ~isempty(SIM_RUNS{1}.splittedVehicles) && ...
           ismember(vehicle_id, SIM_RUNS{1}.splittedVehicles)
            result = false;
        else
            result = true;
        end
        return;
    end
    
    prev_run = SIM_RUNS{current_run-1};
    if isfield(prev_run, 'splittedVehicles') && ...
       ~isempty(prev_run.splittedVehicles) && ...
       ismember(vehicle_id, prev_run.splittedVehicles)
        
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
    
    current_platoon = N_PLATOON + 1;
    disp(['[INFO] Creating NEW PLATOON ' num2str(current_platoon) ' with LEADER=' ...
          num2str(violating_vehicle) ', absolute time=' ...
          num2str(N_PLATOON*T) ', position=' num2str(stop_position)]);

    start_offset = N_PLATOON*T;  
    N_PLATOON = N_PLATOON + 1;
    
    run_optimizer_and_plot(violating_vehicle, start_offset, stop_position);
    clear check_red_light_violations
end

function final_plot()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[final_plot] No data to plot.');
        return;
    end
    
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
    figure('Name','Optimal Trajectories and Traffic Lights', 'Position', [100, 100, 900, 600]);
    hold on;
    
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    [all_times, all_distances, all_colors] = prepare_traffic_light_data(traffic_lights);
    scatter(all_times, all_distances, 10, all_colors, 'filled', 'DisplayName', 'Traffic Lights');
    
    markers = {'o','s','d','^','v','>','<'};
    colors = {'b','r','g','m','c','k',[0.8 0.4 0],[0.5 0.5 0.5],[0.2 0.6 0.8]};
    line_styles = {'-','--',':','-.'};
    legend_handles = [];
    legend_texts = {};
    
    for run_i = 1:length(SIM_RUNS)
        [legend_handles, legend_texts] = plot_run_optimal_trajectories(run_i, ...
                                          markers, colors, line_styles, ...
                                          legend_handles, legend_texts);
    end
    
    legend(legend_handles, legend_texts, 'Location', 'Best', 'NumColumns', 2);
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('Position [m]', 'FontSize', 12);
    title('Optimal Trajectories and Traffic Lights', 'FontSize', 14, 'FontWeight', 'bold');
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
    
    platoon_vehicles = get_platoon_vehicles(run_i);
    
    v_targets = calculate_target_velocities(opt_t, opt_d);
    
    color_idx = mod(leader-1, length(colors))+1;
    line_idx = mod(run_i-1, length(line_styles))+1;
    marker_idx = mod(run_i-1, length(markers))+1;
    
    h = plot(opt_t, opt_d, [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 3);
    scatter(opt_t, opt_d, 70, colors{color_idx}, markers{marker_idx}, 'filled');
    
    legend_handles(end+1) = h;
    legend_texts{end+1} = ['Leader ' num2str(leader) ' (Platoon ' num2str(run_i) ')'];
    
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
            legend_texts{end+1} = ['Follower ' num2str(v) ' (Platoon ' num2str(run_i) ')'];
        end
    end
end

function plot_real_trajectories()
    global SIM_RUNS
    
    figure('Name', 'Real Trajectories Plot', 'Position', [150, 150, 900, 600]);
    hold on;
    
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    [all_times, all_distances, all_colors] = prepare_traffic_light_data(traffic_lights);
    scatter(all_times, all_distances, 10, all_colors, 'filled');
    
    colors = {'b','r','g','m','c','y','k'};
    line_styles = {'-','-',':','-.'};
    
    legend_handles = [];
    legend_texts = {};
    
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        leader = runData.leader;
        
        platoon_vehicles = get_platoon_vehicles(run_i);
        
        for v = platoon_vehicles
            color_idx = mod(v-1, length(colors))+1;
            line_idx = mod(run_i-1, length(line_styles))+1;
            
            h = plot(t, x(:,v), [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 2);
            
            if v == leader
                legend_texts{end+1} = ['Leader ' num2str(v) ' (Platoon ' num2str(run_i) ')'];
            else
                legend_texts{end+1} = ['Vehicle ' num2str(v) ' (Platoon ' num2str(run_i) ')'];
            end
            legend_handles(end+1) = h;
        end
    end
    
    legend(legend_handles, legend_texts, 'Location', 'Best', 'NumColumns', 2);
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('Position [m]', 'FontSize', 12);
    title('Real Trajectories and Traffic Lights', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
end

function plot_comparison()
    global SIM_RUNS
    
    figure('Name', 'Real vs Optimal + Traffic Lights Comparison', 'Position', [200, 200, 1000, 600]);
    hold on;
    
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    [all_times, all_distances, all_colors] = prepare_traffic_light_data(traffic_lights);
    scatter(all_times, all_distances, 10, all_colors, 'filled');
    
    colors_real = {'b','r','g','m','c','y','k'};
    colors_opt = {'b','r','g','m','c','k',[0.8 0.4 0],[0.5 0.5 0.5],[0.2 0.6 0.8]};
    
    legend_handles = [];
    legend_texts = {};
    
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        leader = runData.leader;
        
        platoon_vehicles = get_platoon_vehicles(run_i);
        
        for v = platoon_vehicles
            color_idx = mod(v-1, length(colors_real))+1;
            h_real = plot(t, x(:,v), [colors_real{color_idx}, '-'], 'LineWidth', 2);
            
            if v == leader
                legend_texts{end+1} = ['Leader ' num2str(v) ' (real, Platoon ' num2str(run_i) ')'];
            else
                legend_texts{end+1} = ['Vehicle ' num2str(v) ' (real, Platoon ' num2str(run_i) ')'];
            end
            legend_handles(end+1) = h_real;
        end
        
        if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
            opt_t = runData.opt_t;
            opt_d = runData.opt_d;
            v_targets = calculate_target_velocities(opt_t, opt_d);
            
            color_idx = mod(leader-1, length(colors_opt))+1;
            h_opt_leader = plot(opt_t, opt_d, [colors_opt{color_idx}, '--'], 'LineWidth', 2);
            legend_texts{end+1} = ['Leader ' num2str(leader) ' (optimal, Platoon ' num2str(run_i) ')'];
            legend_handles(end+1) = h_opt_leader;
            
            for v = platoon_vehicles
                if v ~= leader
                    follower_opt_d = calculate_follower_trajectory(v, leader, opt_t, opt_d, v_targets);
                    color_idx = mod(v-1, length(colors_opt))+1;
                    h_opt_follower = plot(opt_t, follower_opt_d, [colors_opt{color_idx}, '--'], 'LineWidth', 2);
                    legend_texts{end+1} = ['Vehicle ' num2str(v) ' (optimal, Platoon ' num2str(run_i) ')'];
                    legend_handles(end+1) = h_opt_follower;
                end
            end
        end
    end
    
    legend(legend_handles, legend_texts, 'Location', 'Best', 'NumColumns', 2);
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('Position [m]', 'FontSize', 12);
    title('Trajectories (Real vs Optimal) + Traffic Lights', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
end

function plot_leader_velocities()
    global SIM_RUNS
    
    leaders = [];
    for run_i = 1:length(SIM_RUNS)
        leader = SIM_RUNS{run_i}.leader;
        if ~ismember(leader, leaders)
            leaders = [leaders, leader];
        end
    end
    
    for l = 1:length(leaders)
        current_leader = leaders(l);
        figure('Name', ['Leader Velocity ' num2str(current_leader)], 'Position', [250+l*50, 250+l*50, 800, 500]);
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
                legend_texts{end+1} = ['Real speed (Platoon ' num2str(run_i) ')'];
                
                if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
                    opt_t = runData.opt_t;
                    opt_d = runData.opt_d;
                    opt_v = calculate_target_velocities(opt_t, opt_d);
                    
                    for i = 1:length(opt_v)
                        t_segment = [opt_t(i), opt_t(i+1)];
                        v_segment = [opt_v(i), opt_v(i)];
                        if i == 1
                            h_opt = plot(t_segment, v_segment, 'r--', 'LineWidth', 2);
                            legend_handles(end+1) = h_opt;
                            legend_texts{end+1} = ['Target speed (Platoon ' num2str(run_i) ')'];
                        else
                            plot(t_segment, v_segment, 'r--', 'LineWidth', 2);
                        end
                    end
                    
                    scatter(opt_t(1:end-1), opt_v, 50, 'r', 'filled');
                end
            end
        end
        
        legend(legend_handles, legend_texts, 'Location', 'Best');
        xlabel('Time [s]', 'FontSize', 12);
        ylabel('Speed [m/s]', 'FontSize', 12);
        title(['Leader Velocity ' num2str(current_leader)], 'FontSize', 14, 'FontWeight', 'bold');
        grid on;
        ylim([0, 35]);
    end
end

function plot_energy_consumption()
    global SIM_RUNS
    
    figure('Name', 'Energy Consumption', 'Position', [300, 300, 800, 500]);
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
        
        b1 = 0.1;  
        b2 = 0.01;
        
        n_vehicles = size(x, 2)/2;
        total_energy = 0;
        
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
    
    bar_h = bar(1:length(SIM_RUNS), platoon_energy);
    
    for i = 1:length(SIM_RUNS)
        text(i, platoon_energy(i) + max(platoon_energy)*0.03, ...
             ['Leader ' num2str(platoon_leaders(i)) ' (' num2str(platoon_vehicle_count(i)) ' vehicles)'], ...
             'HorizontalAlignment', 'center');
    end
    
    xlabel('Platoon Number', 'FontSize', 12);
    ylabel('Energy Consumption [J]', 'FontSize', 12);
    title('Energy Consumption per Platoon', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
    set(gca, 'XTick', 1:length(SIM_RUNS));
end

function plot_inter_vehicle_distances()
    global SIM_RUNS
    
    figure('Name', 'Inter-Vehicle Distances', 'Position', [350, 350, 1000, 600]);
    
    n_plots = length(SIM_RUNS);
    rows = ceil(sqrt(n_plots));
    cols = ceil(n_plots/rows);
    
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        leader = runData.leader;
        
        subplot(rows, cols, run_i);
        hold on;
        
        platoon_vehicles = get_platoon_vehicles(run_i);
        
        distances = [];
        labels = {};
        
        if length(platoon_vehicles) >= 2
            [~, order_idx] = sort(x(end, platoon_vehicles), 'descend');
            sorted_vehicles = platoon_vehicles(order_idx);
            
            for i = 1:length(sorted_vehicles)-1
                v1 = sorted_vehicles(i);
                v2 = sorted_vehicles(i+1);
                distance = x(:, v1) - x(:, v2);
                plot(t, distance, 'LineWidth', 2);
                distances(end+1) = distance(end);
                labels{end+1} = ['Distance ' num2str(v1) '-' num2str(v2)];
            end
            
            title(['Inter-Vehicle Distances - Platoon ' num2str(run_i) ' (Leader ' num2str(leader) ')'], 'FontSize', 12);
            xlabel('Time [s]');
            ylabel('Distance [m]');
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
            text(0.5, 0.5, 'Not enough vehicles to compute distances', 'HorizontalAlignment', 'center');
            axis([0 1 0 1]);
        end
    end
end

function rerun_optimizer_from_traffic_light(vehicle_id, start_time, start_position, traffic_light)
    clear system_dynamics_new_platoon
    clear get_current_v_target_indexed
    
    global SIM_RUNS
    global N_PLATOON
    
    current_platoon = N_PLATOON + 1;
    disp(['[INFO] Creating NEW PLATOON ' num2str(current_platoon) ' with LEADER=' ...
          num2str(vehicle_id) ', waiting for green light at t=' ...
          num2str(start_time) ', position=' num2str(start_position)]);
    
    N_PLATOON = N_PLATOON + 1;
    
    run_optimizer_from_traffic_light(vehicle_id, start_time, start_position, traffic_light);
    
    clear check_red_light_violations
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
        disp('No data available for the delta speed plot.');
        return;
    end
    
    runData = SIM_RUNS{1};
    t_sim = runData.t;
    n_vehicles = size(runData.x, 2) / 2;
    
    figure('Name','Values for Delta Speed Calculation', 'Position', [100, 100, 1200, 800]);
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
        plot(t_sim, v_sim, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulated');
        plot(t_sim, v_opt, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Optimal');
        plot(t_sim, diff_v, 'r-.', 'LineWidth', 1.5, 'DisplayName', 'Delta (calculated)');
        xlabel('Time [s]');
        ylabel('Speed [m/s]');
        title(['Vehicle ' num2str(v) ': v_{sim}, v_{optimal}, and Delta']);
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
        disp('No data to plot for velocity triggers.');
        return;
    end
    
    split_times = [];
    split_vehicles = [];
    split_reasons = {};
    
    for i = 1:length(SIM_RUNS)
        if i < length(SIM_RUNS)
            if isfield(SIM_RUNS{i}, 'splittedVehicles') && ~isempty(SIM_RUNS{i}.splittedVehicles)
                next_leader = SIM_RUNS{i+1}.leader;
                split_time = SIM_RUNS{i+1}.offset;
                split_times(end+1) = split_time;
                split_vehicles(end+1) = next_leader;
                
                if split_time == i*30
                    split_reasons{end+1} = 'traffic light';
                else
                    split_reasons{end+1} = 'trigger';
                end
            end
        end
    end
    
    runData = SIM_RUNS{1};
    t_sim = runData.t;
    n_vehicles = size(runData.x, 2) / 2;
    
    figure('Name', 'Velocity Trigger and Platoon Split', 'Position', [100, 100, 1200, 800]);
    
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
        
        h5 = [];
        h6 = [];
        
        y_lim = get(gca, 'YLim');
        max_y = y_lim(2);
        
        for idx = 1:length(split_times)
            if split_vehicles(idx) == v
                if strcmp(split_reasons{idx}, 'traffic light')
                    h5 = plot([split_times(idx) split_times(idx)], [0 max_y], 'r-', 'LineWidth', 3);
                    text(split_times(idx), max_y*0.9, 'SPLIT (Traffic Light)', 'Color', 'r', ...
                         'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'FontWeight', 'bold');
                else
                    h6 = plot([split_times(idx) split_times(idx)], [0 max_y], 'c-', 'LineWidth', 3);
                    text(split_times(idx), max_y*0.9, 'SPLIT (Trigger)', 'Color', 'c', ...
                         'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'FontWeight', 'bold');
                end
            end
        end
        
        legend_handles = [h1, h2, h3, h4];
        legend_names = {'Simulated Speed', 'Optimal Speed', 'Speed Delta', 'Active Trigger'};
        
        if ~isempty(h5)
            legend_handles(end+1) = h5;
            legend_names{end+1} = 'Split for Red Light';
        end
        if ~isempty(h6)
            legend_handles(end+1) = h6;
            legend_names{end+1} = 'Split for Speed Trigger';
        end
        
        legend(legend_handles, legend_names, 'Location', 'best');
        
        xlabel('Time [s]');
        ylabel('Speed [m/s]');
        title(['Vehicle ' num2str(v) ': Trigger and Split Detection']);
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
    
    transitorio_inibizione = 10;
    
    current_offset = SIM_RUNS{current_run}.offset;
    tempo_relativo = t_abs(1) - current_offset;
    
    if tempo_relativo < transitorio_inibizione
        fprintf('[INFO] Transient active: triggers disabled for another %.1f seconds\n', ...
                transitorio_inibizione - tempo_relativo);
        return;
    end
    
    runData = SIM_RUNS{current_run};
    t_sim = t_abs;
    opt_t = runData.opt_t;
    opt_d = runData.opt_d;
    
    for v = 1:n_vehicles
        if ~is_in_current_platoon(v, current_run)
            continue;
        end
        
        if v == runData.leader
            continue;
        end
        
        pos_v = x_sim(:, v);
        v_vel = x_sim(:, n_vehicles + v);
        
        look_ahead_time = 5;
        
        for L = 1:length(traffic_lights)
            light_d = traffic_lights(L).distance;
            
            current_idx = length(pos_v);
            
            if pos_v(current_idx) < light_d && pos_v(current_idx) + v_vel(current_idx)*look_ahead_time >= light_d
                time_to_light = (light_d - pos_v(current_idx)) / max(v_vel(current_idx), 0.1);
                cross_time = t_abs(current_idx) + time_to_light;
                
                if ~is_green(traffic_lights(L), cross_time)
                    fprintf('\n[TRIGGER] Vehicle %d is about to pass the red light at intersection %d (t=%.2f s)\n', ...
                            v, L, t_abs(current_idx));
                    
                    if ~trigger_detected
                        trigger_detected = true;
                        fprintf('>> Vehicle %d becomes leader of a new platoon to avoid red light!\n', v);
                        
                        SIM_RUNS{current_run}.splittedVehicles = v:n_vehicles;
                        
                        rerun_optimizer_for_trigger(v, t_abs(current_idx));
                        trigger_detected = false;
                        last_platoon_time = t_abs(current_idx);
                        return;
                    end
                end
            end
        end
        
        v_sim = x_sim(:, n_vehicles + v);
        pos_opt = interp1(opt_t, opt_d, t_sim, 'linear', 'extrap');
        v_opt = gradient(pos_opt, t_sim);
        offset_value = v_opt(end) - v_sim(end);
        diff_v = (v_opt - v_sim) - offset_value;
        
        trigger_state = velocity_trigger(t_sim, diff_v, v_opt);
        
        if any(trigger_state) && ~trigger_detected
            trigger_idx = find(trigger_state, 1);
            trigger_time = t_sim(trigger_idx);
            
            current_position = x_sim(end, v);
            
            fprintf('\n[TRIGGER] Vehicle %d has triggered a velocity event at t=%.2f s, pos=%.2f m\n',...
                    v, trigger_time, current_position);
            
            trigger_detected = true;
            fprintf('>> Vehicle %d stops and becomes leader of a new platoon!\n', v);
            
            SIM_RUNS{current_run}.splittedVehicles = v:n_vehicles;
            
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
    
    current_platoon = N_PLATOON + 1;
    disp(['[INFO] TRIGGER ACTIVE: Creating NEW PLATOON ' num2str(current_platoon) ' with LEADER=' ...
          num2str(trigger_vehicle) ', absolute time=' ...
          num2str(trigger_time) ', position=' num2str(stop_position)]);
    
    start_offset = trigger_time;  
    N_PLATOON = N_PLATOON + 1;
    
    run_optimizer_and_plot(trigger_vehicle, start_offset, stop_position);
    
    clear check_red_light_violations
    clear check_velocity_triggers
end

function t_out = next_green(light, t_in)
    cycle = light.cycle_time;
    offset = light.offset;
    green_s = light.green_start; 
    green_e = light.green_end;

    dt = t_in - offset;
    if dt < 0, t_out = offset + green_s; return; end
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
        if phase >= green_s || phase <= green_e
            t_out = t_in;
        else
            t_out = offset + (k+1)*cycle + green_s;
        end
    end
end

function t_out = prev_green(light, t_in)
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
        if phase < green_s && phase > green_e
            t_out = offset + k*cycle + green_e;
        else
            t_out = t_in;
        end
    end
end

function flag = is_green(light, t_in)
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
        if phase >= green_s || phase <= green_e
            flag = true;
        else
            flag = false;
        end
    end
end

function check_and_trigger(t_abs, x_sim, traffic_lights, leader_vehicle)
    global SIM_RUNS

    [veh_red, time_red, pos_red] = detect_red_light(t_abs, x_sim, traffic_lights);
    if ~isempty(veh_red)
        fprintf('[TRIGGER] Vehicle %d is about to pass the red light.\n', veh_red);
        splitted_vehicles = veh_red : size(x_sim,2)/2;  
        SIM_RUNS{end}.splittedVehicles = splitted_vehicles;
        
        time_stop = time_red + 20;  
        run_new_platoon(veh_red, time_stop, pos_red, traffic_lights);
        return; 
    end

    [veh_vel_diff, trigger_time, trigger_pos] = detect_speed_trigger(t_abs, x_sim, traffic_lights);
    if ~isempty(veh_vel_diff)
        fprintf('[TRIGGER] Vehicle %d with excessive speed difference.\n', veh_vel_diff);
        splitted_vehicles = veh_vel_diff : size(x_sim,2)/2;
        SIM_RUNS{end}.splittedVehicles = splitted_vehicles;
        
        run_new_platoon(veh_vel_diff, trigger_time, trigger_pos, traffic_lights);
        return;
    end
end

function run_new_platoon(leader_id, start_time, start_position, traffic_lights)
    global N_PLATOON SIM_RUNS
    N_PLATOON = N_PLATOON + 1;
    fprintf('[INFO] Creating NEW PLATOON %d with Leader=%d\n', N_PLATOON, leader_id);
    
    run_optimizer_from_traffic_light(leader_id, start_time, start_position, traffic_lights);
end

function [veh_red, time_red, pos_red] = detect_red_light(t_abs, x_sim, lights)
    veh_red = [];
    time_red = [];
    pos_red = [];

    for i = 1 : size(x_sim,2)/2
        vPos = x_sim(end, i);
        vVel = x_sim(end, i + size(x_sim,2)/2);
        if vPos > 500 && vVel > 0
            veh_red = i;
            time_red = t_abs;
            pos_red = vPos;
            return;
        end
    end
end

function [veh_vel_diff, trigger_time, trigger_pos] = detect_speed_trigger(t_abs, x_sim, lights)
    veh_vel_diff = [];
    trigger_time = [];
    trigger_pos  = [];

    for i = 1 : size(x_sim,2)/2
        vVel = x_sim(end, i + size(x_sim,2)/2);
        if vVel > 35
            veh_vel_diff = i;
            trigger_time = t_abs;
            trigger_pos  = x_sim(end, i);
            return;
        end
    end
end