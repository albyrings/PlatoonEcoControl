clear;
clearAllMemoizedCaches; 
clc; 
close all;
reset_persistent_variables();

global SIM_RUNS;    % Per salvare le singole simulazioni e creare un plot unico
global N_PLATOON;   % Per tracciare il numero di plotoni creati

SIM_RUNS = {};      % Ciascun elemento: struct con campi (t, x, offset, leader)
N_PLATOON = 1;      % Inizializza il contatore dei plotoni

disp('=== Avvio prima simulazione con Leader=1 ===');
run_optimizer_and_plot(1, 0);   % Leader veicolo 1, offset tempo = 0

final_plot();
%differencePlots();

%%%% --------------------------------------------------------------------
%%%%                    F U N Z I O N I   
%%%% --------------------------------------------------------------------

function run_optimizer_and_plot(leader_vehicle, time_offset)
    global SIM_RUNS

    % Soglia finale e parametri
    final_time     = 150;         
    final_distance = 1800;    
    T  = 30;                   
    tf = final_time;
    v_min = 5;   
    v_max = 30;  
    b1 = 0.1;  
    b2 = 0.01;

    fprintf('\n[INFO] run_optimizer_and_plot(Leader=%d, offset=%.2f)\n', leader_vehicle, time_offset);

    % Crea e configura semafori
    traffic_lights = [
        create_traffic_light(300,   0, 10, T)
        create_traffic_light(600,  10, 20, T)
        create_traffic_light(900,  20, 30, T)
        create_traffic_light(1200,  0, 10, T)
        create_traffic_light(1550, 10, 20, T)
    ];

    % Esempio di pruning min/max
    [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance]; 
    nIntersections = length(traffic_lights);

    % Nodi
    Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
    nodeId = 1;
    Nodes(nodeId) = struct('id', nodeId, 't', 0, 'd', 0, 'int', 0); 
    nodeId=nodeId+1;

    for i=1:nIntersections
        light = traffic_lights(i);
        for k=0:ceil(tf / light.cycle_time)
            cycle_start = k*light.cycle_time + light.offset;
            if light.green_start <= light.green_end
                abs_green_start = cycle_start + light.green_start;
                abs_green_end   = cycle_start + light.green_end;
                if abs_green_start <= tf
                    overlap_start = max(abs_green_start, t_min(i));
                    overlap_end   = min(abs_green_end, t_max(i));
                    if overlap_start<overlap_end
                        middle_time = ceil((overlap_start+overlap_end)/2);
                        Nodes(nodeId)=struct('id',nodeId,'t',middle_time,'d',d(i),'int',i);
                        nodeId=nodeId+1;
                    end
                end
            else
                abs_green_start_1 = cycle_start + light.green_start;
                abs_green_end_1   = cycle_start + light.cycle_time;
                if abs_green_start_1 <= tf
                    ov_start = max(abs_green_start_1, t_min(i));
                    ov_end   = min(abs_green_end_1,   t_max(i));
                    if ov_start<ov_end
                        mid_t = (ov_start+ov_end)/2; 
                        Nodes(nodeId)=struct('id',nodeId,'t',mid_t,'d',d(i),'int',i);
                        nodeId=nodeId+1;
                    end
                end
                abs_green_start_2 = cycle_start;
                abs_green_end_2   = cycle_start + light.green_end;
                if abs_green_end_2 <= tf
                    ov_start = max(abs_green_start_2, t_min(i));
                    ov_end   = min(abs_green_end_2,   t_max(i));
                    if ov_start<ov_end
                        mid_t = (ov_start+ov_end)/2;
                        Nodes(nodeId)=struct('id',nodeId,'t',mid_t,'d',d(i),'int',i);
                        nodeId=nodeId+1;
                    end
                end
            end
        end
    end
    Nodes(nodeId)=struct('id',nodeId,'t',tf,'d',final_distance,'int',nIntersections+1);
    nNodes=nodeId;

    % Archi
    Edges= struct('from',{},'to',{},'w',{});
    edgeCount=1;
    for i=1:nNodes
        lvlA=Nodes(i).int;
        for j=1:nNodes
            lvlB=Nodes(j).int;
            if lvlB==lvlA+1
                if Nodes(j).t>Nodes(i).t && Nodes(j).d>Nodes(i).d
                    if Nodes(j).int>0 && Nodes(j).int<=nIntersections
                        if ~is_green(traffic_lights(Nodes(j).int), Nodes(j).t)
                            continue;
                        end
                    end
                    delta_t= Nodes(j).t- Nodes(i).t;
                    delta_d= Nodes(j).d- Nodes(i).d;
                    v_link = delta_d/delta_t;
                    if v_link>=v_min && v_link<=v_max
                        E_link= delta_t*(b1*v_link + b2*v_link^2);
                        Edges(edgeCount)=struct('from',Nodes(i).id,'to',Nodes(j).id,'w',E_link);
                        edgeCount=edgeCount+1;
                    end
                end
            end
        end
    end

    [path, cost] = dijkstra(Nodes,Edges,1,nNodes);
    fprintf('>>> Leader=%.1f, Costo ottimo=%.3f\n', leader_vehicle, cost);
    opt_nodes= Nodes(path);
    opt_t    = arrayfun(@(n)n.t, opt_nodes);
    opt_d    = arrayfun(@(n)n.d, opt_nodes);

    speeds= zeros(1,length(path)-1);
    for k=1:(length(path)-1)
        d_ = opt_nodes(k+1).d- opt_nodes(k).d;
        t_ = opt_nodes(k+1).t- opt_nodes(k).t;
        speeds(k)= d_/t_;
    end

    disp('Velocità media (m/s):');
    disp(num2str(speeds,'%.2f '));

    % Simulazione ODE
    n_vehicles=6;
    m_vehicles=1000*ones(1,n_vehicles);
    v_targets=speeds;  

    % PID
    K_p_speed=7000; K_i_speed=0; K_d_speed=0.1;
    K_p_dist=2000;  K_i_dist=0.8;K_d_dist=0.4;
    t_CTH=1.5;  
    d_init=4;

    x0=zeros(2*n_vehicles,1);
    for i=1:n_vehicles
        if i==1, x0(i)=0;
        else,    x0(i)=-d_init*(i-1);
        end
    end

    t_span=[0 150];
    [t_sim,x_sim]= ode45(@(t,x)system_dynamics_new_platoon( ...
          t,x,n_vehicles,m_vehicles,@(tt)0, traffic_lights,v_targets,t_CTH, ...
          K_p_speed,K_i_speed,K_d_speed,K_p_dist,K_i_dist,K_d_dist, ...
          leader_vehicle, time_offset), t_span, x0);

    % Salva i risultati con offset fittizio
    T_abs= t_sim + time_offset;
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

    % Check passaggi col rosso su tempo ASSOLUTO
    check_red_light_violations(T_abs, x_sim, traffic_lights,T);
end

function dx = system_dynamics_new_platoon(t, x, n_vehicles, m, delta_func, ...
    traffic_lights, v_targets, t_CTH, K_p_speed, K_i_speed, K_d_speed, K_p_dist, K_i_dist, K_d_dist, ...
    leader_vehicle, time_offset)
    % Usa tempo "assoluto" = t + time_offset
    dx = zeros(2*n_vehicles, 1);

    % Calcolo corretto di dt
    persistent t_prev
    if isempty(t_prev), t_prev = t; end
    dt = t - t_prev;
    if dt <= 0, dt = 0.00001; end 
    t_prev = t;  

    % Variabili PID
    persistent e_int_speed e_old_speed
    persistent e_int_dist  e_old_dist
    if isempty(e_int_speed), e_int_speed = 0; e_old_speed = 0; end
    if isempty(e_int_dist),  e_int_dist = zeros(n_vehicles, 1); e_old_dist = zeros(n_vehicles, 1); end

    abs_t = t + time_offset;  

    for i = 1:n_vehicles
        % La derivata della posizione è la velocità
        dx(i) = x(n_vehicles + i);
        
        if i == leader_vehicle
            % --- LEADER: controllo velocità ---
            vt = get_current_v_target_indexed(x(leader_vehicle), traffic_lights, v_targets);
            vel_err = vt - x(n_vehicles + i);

            % Integrale dell'errore
            e_int_speed = e_int_speed + vel_err * dt;
            
            % Derivata dell'errore
            vel_deriv = (vel_err - e_old_speed) / dt;
            e_old_speed = vel_err;

            % Controllo PID -> forza
            U_leader = K_p_speed * vel_err + K_i_speed * e_int_speed + K_d_speed * vel_deriv;
            dx(n_vehicles + i) = (U_leader + delta_func(abs_t)) / m(i);

            % Limiti velocità
            max_speed = 30;
            new_vel = x(n_vehicles + i) + dx(n_vehicles + i) * dt;
            if new_vel < 0, new_vel = 0; end
            if new_vel > max_speed, new_vel = max_speed; end
            dx(n_vehicles + i) = (new_vel - x(n_vehicles + i)) / dt;
        else
            % --- FOLLOWER: controllo distanza ---
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

function vt= get_current_v_target_indexed(x_leader, traffic_lights, v_targets)
    idx= find(x_leader < [traffic_lights.distance],1);
    if isempty(idx)
        vt= v_targets(end);
    else
        idx= min(idx, length(v_targets));
        vt= v_targets(idx);
    end
end

function check_red_light_violations(t_abs, x_sim, traffic_lights,T)
    persistent new_leader_detected
    if isempty(new_leader_detected), new_leader_detected = false; end

    n_vehicles = size(x_sim,2)/2;
    for v=1:n_vehicles
        pos_v = x_sim(:,v);
        for L=1:length(traffic_lights)
            light_d = traffic_lights(L).distance;
            cross_idx = find(pos_v(1:end-1)<light_d & pos_v(2:end)>=light_d,1);
            if ~isempty(cross_idx)
                cross_time = t_abs(cross_idx);
                if ~is_green(traffic_lights(L), cross_time)
                    fprintf('\n[WARNING] Veicolo %d passa col rosso a incrocio %d (t=%.2f s)\n',...
                        v,L,cross_time);
                    if ~new_leader_detected
                        new_leader_detected = true;
                        fprintf('>> Veicolo %d diventa leader di un nuovo plotone!\n',v);
                        
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
          ', riparto da tempo assoluto=', num2str(N_PLATOON*T)]);

    start_offset = N_PLATOON*T;  
    N_PLATOON = N_PLATOON + 1;
    run_optimizer_and_plot(violating_vehicle, start_offset);
end

function final_plot()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[final_plot] Nessun dato da plottare.');
        return;
    end
    
    figure('Name','Posizioni Ottimali e Semafori', 'Position', [100, 100, 1000, 600]);
    hold on;
    
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    max_time = 0;
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
    
    scatter(all_times, all_distances, 10, all_colors, 'filled', 'DisplayName', 'Semafori');
    
    markers = {'o', 's', 'd', '^', 'v', '>', '<'};
    colors = {'b', 'r', 'g', 'm', 'c', 'k', [0.8 0.4 0], [0.5 0.5 0.5], [0.2 0.6 0.8]};
    line_styles = {'-', '--', ':', '-.'};
    
    legend_handles = [];
    legend_texts = {};
    
    n_vehicles = size(SIM_RUNS{1}.x, 2)/2;
    
    t_CTH = 1.5;  
    d_min = 1;    
    
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
            
            v_targets = [];
            for i = 1:length(opt_t)-1
                v = (opt_d(i+1) - opt_d(i)) / (opt_t(i+1) - opt_t(i));
                v_targets(i) = v;
            end
            
            color_idx = mod(leader-1, length(colors))+1;
            line_idx = mod(run_i-1, length(line_styles))+1;
            marker_idx = mod(run_i-1, length(markers))+1;
            
            h = plot(opt_t, opt_d, [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 3);
            scatter(opt_t, opt_d, 70, colors{color_idx}, markers{marker_idx}, 'filled');
            
            legend_handles(end+1) = h;
            legend_texts{end+1} = ['Leader ' num2str(leader) ' (Plotone ' num2str(run_i) ')'];
            
            if isempty(platoon_vehicles)
                ordered_vehicles = leader;
            else
                x_initial = runData.x(1, 1:n_vehicles);
                [~, idx] = sort(x_initial, 'descend');
                ordered_vehicles = idx;
            end
            
            for vidx = 1:length(ordered_vehicles)
                v = ordered_vehicles(vidx);
                if v ~= leader
                    preceding_vehicle_idx = find(ordered_vehicles == v) - 1;
                    if preceding_vehicle_idx < 1
                        preceding_vehicle = leader;
                    else
                        preceding_vehicle = ordered_vehicles(preceding_vehicle_idx);
                    end
                    
                    follower_opt_t = opt_t; 
                    follower_opt_d = zeros(size(opt_d));
                    
                    for i = 1:length(opt_t)
                        if i == 1
                            current_v = v_targets(1);
                        else
                            if i <= length(v_targets)
                                current_v = v_targets(i-1);
                            else
                                current_v = v_targets(end);
                            end
                        end
                        
                        safety_distance = d_min + t_CTH * current_v;
                        
                        if preceding_vehicle == leader
                            preceding_position = opt_d(i);
                        else
                            preceding_position = follower_opt_d(i);
                        end
                        
                        follower_opt_d(i) = preceding_position - safety_distance;
                    end
                    
                    follower_color_idx = mod(v-1, length(colors))+1;
                    follower_line_idx = mod(run_i-1, length(line_styles))+1;
                    follower_marker_idx = mod(v-1, length(markers))+1;
                    
                    h_follower = plot(follower_opt_t, follower_opt_d, [colors{follower_color_idx}, line_styles{follower_line_idx}], 'LineWidth', 2);
                    scatter(follower_opt_t, follower_opt_d, 40, colors{follower_color_idx}, markers{follower_marker_idx}, 'filled');
                    
                    legend_handles(end+1) = h_follower;
                    legend_texts{end+1} = ['Follower ' num2str(v) ' (Plotone ' num2str(run_i) ')'];
                end
            end
        end
    end
    
    legend(legend_handles, legend_texts, 'Location', 'Best');
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
    title('Traiettorie ottimali dei veicoli e stato semafori');
    grid on;
    
    figure('Name','Grafico Traiettorie Reali', 'Position', [150, 150, 1000, 600]);
    hold on;
    
    scatter(all_times, all_distances, 10, all_colors, 'filled');
    
    colors = {'b', 'r', 'g', 'm', 'c', 'y', 'k'};
    line_styles = {'-', '-', ':', '-.'};
    
    plotted_vehicles = [];
    
    for run_i=1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        
        if isfield(runData, 'splittedVehicles')
            splitted = runData.splittedVehicles;
        else
            splitted = [];
        end
        
        for v=1:size(x,2)/2
            if ismember(v, splitted) || ismember(v, plotted_vehicles)
                continue;
            end
            
            color_idx = mod(v-1, length(colors))+1;
            line_idx = mod(run_i-1, length(line_styles))+1;
            
            plot(t, x(:,v), [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 2);
            plotted_vehicles = [plotted_vehicles, v];
        end
    end
    
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
    title('Traiettorie reali dei veicoli e stato semafori');
    grid on;
    
    max_vehicle_id = 0;
    for run_i=1:length(SIM_RUNS)
        n_vehicles = size(SIM_RUNS{run_i}.x, 2)/2;
        max_vehicle_id = max(max_vehicle_id, n_vehicles);
    end
    
    vehicles_in_platoon2 = [];
    if length(SIM_RUNS) >= 2
        if isfield(SIM_RUNS{1}, 'splittedVehicles') && ~isempty(SIM_RUNS{1}.splittedVehicles)
            vehicles_in_platoon2 = SIM_RUNS{1}.splittedVehicles;
        end
    end
    
    for v=1:max_vehicle_id
        in_platoon2 = ismember(v, vehicles_in_platoon2);
        
        is_leader = false;
        leader_of_run = 0;
        for run_i=1:length(SIM_RUNS)
            if SIM_RUNS{run_i}.leader == v
                is_leader = true;
                leader_of_run = run_i;
                break;
            end
        end
        
        if is_leader
            fig_title = ['Profilo di Velocità - Veicolo ' num2str(v) ' (Leader)'];
        else
            fig_title = ['Profilo di Velocità - Veicolo ' num2str(v) ' (Follower)'];
        end
        
        figure('Name', fig_title, 'Position', [200+v*30, 200+v*30, 800, 400]);
        hold on;
        
        legend_handles = [];
        legend_texts = {};
        
        for run_i=1:length(SIM_RUNS)
            runData = SIM_RUNS{run_i};
            
            if in_platoon2 && run_i == 1
                continue;  
            end
            
            if ~in_platoon2 && run_i > 1
                continue;  
            end
            
            t_sim = runData.t;
            x_sim = runData.x;
            n_vehicles = size(x_sim, 2)/2;
            leader = runData.leader;
            
            if v <= n_vehicles
                v_real = x_sim(:, n_vehicles + v);
                h_real = plot(t_sim, v_real, 'b-', 'LineWidth', 2);
                legend_handles(end+1) = h_real;
                legend_texts{end+1} = ['Velocità Reale (Plotone ' num2str(run_i) ')'];
                
                if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
                    opt_t = runData.opt_t;
                    opt_d = runData.opt_d;
                    
                    leader_v_targets = [];
                    for i = 1:length(opt_t)-1
                        delta_t = opt_t(i+1) - opt_t(i);
                        delta_d = opt_d(i+1) - opt_d(i);
                        if delta_t > 0
                            leader_v_targets(i) = delta_d / delta_t;
                        else
                            leader_v_targets(i) = 0;
                        end
                    end
                    
                    if v == leader
                        v_targets = leader_v_targets;
                        
                        time_points = [];
                        velocity_points = [];
                        
                        for i = 1:length(v_targets)
                            if i == 1
                                time_points = [time_points, opt_t(i)];
                                velocity_points = [velocity_points, v_targets(i)];
                            end
                            
                            time_points = [time_points, opt_t(i+1)];
                            velocity_points = [velocity_points, v_targets(i)];
                        end
                        
                        h_opt = stairs(time_points, velocity_points, 'r--', 'LineWidth', 2);
                        scatter(opt_t(1:end-1), v_targets, 50, 'r', 'filled');
                        
                        legend_handles(end+1) = h_opt;
                        legend_texts{end+1} = ['Velocità Target (Plotone ' num2str(run_i) ')'];
                    else
                        v_targets = leader_v_targets;
                        
                        time_points = [];
                        velocity_points = [];
                        
                        for i = 1:length(v_targets)
                            if i == 1
                                time_points = [time_points, opt_t(i)];
                                velocity_points = [velocity_points, v_targets(i)];
                            end
                            
                            time_points = [time_points, opt_t(i+1)];
                            velocity_points = [velocity_points, v_targets(i)];
                        end
                        
                        h_opt = plot(time_points, velocity_points, 'r--', 'LineWidth', 2);
                        
                        legend_handles(end+1) = h_opt;
                        legend_texts{end+1} = ['Velocità Target (da Leader ' num2str(leader) ')'];
                    end
                end
            end
        end
        
        if ~isempty(legend_handles)
            legend(legend_handles, legend_texts, 'Location', 'Best');
        end
        
        xlabel('Tempo [s]');
        ylabel('Velocità [m/s]');
        title(['Profilo di Velocità - Veicolo ' num2str(v)]);
        grid on;
        ylim([0, 35]);
    end
    
    for v=1:max_vehicle_id
        in_platoon2 = ismember(v, vehicles_in_platoon2);
        
        is_leader = false;
        for run_i=1:length(SIM_RUNS)
            if SIM_RUNS{run_i}.leader == v
                is_leader = true;
                break;
            end
        end
        
        if is_leader
            fig_title = ['Posizione - Veicolo ' num2str(v) ' (Leader)'];
        else
            fig_title = ['Posizione - Veicolo ' num2str(v) ' (Follower)'];
        end
        
        figure('Name', fig_title, 'Position', [250+v*30, 250+v*30, 800, 400]);
        hold on;
        
        scatter(all_times, all_distances, 10, all_colors, 'filled');
        
        legend_handles = [];
        legend_texts = {};
        
        for run_i=1:length(SIM_RUNS)
            runData = SIM_RUNS{run_i};
            
            if in_platoon2 && run_i == 1
                continue;
            end
            
            if ~in_platoon2 && run_i > 1
                continue;
            end
            
            t_sim = runData.t;
            x_sim = runData.x;
            n_vehicles = size(x_sim, 2)/2;
            leader = runData.leader;
            
            if v <= n_vehicles
                pos_real = x_sim(:, v);
                h_real = plot(t_sim, pos_real, 'b-', 'LineWidth', 2);
                legend_handles(end+1) = h_real;
                legend_texts{end+1} = ['Posizione Reale (Plotone ' num2str(run_i) ')'];
                
                if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
                    platoon_vehicles = [];
                    if run_i == 1
                        platoon_vehicles = 1:n_vehicles;
                    else
                        prev_run = SIM_RUNS{run_i-1};
                        if isfield(prev_run, 'splittedVehicles') && ~isempty(prev_run.splittedVehicles)
                            platoon_vehicles = prev_run.splittedVehicles;
                        end
                    end
                    
                    if isempty(platoon_vehicles)
                        ordered_vehicles = leader;
                    else
                        x_initial = runData.x(1, 1:n_vehicles);
                        [~, idx] = sort(x_initial, 'descend');
                        ordered_vehicles = idx;
                    end
                    
                    opt_t = runData.opt_t;
                    opt_d = runData.opt_d;
                    
                    v_targets = [];
                    for i = 1:length(opt_t)-1
                        delta_t = opt_t(i+1) - opt_t(i);
                        delta_d = opt_d(i+1) - opt_d(i);
                        if delta_t > 0
                            v_targets(i) = delta_d / delta_t;
                        else
                            v_targets(i) = 0;
                        end
                    end
                    
                    if v == leader
                       h_opt = stairs(time_points, velocity_points, 'r--', 'LineWidth', 2);
                        scatter(opt_t, opt_d, 50, 'r', 'filled');
                        
                        legend_handles(end+1) = h_opt;
                        legend_texts{end+1} = ['Posizione Target (Plotone ' num2str(run_i) ')'];
                    else
                        follower_idx = find(ordered_vehicles == v);
                        preceding_vehicle_idx = follower_idx - 1;
                        
                        if preceding_vehicle_idx < 1 || preceding_vehicle_idx > length(ordered_vehicles)
                            preceding_vehicle = leader;
                        else
                            preceding_vehicle = ordered_vehicles(preceding_vehicle_idx);
                        end
                        
                        follower_opt_t = opt_t;
                        follower_opt_d = zeros(size(opt_d));
                        
                        d_min = 1;
                        t_CTH = 1.5;
                        
                        for i = 1:length(opt_t)
                            if i == 1
                                current_v = v_targets(1);
                            else
                                if i <= length(v_targets)
                                    current_v = v_targets(i-1);
                                else
                                    current_v = v_targets(end);
                                end
                            end
                            safety_distance = d_min + t_CTH * current_v;
                            
                            if preceding_vehicle == leader
                                preceding_position = opt_d(i);
                            else
                                if exist('follower_positions','var') && isfield(follower_positions, ['v' num2str(preceding_vehicle)])
                                    preceding_position = follower_positions.(['v' num2str(preceding_vehicle)])(i);
                                else
                                    rank_diff = abs(find(ordered_vehicles == preceding_vehicle) - find(ordered_vehicles == leader));
                                    preceding_position = opt_d(i) - safety_distance * rank_diff;
                                end
                            end
                            
                            follower_opt_d(i) = preceding_position - safety_distance;
                        end
                        
                        if ~exist('follower_positions', 'var')
                            follower_positions = struct();
                        end
                        follower_positions.(['v' num2str(v)]) = follower_opt_d;
                        
                        h_opt = plot(follower_opt_t, follower_opt_d, 'r--', 'LineWidth', 2);
                        legend_handles(end+1) = h_opt;
                        legend_texts{end+1} = ['Posizione Target (Plotone ' num2str(run_i) ')'];
                    end
                end
            end
        end
        
        if ~isempty(legend_handles)
            legend(legend_handles, legend_texts, 'Location', 'Best');
        end
        
        xlabel('Tempo [s]');
        ylabel('Posizione [m]');
        title(['Posizione - Veicolo ' num2str(v)]);
        grid on;
    end
end

function analyze_velocity_differences()
    global SIM_RUNS
    trigger_threshold = 2.0;
    
    figure('Name', 'Differenze Velocità rispetto alla Soglia', 'Position', [300, 300, 1200, 600]);
    hold on;
    title(['Differenze tra profili di velocità (soglia = ', num2str(trigger_threshold), ' m/s)']);
    xlabel('Tempo [s]');
    ylabel('Differenza di velocità - Soglia [m/s]');
    grid on;
    
    colors = {'b','r','g','m','c','k',[0.8 0.4 0],[0.5 0.5 0.5],[0.2 0.6 0.8]};
    line_styles = {'-','--',':','-.'};
    
    legend_handles = [];
    legend_texts = {};
    
    n_vehicles = size(SIM_RUNS{1}.x, 2)/2;
    
    for v = 1:n_vehicles
        for run_i = 1:length(SIM_RUNS)
            runData = SIM_RUNS{run_i};
            if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d')
                continue;
            end
            t_real = runData.t;
            x_sim = runData.x;
            v_real = x_sim(:, n_vehicles + v);
            leader = runData.leader;
            
            opt_t = runData.opt_t;
            opt_d = runData.opt_d;
            opt_v = [];
            opt_t_mid = [];
            
            for i = 1:length(opt_t)-1
                vel = (opt_d(i+1) - opt_d(i)) / (opt_t(i+1) - opt_t(i));
                opt_v = [opt_v, vel];
                opt_t_mid = [opt_t_mid, (opt_t(i) + opt_t(i+1))/2];
            end
            
            v_opt_interp = interp1(opt_t_mid, opt_v, t_real, 'previous', 'extrap');
            v_diff = abs(v_opt_interp - v_real) - trigger_threshold;
            
            color_idx = mod(v-1, length(colors))+1;
            line_idx = mod(run_i-1, length(line_styles))+1;
            h = plot(t_real, v_diff, 'Color', colors{color_idx}, 'LineStyle', line_styles{line_idx}, 'LineWidth', 1.5);
            
            if v == leader
                vehicle_type = 'Leader';
            else
                vehicle_type = 'Follower';
            end
            legend_handles(end+1) = h;
            legend_texts{end+1} = [vehicle_type ' ' num2str(v) ' (Plotone ' num2str(run_i) ')'];
        end
    end
    
    xl = xlim;
    plot(xl, [0 0], 'k--', 'LineWidth', 1.5);
    text(xl(1) + 0.02*(xl(2)-xl(1)), 0.1, 'Soglia', 'FontSize', 10);
    
    legend(legend_handles, legend_texts, 'Location', 'EastOutside');
    yl = ylim;
    if yl(1) > -2
        yl(1) = -2;
    end
    ylim(yl);
end

function analyze_velocity_differences_subplot()
    global SIM_RUNS
    trigger_threshold = 2.0;
    n_vehicles = size(SIM_RUNS{1}.x, 2)/2;
    
    n_rows = ceil(sqrt(n_vehicles));
    n_cols = ceil(n_vehicles / n_rows);
    
    figure('Name', 'Differenze Velocità per ogni veicolo', 'Position', [100, 100, 1200, 800]);
    
    colors = {'b','r','g','m','c','k',[0.8 0.4 0],[0.5 0.5 0.5],[0.2 0.6 0.8]};
    line_styles = {'-','--',':','-.'};
    
    for v = 1:n_vehicles
        subplot(n_rows, n_cols, v);
        hold on;        
        title(['Veicolo ' num2str(v)]);
        
        legend_handles = [];
        legend_texts = {};
        
        vehicle_data_found = false;
        
        for run_i = 1:length(SIM_RUNS)
            runData = SIM_RUNS{run_i};
            if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d')
                continue;
            end
            
            t_real = runData.t;
            x_sim = runData.x;
            v_real = x_sim(:, n_vehicles + v);
            leader = runData.leader;
            
            opt_t = runData.opt_t;
            opt_d = runData.opt_d;
            opt_v = [];
            opt_t_mid = [];
            
            for i = 1:length(opt_t)-1
                vel = (opt_d(i+1) - opt_d(i)) / (opt_t(i+1) - opt_t(i));
                opt_v = [opt_v, vel];
                opt_t_mid = [opt_t_mid, (opt_t(i) + opt_t(i+1))/2];
            end
            
            v_opt_interp = interp1(opt_t_mid, opt_v, t_real, 'previous', 'extrap');
            v_diff = abs(v_opt_interp - v_real) - trigger_threshold;
            
            color_idx = mod(run_i-1, length(colors))+1;
            line_idx = mod(run_i-1, length(line_styles))+1;
            h = plot(t_real, v_diff, 'Color', colors{color_idx}, 'LineStyle', line_styles{line_idx}, 'LineWidth', 1.5);
            
            vehicle_data_found = true;
            if v == leader
                legend_handles(end+1) = h;
                legend_texts{end+1} = ['Leader in Plotone ' num2str(run_i)];
            else
                legend_handles(end+1) = h;
                legend_texts{end+1} = ['Follower in Plotone ' num2str(run_i)];
            end
        end
        
        if ~vehicle_data_found
            text(0.5, 0.5, 'Nessun dato disponibile', 'HorizontalAlignment', 'center');
            axis([0 1 0 1]);
            continue;
        end
        
        xl = xlim;
        plot(xl, [0 0], 'k--', 'LineWidth', 1.0);
        
        if v > (n_rows-1)*n_cols
            xlabel('Tempo [s]');
        end
        if mod(v-1, n_cols) == 0
            ylabel('Diff velocità - Soglia [m/s]');
        end
        
        if ~isempty(legend_handles)
            legend(legend_handles, legend_texts, 'Location', 'Best', 'FontSize', 8);
        end
        
        ylim([-3, 3]);
        text(xl(1) + 0.05*(xl(2)-xl(1)), 0.2, ['Soglia = ' num2str(trigger_threshold) ' m/s'], 'FontSize', 7);
        grid on;
    end
    
    sgtitle(['Differenze di velocità rispetto alla soglia (' num2str(trigger_threshold) ' m/s)'], 'FontSize', 14);
    set(gcf, 'Position', [100, 100, 1200, 800]);
end

function light=create_traffic_light(distance, green_start, green_end, cycle_time)
    light.distance      = distance;
    light.green_start   = green_start;
    light.green_end     = green_end;
    light.cycle_time    = cycle_time;
    light.green_duration= green_end - green_start;
    light.offset        = mod(green_start, cycle_time);
end

function st=is_green(light, time)
    t_in_cycle= mod(time- light.offset, light.cycle_time);
    if light.green_start<=light.green_end
        st= (t_in_cycle>=light.green_start && t_in_cycle<light.green_end);
    else
        st= (t_in_cycle>=light.green_start || t_in_cycle<light.green_end);
    end
end

function [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, ...
                                           v_min, v_max)
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
    if is_green(light,t)
        t_next=t;
    else
        cyc= mod(t- light.offset, light.cycle_time);
        if cyc< light.green_start
            t_next= t+(light.green_start- cyc);
        else
            t_next= t+(light.cycle_time- cyc)+ light.green_start;
        end
    end
end

function t_prev=prev_green(light, t)
    if is_green(light,t)
        t_prev= t;
    else
        cyc= mod(t- light.offset, light.cycle_time);
        if cyc>= light.green_end
            t_prev= t-(cyc- light.green_end);
        else
            t_prev= t- cyc- (light.cycle_time- light.green_end);
        end
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

function reset_persistent_variables()
    % Resetta esplicitamente le funzioni che usano variabili persistent
    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear get_current_v_target_indexed
    clear next_green
    clear prev_green
    clear dijkstra
end

function differencePlots()
    % Calcola e plotta le differenze fra le posizioni e velocità reali
    % e quelle ottimali. Crea anche due grafici extra per ogni veicolo.

    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[differencePlots] Nessun dato da plottare.');
        return;
    end
    
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        if ~isfield(runData,'opt_t') || ~isfield(runData,'opt_d'), continue; end

        t_sim    = runData.t;        % Tempi reali di simulazione
        x_sim    = runData.x;        % Stati: prime colonne posizioni, seconde metà velocità
        n_vehicles = size(x_sim,2)/2;

        % Posizioni ottimali a gradini
        pos_opt = interp1(runData.opt_t, runData.opt_d, t_sim, 'previous', 'extrap');
        
        % Velocità ottimali a gradini
        opt_v    = [];
        opt_tmid = [];
        for k=1:(length(runData.opt_t)-1)
            dT = runData.opt_t(k+1) - runData.opt_t(k);
            dD = runData.opt_d(k+1) - runData.opt_d(k);
            opt_v(end+1)    = dD/dT;
            opt_tmid(end+1) = (runData.opt_t(k) + runData.opt_t(k+1))/2;
        end
        vel_opt = interp1(opt_tmid, opt_v, t_sim, 'previous', 'extrap');
        
        % Per ogni veicolo, grafici separati
        for v = 1:n_vehicles
            pos_real = x_sim(:, v);
            vel_real = x_sim(:, n_vehicles + v);

            pos_diff = pos_opt - pos_real;
            vel_diff = vel_opt - vel_real;
            
            % Primo grafico: differenza posizione
            figure('Name', sprintf('Differenza Posizione (Run %d, Veicolo %d)', run_i, v), ...
                   'Position', [300 300 600 400]);
            plot(t_sim, pos_diff, 'b-', 'LineWidth',1.5); grid on;
            xlabel('Tempo [s]');
            ylabel('Diff Posizione [m]');
            title(sprintf('Veicolo %d - Differenza Posizione (Run %d)', v, run_i));
            
            % Secondo grafico: differenza velocità
            figure('Name', sprintf('Differenza Velocità (Run %d, Veicolo %d)', run_i, v), ...
                   'Position', [350 350 600 400]);
            plot(t_sim, vel_diff, 'r-', 'LineWidth',1.5); grid on;
            xlabel('Tempo [s]');
            ylabel('Diff Velocità [m/s]');
            title(sprintf('Veicolo %d - Differenza Velocità (Run %d)', v, run_i));
        end
    end
end