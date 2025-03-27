%%%% filepath: /path/to/main_integrato.m
%%%% Codice completo funzionante

clear; clc; close all;

%% Esegui la simulazione iniziale con <Leader = Veicolo 1>
disp('=== Avvio simulazione con Leader = Veicolo 1 ===');
run_optimizer_and_plot(1);


%%%% =========================================================================
%%%%             S C R I P T   E   F U N Z I O N I   L O C A L I
%%%% =========================================================================

function run_optimizer_and_plot(leader_vehicle)
    % Ricostruisce l’intero flusso (1) ottimizzazione, (2) simulazione ODE, (3) check passaggi rosso
    fprintf('[INFO] Eseguo run_optimizer_and_plot con Leader = %d\n', leader_vehicle);

    final_time     = 150;         
    final_distance = 1800;    
    T  = 30;                   
    tf = final_time;
    v_min = 5;   
    v_max = 30;  
    b1 = 0.1;  
    b2 = 0.01;

    %% Creazione semafori
    traffic_lights = [
        create_traffic_light(300,   0, 10, T)
        create_traffic_light(600,  10, 20, T)
        create_traffic_light(900,  20, 30, T)
        create_traffic_light(1200,  0, 10, T)
        create_traffic_light(1550, 10, 20, T)
    ];

    %% Primo plot: stato semafori
    figure('Name',['Semafori (Leader=',num2str(leader_vehicle),')']);
    times = 0:tf-1;
    nLights = length(traffic_lights);
    all_times = []; 
    all_distances = []; 
    all_colors = [];
    for i = 1:nLights
        for t_ = times
            all_times(end+1)     = t_;
            all_distances(end+1) = traffic_lights(i).distance;
            if is_green(traffic_lights(i), t_)
                all_colors(end+1,:) = [0,1,0]; 
            else
                all_colors(end+1,:) = [1,0,0]; 
            end
        end
    end
    scatter(all_times, all_distances, 10, all_colors, 'filled');
    title('Stato dei semafori nel tempo');
    xlabel('Tempo (s)'); ylabel('Distanza (m)');
    grid on; hold on;

    %% Disegno t_min e t_max (vel pruning)
    draw_vehicle_path_simulation(tf, final_distance, traffic_lights, v_min, v_max);
    draw_vehicle_path_simulation_max(tf, final_distance, traffic_lights, v_min, v_max);

    %% COSTRUZIONE GRAFO + NODI
    [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance]; 
    nIntersections = length(traffic_lights);
    Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
    nodeId = 1;
    Nodes(nodeId) = struct('id', nodeId, 't', 0,'d', 0,'int', 0); 
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
                    overlap_end   = min(abs_green_end,   t_max(i));
                    if overlap_start < overlap_end
                        middle_time = ceil((overlap_start + overlap_end)/2);
                        Nodes(nodeId)=struct('id',nodeId,'t',middle_time,'d',d(i),'int',i);
                        nodeId = nodeId + 1;
                    end
                end
            else
                abs_green_start_1 = cycle_start + light.green_start;
                abs_green_end_1   = cycle_start + light.cycle_time;
                if abs_green_start_1 <= tf
                    overlap_start = max(abs_green_start_1, t_min(i));
                    overlap_end   = min(abs_green_end_1,   t_max(i));
                    if overlap_start < overlap_end
                        middle_time = (overlap_start + overlap_end)/2;
                        Nodes(nodeId)=struct('id',nodeId,'t',middle_time,'d',d(i),'int',i);
                        nodeId = nodeId + 1;
                    end
                end
                abs_green_start_2 = cycle_start;
                abs_green_end_2   = cycle_start + light.green_end;
                if abs_green_end_2 <= tf
                    overlap_start = max(abs_green_start_2, t_min(i));
                    overlap_end   = min(abs_green_end_2,   t_max(i));
                    if overlap_start < overlap_end
                        middle_time = (overlap_start + overlap_end)/2;
                        Nodes(nodeId)=struct('id',nodeId,'t',middle_time,'d',d(i),'int',i);
                        nodeId = nodeId + 1;
                    end
                end
            end
        end
    end

    Nodes(nodeId)=struct('id',nodeId,'t',tf,'d',final_distance,'int',nIntersections+1); 
    nNodes = nodeId;

    %% COSTRUZIONE ARCHI
    Edges = struct('from',{},'to',{},'w',{});
    edgeCount=1;
    for i = 1:nNodes
        curr_lvl = Nodes(i).int;
        for j = 1:nNodes
            next_lvl = Nodes(j).int;
            if next_lvl == curr_lvl+1
                if Nodes(j).t>Nodes(i).t && Nodes(j).d>Nodes(i).d
                    if Nodes(j).int>0 && Nodes(j).int<=nIntersections
                        if ~is_green(traffic_lights(Nodes(j).int), Nodes(j).t)
                            continue;
                        end
                    end
                    delta_t = Nodes(j).t - Nodes(i).t;
                    delta_d = Nodes(j).d - Nodes(i).d;
                    v_link  = delta_d/delta_t;
                    if v_link>=v_min && v_link<=v_max
                        E_link = delta_t*(b1*v_link + b2*v_link^2);
                        Edges(edgeCount)=struct('from',Nodes(i).id,'to',Nodes(j).id,'w',E_link);
                        edgeCount = edgeCount+1;
                    end
                end
            end
        end
    end

    %% DIJKSTRA
    [path, cost] = dijkstra(Nodes, Edges, Nodes(1).id, Nodes(end).id);
    fprintf('>>> Leader = %.2f, Costo energetico ottimo: %.6f\n', leader_vehicle, cost);
    fprintf('Percorso ottimo (node id - crossing time):\n');
    for k = 1:length(path)
        n_idx = path(k);
        fprintf('Node %d: t=%.6f s, d=%.6f m\n', ...
            Nodes(n_idx).id, Nodes(n_idx).t, Nodes(n_idx).d);
    end

    opt_nodes = Nodes(path);
    opt_t = arrayfun(@(n)n.t, opt_nodes);
    opt_d = arrayfun(@(n)n.d, opt_nodes);

    speeds = zeros(1,length(path)-1);
    for k = 1:(length(path)-1)
        idxA = path(k);
        idxB = path(k+1);
        delta_d = Nodes(idxB).d - Nodes(idxA).d;
        delta_t = Nodes(idxB).t - Nodes(idxA).t;
        speeds(k) = delta_d/delta_t;
    end
    disp('Velocità media su ciascun segmento (m/s):');
    disp(['[ ' num2str(speeds, '%.3f, ') ']']);

    plot(opt_t, opt_d, 'k--','LineWidth',3);
    title(['Traiettoria ottima (Leader=' num2str(leader_vehicle) ')']);
    legend('Semafori','t_{min}','t_{max}','Traiettoria ottima','Location','Best');

    %% SECONDA PARTE: SIMULAZIONE ODE
    b1_for_delta = 450;
    b2_for_delta = 450;
    delta_disturbo = @(t) 0*(b1_for_delta + b2_for_delta*sin(t)); % es. zero

    n_vehicles = 5;
    m_vehicles = 1000*ones(1,n_vehicles);

    % PID
    K_p_speed = 7000;     
    K_i_speed = 0;
    K_d_speed = 0.7;
    K_p_dist  = 2000;  
    K_i_dist  = 0.8;  
    K_d_dist  = 0.4;
    t_CTH     = 1.5;  
    d_init    = 4;

    v_targets = speeds;  
    x0 = zeros(2*n_vehicles,1);
    for i=1:n_vehicles
        if i==1, x0(i) = 0;
        else,    x0(i) = -d_init*(i-1);
        end
    end

    t_span = [0 150];
    [t_sim, x_sim] = ode45(@(t,x) system_dynamics_new_platoon( ...
       t,x,n_vehicles,m_vehicles,delta_disturbo,traffic_lights,v_targets,...
       t_CTH,K_p_speed,K_i_speed,K_d_speed,K_p_dist,K_i_dist,K_d_dist,leader_vehicle), ...
       t_span, x0);

    check_red_light_violations(t_sim, x_sim, traffic_lights);

    figure('Name',['Semafori+Veicoli (Leader=' num2str(leader_vehicle) ')']);
    hold on;
    times = floor(t_span(1)):floor(t_span(2));
    all_times     = [];
    all_positions = [];
    all_colors    = [];
    for i=1:length(traffic_lights)
        for j=1:length(times)
            time = times(j);
            all_times(end+1)     = time;
            all_positions(end+1) = traffic_lights(i).distance;
            if is_green(traffic_lights(i), time)
                all_colors(end+1,:)=[0,1,0];
            else
                all_colors(end+1,:)=[1,0,0];
            end
        end
    end
    scatter(all_times, all_positions, 10, all_colors, 'filled','DisplayName','Semafori');

    for i=1:n_vehicles
        plot(t_sim, x_sim(:,i), 'DisplayName',['Veicolo ' num2str(i)]);
    end
    title(['Mappa semafori e posizioni (Leader=' num2str(leader_vehicle) ')']);
    xlabel('Tempo [s]'); ylabel('Posizione [m]');
    legend('show');

    figure('Name',['Stati Veicoli (Leader=' num2str(leader_vehicle) ')']);
    for i=1:n_vehicles
        subplot(2,n_vehicles,i);
        plot(t_sim, x_sim(:, n_vehicles + i));
        title(['Vel Veicolo ' num2str(i)]);
        xlabel('Tempo [s]'); ylabel('V [m/s]');

        subplot(2,n_vehicles,n_vehicles + i);
        plot(t_sim, x_sim(:, i));
        title(['Pos Veicolo ' num2str(i)]);
        xlabel('Tempo [s]'); ylabel('X [m]');
    end
end


function dx = system_dynamics_new_platoon(t, x, n_vehicles, m, delta_func, ...
  traffic_lights, v_targets, t_CTH, K_p_speed, K_i_speed, K_d_speed, ...
  K_p_dist, K_i_dist, K_d_dist, leader_vehicle)
    % Dinamica con possibilita' di cambiare leader
    dx = zeros(2*n_vehicles,1);

    persistent t_prev
    if isempty(t_prev), t_prev = t; end
    dt = t - t_prev;
    if dt<=0, dt=0.01; end
    t_prev = t;

    persistent error_integral_speed previous_error_speed
    persistent error_integral_dist  previous_error_dist
    if isempty(error_integral_speed) || isempty(error_integral_dist)
        error_integral_speed = 0;
        previous_error_speed = 0;
        error_integral_dist  = zeros(n_vehicles,1);
        previous_error_dist  = zeros(n_vehicles,1);
    end

    for i=1:n_vehicles
        dx(i) = x(n_vehicles + i);

        if i == leader_vehicle
            vt = get_current_v_target_indexed(x(leader_vehicle), traffic_lights, v_targets);
            velocity_error = vt - x(n_vehicles + i);

            error_integral_speed = error_integral_speed + velocity_error*dt;
            velocity_derivative  = (velocity_error - previous_error_speed)/dt;
            previous_error_speed = velocity_error;

            U_leader = K_p_speed*velocity_error ...
                     + K_i_speed*error_integral_speed ...
                     + K_d_speed*velocity_derivative;

            dx(n_vehicles + i) = (U_leader + delta_func(t))/m(i);

            max_speed = 30;
            current_speed = x(n_vehicles + i)+ dx(n_vehicles + i)*dt;
            if current_speed<0
                current_speed=0;
            elseif current_speed>max_speed
                current_speed=max_speed;
            end
            dx(n_vehicles + i)=(current_speed - x(n_vehicles + i))/dt;
        else
            if i>1
                distance= x(i)- x(i-1);
            else
                distance= 10; 
            end
            v_cur = x(n_vehicles + i);
            d_min_val=1;
            d_CHT= -(d_min_val + t_CTH*v_cur);
            dist_error = d_CHT - distance;

            error_integral_dist(i)=error_integral_dist(i)+dist_error*dt;
            distance_derivative= (dist_error - previous_error_dist(i))/dt;
            previous_error_dist(i)=dist_error;

            U_dist= K_p_dist*dist_error ...
                  + K_i_dist*error_integral_dist(i) ...
                  + K_d_dist*distance_derivative;

            dx(n_vehicles + i)= U_dist/m(i);

            hypotetical_speed= x(n_vehicles + i)+ dx(n_vehicles + i)*dt;
            if hypotetical_speed<0
                hypotetical_speed=0;
            end
            dx(n_vehicles + i)=(hypotetical_speed - x(n_vehicles + i))/dt;
        end
    end
end

function vt = get_current_v_target_indexed(x_leader, traffic_lights, v_targets)
    idx = find(x_leader < [traffic_lights.distance], 1);
    if isempty(idx)
        vt = v_targets(end);
    else
        idx_clamp = min(idx, length(v_targets));
        vt = v_targets(idx_clamp);
    end
end

function check_red_light_violations(t_sim, x_sim, traffic_lights)
    persistent new_leader_detected
    if isempty(new_leader_detected), new_leader_detected = false; end

    n_vehicles = size(x_sim,2)/2;
    for v=1:n_vehicles
        pos_v = x_sim(:, v);
        for L=1:length(traffic_lights)
            light_dist = traffic_lights(L).distance;
            for k=2:length(t_sim)
                if pos_v(k-1)<light_dist && pos_v(k)>=light_dist
                    cross_time= t_sim(k-1)+(t_sim(k)-t_sim(k-1)) ...
                        *((light_dist - pos_v(k-1))/(pos_v(k)-pos_v(k-1)));
                    if ~is_green(traffic_lights(L), cross_time)
                        fprintf('\n[WARNING] Veicolo %d passa col rosso a incrocio %d (t=%.2f s)\n',...
                                v,L,cross_time);
                        if ~new_leader_detected
                            new_leader_detected= true;
                            fprintf('>> Veicolo %d diventa leader di un NUOVO plotone!\n', v);
                            rerun_optimizer_for_new_leader(v);
                        end
                        return;
                    end
                end
            end
        end
    end
end

function rerun_optimizer_for_new_leader(violating_vehicle)
    % Azzera le variabili *persistent* per ripartire da zero
    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear get_current_v_target_indexed

    disp(['[INFO] Ricalcolo tutto con NUOVO LEADER = Veicolo ', num2str(violating_vehicle)]);
    run_optimizer_and_plot(violating_vehicle);
end

function light = create_traffic_light(distance, green_start, green_end, cycle_time)
    light.distance      = distance;
    light.green_start   = green_start;
    light.green_end     = green_end;
    light.cycle_time    = cycle_time;
    light.green_duration= green_end - green_start;
    light.offset        = mod(green_start, cycle_time);
end

function status = is_green(light, time)
    time_in_cycle= mod(time - light.offset, light.cycle_time);
    if light.green_start<= light.green_end
        status= (time_in_cycle>=light.green_start) && (time_in_cycle<light.green_end);
    else
        status= (time_in_cycle>=light.green_start)||(time_in_cycle<light.green_end);
    end
end

function [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max)
    n= length(traffic_lights);
    d= [traffic_lights.distance];
    t_min= zeros(1,n); 
    t_max= zeros(1,n);

    t_min(1)= d(1)/v_max;
    t_max(1)= d(1)/v_min;
    t_min(1)= next_green(traffic_lights(1), t_min(1));
    t_max(1)= prev_green(traffic_lights(1), t_max(1));

    for i=2:n
        dist_inc= d(i)- d(i-1);
        t_min(i)= t_min(i-1)+ (dist_inc/v_max);
        t_max(i)= t_max(i-1)+ (dist_inc/v_min);
        t_max(i)= min(t_max(i), tf-(final_distance-d(i))/v_max);

        t_min(i)= next_green(traffic_lights(i), t_min(i));
        t_max(i)= prev_green(traffic_lights(i), t_max(i));
    end

    for i=n:-1:2
        needed_time= (d(i)- d(i-1))/v_max;
        if t_max(i)> t_max(i-1)+ needed_time
            t_max(i-1)= t_max(i)- needed_time;
            t_max(i-1)= prev_green(traffic_lights(i-1), t_max(i-1));
        end
    end
end

function t_next= next_green(light, t)
    if is_green(light, t)
        t_next= t;
    else
        time_in_cycle= mod(t- light.offset, light.cycle_time);
        if time_in_cycle< light.green_start
            t_next= t+ (light.green_start - time_in_cycle);
        else
            t_next= t+ (light.cycle_time- time_in_cycle)+ light.green_start;
        end
    end
end

function t_prev= prev_green(light, t)
    if is_green(light, t)
        t_prev= t;
    else
        time_in_cycle= mod(t- light.offset, light.cycle_time);
        if time_in_cycle>= light.green_end
            t_prev= t- (time_in_cycle- light.green_end);
        else
            t_prev= t- time_in_cycle- (light.cycle_time- light.green_end);
        end
    end
end

function draw_vehicle_path_simulation(tf, final_distance, traffic_lights, v_min, v_max)
    [t_min, ~] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d= [traffic_lights.distance];
    traj_t= [0, t_min, tf];
    traj_d= [0, d, final_distance];
    plot(traj_t, traj_d, 'b-','LineWidth',2);
    hold on;
end

function draw_vehicle_path_simulation_max(tf, final_distance, traffic_lights, v_min, v_max)
    [~, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d= [traffic_lights.distance];
    traj_t= [0, t_max, tf];
    traj_d= [0, d, final_distance];
    plot(traj_t, traj_d, 'b-','LineWidth',2);
    hold on;
end

function [path, cost] = dijkstra(Nodes, Edges, source, target)
    nNodes= length(Nodes);
    cost  = inf(1,nNodes);
    prev  = nan(1,nNodes);
    cost(source)= 0;
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
                    cost(v)= alt;
                    prev(v)= u;
                end
            end
        end
    end

    path= [];
    u= target;
    if ~isnan(prev(u))|| u==source
        while ~isnan(u)
            path= [u, path];
            u= prev(u);
        end
    end
end