%% filepath: simulazione_platoon_modificata.m
% Riscrittura completa con la logica richiesta:
% quando si spezza il plotone, il nuovo ottimizzatore riparte
% dal semaforo in cui il veicolo non passa.

clear; clc; close all;

global SIM_RUNS N_PLATOON;
SIM_RUNS   = {};
N_PLATOON  = 1;

disp('=== Avvio prima simulazione con Leader=1 ===');
reset_persistent_variables();
% Simulazione iniziale: leader=1, tempo=0, distanza_iniziale=0
run_optimizer_and_plot(1, 0, 0);
final_plot();
plot_delta_velocities();
plot_velocity_trigger_per_vehicle();


%% --------------------------- MAIN FUNCTION ------------------------------
function run_optimizer_and_plot(leader_vehicle, time_offset, start_position)
    global SIM_RUNS N_PLATOON

    % Parametri e soglie
    final_time     = 150;  
    final_distance = 1800;  
    T              = 30;  
    v_min          = 5;    
    v_max          = 30;   
    b1             = 0.1;  
    b2             = 0.01;
    b3             = 10;
    b4             = 4;

    % Forzatura delle perturbazioni a zero (o quasi)
    delta_func = @(t) -0*(b1 + b2*(sin((1/b3)*t+b4) + 0.25*rand));

    fprintf('\n[INFO] run_optimizer_and_plot(Leader=%d, offset=%.2f, startPos=%.1f)\n', ...
        leader_vehicle, time_offset, start_position);

    % Definizione semafori
    traffic_lights = [
        create_traffic_light(300,   0, 10, T)
        create_traffic_light(600,  10, 20, T)
        create_traffic_light(900,  20, 30, T)
        create_traffic_light(1200,  0, 10, T)
        create_traffic_light(1550, 10, 20, T)
    ];

    % Pruning delle finestre di tempo possibili
    [t_min, t_max] = velocity_pruning(traffic_lights, final_time, final_distance, v_min, v_max);

    % Costruzione nodi e archi (grafo per ottimizzazione Dijkstra)
    [Nodes, Edges] = build_graph(traffic_lights, t_min, t_max, final_time, ...
                                 final_distance, v_min, v_max, b1, b2);

    % Dijkstra
    [path, cost] = dijkstra(Nodes, Edges, 1, length(Nodes));
    fprintf('>>> Leader=%.1f, Costo ottimo=%.3f\n', leader_vehicle, cost);
    opt_nodes = Nodes(path);
    opt_t     = arrayfun(@(n)n.t, opt_nodes);
    opt_d     = arrayfun(@(n)n.d, opt_nodes);
    
    speeds = zeros(1, length(path)-1);
    for k=1:(length(path)-1)
        d_ = opt_nodes(k+1).d - opt_nodes(k).d;
        t_ = opt_nodes(k+1).t - opt_nodes(k).t;
        speeds(k) = d_/t_;
    end

    % Setup simulazione
    n_vehicles  = 5;
    m_vehicles  = 1000*ones(1,n_vehicles);  % masse
    v_targets   = speeds;
    K_p_speed   = 7000; K_i_speed = 0; K_d_speed = 0.1;
    K_p_dist    = 2000; K_i_dist  = 0.8; K_d_dist = 0.4;
    t_CTH       = 1.5;  
    d_init      = 4;   
    x0          = zeros(2*n_vehicles,1);
    for i=1:n_vehicles
        if i==1
            x0(i) = start_position;  
        else
            x0(i) = start_position - d_init*(i-1);
        end
    end

    % Integrazione
    t_span          = [0, final_time];
    [t_sim, x_sim]  = ode45(@(t,x)system_dynamics_platoon(t, x, n_vehicles, m_vehicles, ...
                           delta_func, traffic_lights, v_targets, t_CTH, ...
                           K_p_speed, K_i_speed, K_d_speed, K_p_dist, ...
                           K_i_dist, K_d_dist, leader_vehicle, time_offset), ...
                           t_span, x0);

    T_abs = t_sim + time_offset;
    SIM_RUNS{end+1} = struct('leader', leader_vehicle, 't', T_abs, 'x', x_sim, ...
                             'offset', time_offset, 'traffic_lights', traffic_lights, ...
                             'splittedVehicles', [], 'v_targets', speeds, ...
                             'opt_t', opt_t + time_offset, 'opt_d', opt_d, ...
                             'startPos', start_position);

    % Verifica passaggi semafori
    check_red_light_violations(T_abs, x_sim, traffic_lights, T);
    check_velocity_triggers(T_abs, x_sim, traffic_lights);
end


%% --------------------------- DYNAMICS -----------------------------------
function dx = system_dynamics_platoon(t, x, n_vehicles, m, delta_func, ...
    traffic_lights, v_targets, t_CTH, K_p_speed, K_i_speed, K_d_speed, ...
    K_p_dist, K_i_dist, K_d_dist, leader_vehicle, time_offset)

    dx = zeros(2*n_vehicles, 1);
    persistent t_prev
    if isempty(t_prev), t_prev = t; end
    dt = t - t_prev; if dt<=0, dt=1e-5; end
    t_prev = t;

    persistent e_int_speed e_old_speed e_int_dist e_old_dist
    if isempty(e_int_speed), e_int_speed=0; e_old_speed=0; end
    if isempty(e_int_dist),  e_int_dist=zeros(n_vehicles,1); e_old_dist=zeros(n_vehicles,1); end

    abs_t = t + time_offset;

    for i=1:n_vehicles
        dx(i) = x(n_vehicles + i);  % derivata della posizione = velocità
        if i == leader_vehicle
            vt         = get_current_v_target_indexed(x(leader_vehicle), traffic_lights, v_targets);
            vel_err    = vt - x(n_vehicles + i);
            e_int_speed = e_int_speed + vel_err*dt;
            vel_deriv   = (vel_err - e_old_speed)/dt;
            e_old_speed = vel_err;
            U_leader    = K_p_speed*vel_err + K_i_speed*e_int_speed + K_d_speed*vel_deriv;
            dx(n_vehicles + i) = (U_leader + delta_func(abs_t))/m(i);
            
            % Limitazione velocità
            new_vel = x(n_vehicles+i) + dx(n_vehicles+i)*dt;
            if new_vel<0, new_vel=0; end
            if new_vel>30, new_vel=30; end
            dx(n_vehicles+i) = (new_vel - x(n_vehicles+i))/dt;
        else
            if i>1
                dist = x(i-1) - x(i);
            else
                dist = 12;  
            end
            v_cur      = x(n_vehicles + i);
            d_min_val  = 1;
            d_desired  = d_min_val + t_CTH*v_cur;
            dist_err   = dist - d_desired;
            e_int_dist(i) = e_int_dist(i) + dist_err*dt;
            dist_deriv = (dist_err - e_old_dist(i))/dt;
            e_old_dist(i) = dist_err;
            U_dist     = K_p_dist*dist_err + K_i_dist*e_int_dist(i) + K_d_dist*dist_deriv;
            dx(n_vehicles+i) = U_dist/m(i);
            
            % Limitazione velocità
            new_vel = x(n_vehicles+i) + dx(n_vehicles+i)*dt;
            if new_vel<0, new_vel=0; end
            if new_vel>30, new_vel=30; end
            dx(n_vehicles+i) = (new_vel - x(n_vehicles+i))/dt;
        end
    end
end


%% --------------------------- SPLIT LOGIC --------------------------------
function check_red_light_violations(t_abs, x_sim, traffic_lights, T)
    persistent violations_detected
    if isempty(violations_detected), violations_detected=[]; end
    
    global SIM_RUNS
    current_run   = length(SIM_RUNS);
    current_leader= SIM_RUNS{current_run}.leader;
    n_vehicles    = size(x_sim,2)/2;

    for v=1:n_vehicles
        if ~is_in_current_platoon(v, current_run), continue; end
        pos_v = x_sim(:,v);
        for L=1:length(traffic_lights)
            light_d   = traffic_lights(L).distance;
            cross_idx = find(pos_v(1:end-1)<light_d & pos_v(2:end)>=light_d, 1);
            if ~isempty(cross_idx)
                cross_time = t_abs(cross_idx);
                if ~is_green(traffic_lights(L), cross_time)
                    % se il veicolo passa col rosso
                    if any(violations_detected==v), continue; end
                    fprintf('\n[WARNING] Veicolo %d passa col rosso a incrocio %d (t=%.2f)\n', v,L,cross_time);
                    violations_detected = [violations_detected, v];
                    fprintf('>> Veicolo %d diventa leader di un nuovo plotone!\n', v);
                    SIM_RUNS{current_run}.splittedVehicles = v:n_vehicles;
                    rerun_optimizer_for_new_leader(v, L, cross_time);  % Modificato
                    return;
                end
            end
        end
    end
end

function check_velocity_triggers(t_abs, x_sim, traffic_lights)
    persistent trigger_detected
    if isempty(trigger_detected), trigger_detected=false; end
    
    global SIM_RUNS N_PLATOON
    current_run = length(SIM_RUNS);
    n_vehicles  = size(x_sim,2)/2;
    transitorio_inibizione = 10; 
    
    runData       = SIM_RUNS{current_run};
    current_leader= runData.leader;
    opt_t         = runData.opt_t;
    opt_d         = runData.opt_d;
    t_sim         = t_abs;
    tempo_relativo= t_abs(1) - runData.offset;
    if tempo_relativo<transitorio_inibizione
        fprintf('[INFO] Transitorio attivo. Trigger inibiti.\n');
        return;
    end
    
    for v=1:n_vehicles
        if ~is_in_current_platoon(v, current_run), continue; end
        if v==current_leader, continue; end
        
        pos_v = x_sim(:,v);
        vel_v = x_sim(:, n_vehicles+v);
        
        % check possibili rossi a breve
        look_ahead_time = 5;
        for L=1:length(traffic_lights)
            light_d = traffic_lights(L).distance;
            if pos_v(end)<light_d && pos_v(end)+vel_v(end)*look_ahead_time>=light_d
                time_to_light = (light_d - pos_v(end))/max(vel_v(end),0.1);
                cross_time    = t_abs(end) + time_to_light;
                if ~is_green(traffic_lights(L), cross_time)
                    fprintf('\n[TRIGGER] Veicolo %d innesca rosso incrocio %d a t=%.2f\n', v, L, t_abs(end));
                    if ~trigger_detected
                        trigger_detected=true;
                        fprintf('>> Veicolo %d diventa leader di un nuovo plotone!\n', v);
                        SIM_RUNS{current_run}.splittedVehicles = v:n_vehicles;
                        rerun_optimizer_for_trigger(v, L, t_abs(end));
                        trigger_detected=false;
                        return;
                    end
                end
            end
        end
        
        % check differenze di velocità
        pos_opt  = interp1(opt_t, opt_d, t_sim, 'linear','extrap');
        v_opt    = gradient(pos_opt, t_sim);
        off_val  = v_opt(end) - vel_v(end);
        diff_v   = (v_opt - vel_v) - off_val;
        trigger_state = velocity_trigger(t_sim, diff_v, v_opt);
        
        if any(trigger_state) && ~trigger_detected
            idx = find(trigger_state,1);
            fprintf('\n[TRIGGER] Veicolo %d attiva trigger di velocità a t=%.2f\n', v, t_sim(idx));
            trigger_detected = true;
            fprintf('>> Veicolo %d nuovo leader!\n', v);
            SIM_RUNS{current_run}.splittedVehicles = v:n_vehicles;
            rerun_optimizer_for_trigger(v, -1, t_sim(idx));
            trigger_detected = false;
            return;
        end
    end
end

function rerun_optimizer_for_new_leader(violating_vehicle, light_index, cross_time)
    clear system_dynamics_platoon get_current_v_target_indexed
    global SIM_RUNS N_PLATOON
    
    % Recupero informazioni sul semaforo e offset
    current_run    = length(SIM_RUNS);
    this_run       = SIM_RUNS{current_run};
    chosen_light   = this_run.traffic_lights(light_index).distance;
    
    new_start_pos  = chosen_light;  
    new_offset     = cross_time;
    current_platoon= N_PLATOON + 1;
    disp(['[INFO] Sono nel Plotone ' num2str(current_platoon) ...
          ', NEW LEADER=' num2str(violating_vehicle) ...
          ', startPos=' num2str(new_start_pos) ...
          ', time=' num2str(new_offset)]);
    N_PLATOON = current_platoon;
    
    run_optimizer_and_plot(violating_vehicle, new_offset, new_start_pos);
    clear check_red_light_violations check_velocity_triggers
end

function rerun_optimizer_for_trigger(trigger_vehicle, light_index, trigger_time)
    clear system_dynamics_platoon get_current_v_target_indexed
    global SIM_RUNS N_PLATOON
    
    current_run  = length(SIM_RUNS);
    this_run     = SIM_RUNS{current_run};
    
    if light_index>0
        new_start_pos = this_run.traffic_lights(light_index).distance;
    else
        % se il trigger non è legato a un incrocio (light_index=-1),
        % facciamo partire dal veicolo (posizione attuale)
        pos_v   = this_run.x(end, trigger_vehicle);
        new_start_pos = pos_v; 
    end
    
    new_offset     = trigger_time;
    current_platoon= N_PLATOON + 1;
    disp(['[INFO] TRIGGER: Plotone ' num2str(current_platoon) ...
          ' leader=' num2str(trigger_vehicle) ...
          ', startPos=' num2str(new_start_pos) ...
          ', time=' num2str(new_offset)]);
    N_PLATOON = current_platoon;
    
    run_optimizer_and_plot(trigger_vehicle, new_offset, new_start_pos);
    clear check_red_light_violations check_velocity_triggers
end


%% --------------------------- UTILITIES ----------------------------------
function reset_persistent_variables()
    clear system_dynamics_platoon
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

function st = is_green(light, time)
    t_in_cycle = mod(time - light.offset, light.cycle_time);
    if light.green_start <= light.green_end
        st = (t_in_cycle >= light.green_start && t_in_cycle < light.green_end);
    else
        st = (t_in_cycle >= light.green_start || t_in_cycle < light.green_end);
    end
end

function [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max)
    n          = length(traffic_lights);
    d          = [traffic_lights.distance];
    t_min      = zeros(1,n);
    t_max      = zeros(1,n);
    t_min(1)   = d(1)/v_max; 
    t_max(1)   = d(1)/v_min;
    t_min(1)   = next_green(traffic_lights(1), t_min(1));
    t_max(1)   = prev_green(traffic_lights(1), t_max(1));
    for i=2:n
        dist_inc   = d(i) - d(i-1);
        t_min(i)   = t_min(i-1) + dist_inc/v_max;
        t_max(i)   = t_max(i-1) + dist_inc/v_min;
        t_max(i)   = min(t_max(i), tf - (final_distance-d(i))/v_max);
        t_min(i)   = next_green(traffic_lights(i), t_min(i));
        t_max(i)   = prev_green(traffic_lights(i), t_max(i));
    end
    for i=n:-1:2
        needed_t = (d(i)-d(i-1))/v_max;
        if t_max(i)>t_max(i-1)+needed_t
            t_max(i-1) = t_max(i)-needed_t;
            t_max(i-1) = prev_green(traffic_lights(i-1), t_max(i-1));
        end
    end
end

function t_next = next_green(light, t)
    if is_green(light,t), t_next=t; return; end
    cyc = mod(t - light.offset, light.cycle_time);
    if cyc<light.green_start
        t_next = t + (light.green_start - cyc);
    else
        t_next = t + (light.cycle_time - cyc) + light.green_start;
    end
end

function t_prev = prev_green(light, t)
    if is_green(light,t), t_prev=t; return; end
    cyc = mod(t - light.offset, light.cycle_time);
    if cyc>=light.green_end
        t_prev = t-(cyc - light.green_end);
    else
        t_prev = t-cyc-(light.cycle_time - light.green_end);
    end
end

function [Nodes, Edges] = build_graph(traffic_lights, t_min, t_max, tf, final_distance, v_min, v_max, b1, b2)
    d            = [traffic_lights.distance];
    nIntersections = length(traffic_lights);
    Nodes        = struct('id',{},'t',{},'d',{},'int',{});
    nodeId       = 1;
    Nodes(nodeId)= struct('id',nodeId,'t',0,'d',0,'int',0); 
    nodeId       = nodeId+1;

    for i=1:nIntersections
        light = traffic_lights(i);
        for k=0:ceil(tf/light.cycle_time)
            cycle_start = k*light.cycle_time + light.offset;
            if light.green_start <= light.green_end
                sub_nodes = get_valid_times(light, cycle_start, t_min(i), t_max(i), tf);
                for kk=1:length(sub_nodes)
                    Nodes(nodeId)= struct('id',nodeId,'t',sub_nodes(kk),'d',d(i),'int',i);
                    nodeId=nodeId+1;
                end
            else
                sub_nodes1 = get_valid_times_wrap(light, cycle_start, t_min(i), t_max(i), tf);
                for kk=1:length(sub_nodes1)
                    Nodes(nodeId)=struct('id',nodeId,'t',sub_nodes1(kk),'d',d(i),'int',i);
                    nodeId=nodeId+1;
                end
            end
        end
    end
    Nodes(nodeId)=struct('id',nodeId,'t',tf,'d',final_distance,'int',nIntersections+1);
    
    Edges   = struct('from',{},'to',{},'w',{});
    nNodes  = length(Nodes);
    edgeCnt = 1;
    for i=1:nNodes
        lvlA=Nodes(i).int;
        for j=1:nNodes
            lvlB=Nodes(j).int;
            if lvlB==lvlA+1
                dt_ = Nodes(j).t - Nodes(i).t;
                dd_ = Nodes(j).d - Nodes(i).d;
                if (dt_>0) && (dd_>0)
                    if (Nodes(j).int>0 && Nodes(j).int<=nIntersections)
                        if ~is_green(traffic_lights(Nodes(j).int), Nodes(j).t), continue; end
                    end
                    v_link = dd_/dt_;
                    if v_link>=v_min && v_link<=v_max
                        E_link = dt_*(b1*v_link + b2*v_link^2);
                        Edges(edgeCnt)=struct('from',Nodes(i).id,'to',Nodes(j).id,'w',E_link);
                        edgeCnt=edgeCnt+1;
                    end
                end
            end
        end
    end
end

function vec = get_valid_times(light, cycle_start, t_min, t_max, tf)
    gS  = cycle_start + light.green_start;
    gE  = cycle_start + light.green_end;
    vec = [];
    if gS<=tf
        ov_start = max(gS, t_min);
        ov_end   = min(gE, t_max);
        if ov_start<ov_end
            mid_t = ceil((ov_start+ov_end)/2);
            vec   = [mid_t];
        end
    end
end

function vec = get_valid_times_wrap(light, cycle_start, t_min, t_max, tf)
    vec = [];
    gS1 = cycle_start + light.green_start;
    gE1 = cycle_start + light.cycle_time;
    if gS1<=tf
        ov_start= max(gS1, t_min);
        ov_end  = min(gE1,t_max);
        if ov_start<ov_end
            vec(end+1)= round((ov_start+ov_end)/2);
        end
    end
    gS2 = cycle_start;
    gE2 = cycle_start + light.green_end;
    if gE2<=tf
        ov_start= max(gS2,t_min);
        ov_end  = min(gE2,t_max);
        if ov_start<ov_end
            vec(end+1)= round((ov_start+ov_end)/2);
        end
    end
end

function [path, cost] = dijkstra(Nodes, Edges, source, target)
    nNodes = length(Nodes);
    cost   = inf(1,nNodes);
    prev   = nan(1,nNodes);
    cost(source)=0;
    Q = 1:nNodes;
    while ~isempty(Q)
        [~, idx] = min(cost(Q));
        u = Q(idx);
        Q(Q==u) = [];
        if u==target, break; end
        for e=Edges
            if e.from==u
                v   = e.to;
                alt = cost(u)+e.w;
                if alt<cost(v)
                    cost(v)=alt; prev(v)=u;
                end
            end
        end
    end
    path=[];
    if ~isnan(prev(target)) || target==source
        u=target;
        while ~isnan(u)
            path=[u,path];
            u=prev(u);
        end
    end
end

function result = is_in_current_platoon(vehicle_id, current_run)
    global SIM_RUNS
    result = false;
    if current_run==1
        if isfield(SIM_RUNS{1},'splittedVehicles') && ~isempty(SIM_RUNS{1}.splittedVehicles) ...
           && ismember(vehicle_id, SIM_RUNS{1}.splittedVehicles)
            result = false;
        else
            result = true;
        end
        return;
    end
    prev_run = SIM_RUNS{current_run-1};
    if isfield(prev_run,'splittedVehicles') && ~isempty(prev_run.splittedVehicles) ...
       && ismember(vehicle_id,prev_run.splittedVehicles)
        if isfield(SIM_RUNS{current_run},'splittedVehicles') ...
           && ~isempty(SIM_RUNS{current_run}.splittedVehicles) ...
           && ismember(vehicle_id,SIM_RUNS{current_run}.splittedVehicles)
            result=false;
        else
            result=true;
        end
    end
end

function vt = get_current_v_target_indexed(x_leader, traffic_lights, v_targets)
    idx = find(x_leader < [traffic_lights.distance], 1);
    if isempty(idx)
        vt = v_targets(end);
    else
        idx = min(idx,length(v_targets));
        vt  = v_targets(idx);
    end
end

function trigger_events = velocity_trigger(t, diff_v, opt_v)
    trigger_threshold  = 5;
    disable_duration   = 10;
    rapid_change_thresh= 0.5;
    initial_delay      = 0;
    trigger_events     = zeros(size(t));
    disable_until      = -inf;
    for i=1:length(t)
        if t(i)<initial_delay || t(i)<disable_until
            trigger_events(i)=0;
        else
            if abs(diff_v(i))>trigger_threshold
                trigger_events(i)=1;
            else
                trigger_events(i)=0;
            end
        end
        if i>1
            delta_opt = abs(opt_v(i)-opt_v(i-1));
            if delta_opt>rapid_change_thresh
                disable_until = t(i)+disable_duration;
            end
        end
    end
end


%% ---------------------- PLOT FUNCTIONS ---------------------------------
function final_plot()
    global SIM_RUNS
    if isempty(SIM_RUNS), disp('[final_plot] Nessun dato.'); return; end
    plot_optimal_trajectories_and_lights();
    plot_real_trajectories();
    plot_comparison();
    plot_leader_velocities();
    plot_energy_consumption();
    plot_inter_vehicle_distances();
end

function plot_delta_velocities()
    global SIM_RUNS
    if isempty(SIM_RUNS), disp('Nessun dato per plot dei delta_vel.'); return; end
    runData    = SIM_RUNS{1};
    t_sim      = runData.t;
    n_vehicles = size(runData.x,2)/2;
    figure('Name','Delta Velocità','Position',[100,100,1200,800]);
    for v=1:n_vehicles
        v_sim = runData.x(:, n_vehicles+v);
        if ~isfield(runData,'opt_t')||~isfield(runData,'opt_d'), continue; end
        opt_t  = runData.opt_t;
        opt_d  = runData.opt_d;
        pos_opt= interp1(opt_t,opt_d,t_sim,'linear','extrap');
        v_opt  = gradient(pos_opt,t_sim);
        offset_value = v_opt(end) - v_sim(end);
        diff_v       = (v_opt - v_sim) - offset_value;
        
        subplot(n_vehicles,1,v); hold on; grid on;
        plot(t_sim, v_sim, 'b-', 'LineWidth',1.5);
        plot(t_sim, v_opt, 'g--','LineWidth',1.5);
        plot(t_sim, diff_v,'r-.','LineWidth',1.5);
        xlabel('Tempo [s]'); ylabel('Velocità [m/s]');
        title(['Veicolo ' num2str(v)]);
    end
end

function plot_velocity_trigger_per_vehicle()
    global SIM_RUNS
    if isempty(SIM_RUNS), disp('Nessun dato per i trigger.'); return; end
    figure('Name','Trigger Velocità','Position',[100,100,1200,800]);
    runData    = SIM_RUNS{1};
    t_sim      = runData.t;
    n_vehicles = size(runData.x,2)/2;
    [split_times, split_vehicles, split_reasons] = find_splits_info();
    
    for v=1:n_vehicles
        v_sim = runData.x(:, n_vehicles+v);
        if ~isfield(runData,'opt_t')||~isfield(runData,'opt_d'), continue; end
        opt_t  = runData.opt_t;
        opt_d  = runData.opt_d;
        pos_opt= interp1(opt_t,opt_d,t_sim,'linear','extrap');
        v_opt  = gradient(pos_opt,t_sim);
        offset_value = v_opt(end)-v_sim(end);
        diff_v       = (v_opt-v_sim)-offset_value;
        trig_state   = velocity_trigger(t_sim,diff_v,v_opt);

        subplot(n_vehicles,1,v); hold on; grid on;
        plot(t_sim,v_sim,'b-','LineWidth',1.5);
        plot(t_sim,v_opt,'g--','LineWidth',1.5);
        plot(t_sim,diff_v,'r-.','LineWidth',1.5);
        plot(t_sim,trig_state*max(abs(diff_v))*0.5,'m-','LineWidth',2);

        y_lim = get(gca,'YLim');
        for idx=1:length(split_times)
            if split_vehicles(idx)==v
                line([split_times(idx),split_times(idx)], [y_lim(1),y_lim(2)], ...
                     'Color','k','LineWidth',2);
                text(split_times(idx), y_lim(2)*0.8, ['SPLIT ',split_reasons{idx}], ...
                     'Color','r','FontWeight','bold');
            end
        end
        xlabel('Tempo [s]'); ylabel('Velocità [m/s]');
        title(['Veicolo ',num2str(v)]);
    end
end

function [split_times, split_vehicles, split_reasons] = find_splits_info()
    global SIM_RUNS
    split_times    = [];
    split_vehicles = [];
    split_reasons  = {};
    for i=1:length(SIM_RUNS)-1
        if isfield(SIM_RUNS{i},'splittedVehicles') && ~isempty(SIM_RUNS{i}.splittedVehicles)
            next_leader  = SIM_RUNS{i+1}.leader;
            split_time   = SIM_RUNS{i+1}.offset;
            diff_time    = split_time - (i-1)*30;
            split_times   = [split_times, split_time];
            split_vehicles= [split_vehicles, next_leader];
            if abs(diff_time)<1e-3
                split_reasons{end+1}='Semaforo';
            else
                split_reasons{end+1}='Trigger';
            end
        end
    end
end

function plot_optimal_trajectories_and_lights()
    global SIM_RUNS
    figure('Name','Traiettorie Ottimali','Position',[100,100,900,600]); hold on;
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    [all_times, all_d, all_c] = prepare_traffic_light_data(traffic_lights);
    scatter(all_times,all_d,10,all_c,'filled');
    markers     = {'o','s','d','^','v'};
    colors      = {'b','r','g','m','c','k'};
    line_styles = {'-','--',':','-.'};
    Lh          = [];
    Lnames      = {};

    for run_i=1:length(SIM_RUNS)
        runData=SIM_RUNS{run_i};
        if ~isfield(runData,'opt_t')||~isfield(runData,'opt_d'), continue; end
        opt_t=runData.opt_t; opt_d=runData.opt_d;
        color_idx=mod(runData.leader-1,length(colors))+1;
        style_idx=mod(run_i-1,length(line_styles))+1;
        mk_idx   = mod(run_i-1,length(markers))+1;
        h=plot(opt_t,opt_d,[colors{color_idx},line_styles{style_idx}],'LineWidth',3);
        scatter(opt_t,opt_d,40,colors{color_idx},markers{mk_idx},'filled');
        Lh(end+1)=h; %#ok<AGROW>
        Lnames{end+1} = ['Leader ',num2str(runData.leader),'(P',num2str(run_i),')']; %#ok<AGROW>
    end
    legend(Lh,Lnames,'Location','Best');
    xlabel('Tempo [s]'); ylabel('Posizione [m]');
    title('Traiettorie Ottimali + Semafori'); grid on;
end

function plot_real_trajectories()
    global SIM_RUNS
    figure('Name','Traiettorie Reali','Position',[150,150,900,600]); hold on;
    traffic_lights=SIM_RUNS{1}.traffic_lights;
    [all_times,all_d,all_c]=prepare_traffic_light_data(traffic_lights);
    scatter(all_times,all_d,10,all_c,'filled');
    colors      = {'b','r','g','m','c','y','k'};
    line_styles = {'-','--',':','-.'};
    Lh=[]; Lnames={};
    for run_i=1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t=runData.t; x=runData.x; 
        n_vehicles=size(x,2)/2;
        plt_v = get_platoon_vehicles(run_i);
        for v=plt_v
            c_idx=mod(v-1,length(colors))+1;
            s_idx=mod(run_i-1,length(line_styles))+1;
            h=plot(t,x(:,v),[colors{c_idx},line_styles{s_idx}],'LineWidth',2);
            if v==runData.leader
                Lnames{end+1}=['Leader ', num2str(v),' P',num2str(run_i)]; %#ok<AGROW>
            else
                Lnames{end+1}=['Veicolo ',num2str(v),' P',num2str(run_i)]; %#ok<AGROW>
            end
            Lh(end+1)=h; %#ok<AGROW>
        end
    end
    legend(Lh,Lnames,'Location','Best','NumColumns',2);
    xlabel('Tempo [s]'); ylabel('Posizione [m]');
    title('Traiettorie Reali + Semafori'); grid on;
end

function plot_comparison()
    global SIM_RUNS
    figure('Name','Confronto Reale vs Ottimale','Position',[200,200,1000,600]); hold on;
    traffic_lights=SIM_RUNS{1}.traffic_lights;
    [all_times,all_d,all_c]=prepare_traffic_light_data(traffic_lights);
    scatter(all_times,all_d,10,all_c,'filled');
    colors_real = {'b','r','g','m','c','y','k'};
    colors_opt  = {'b','r','g','m','c','k',[0.8,0.4,0],[0.5,0.5,0.5]};
    Lh=[];Lnames={};
    for run_i=1:length(SIM_RUNS)
        runData=SIM_RUNS{run_i};
        t=runData.t; x=runData.x; n_vehicles=size(x,2)/2;
        plt_v = get_platoon_vehicles(run_i);
        % reals
        for v=plt_v
            c_idx=mod(v-1,length(colors_real))+1;
            h_real=plot(t,x(:,v),[colors_real{c_idx},'-'],'LineWidth',2);
            if v==runData.leader
                Lnames{end+1}=['Leader ',num2str(v),'(reale,P',num2str(run_i),')']; 
            else
                Lnames{end+1}=['V',num2str(v),'(reale,P',num2str(run_i),')'];
            end
            Lh(end+1)=h_real;
        end
        % ottimali
        if isfield(runData,'opt_t') && isfield(runData,'opt_d')
            opt_t=runData.opt_t; opt_d=runData.opt_d;
            c_idx=mod(runData.leader-1,length(colors_opt))+1;
            h_opt=plot(opt_t,opt_d,[colors_opt{c_idx},'--'],'LineWidth',2);
            Lnames{end+1}=['Leader ',num2str(runData.leader),'(opt,P',num2str(run_i),')']; 
            Lh(end+1)=h_opt;
        end
    end
    legend(Lh,Lnames,'Location','Best','NumColumns',2);
    xlabel('Tempo [s]'); ylabel('Posizione [m]');
    title('Confronto Reale vs Ottimale + Semafori'); grid on;
end

function plot_leader_velocities()
    global SIM_RUNS
    leaders=[];
    for i=1:length(SIM_RUNS)
        if ~ismember(SIM_RUNS{i}.leader,leaders)
            leaders=[leaders, SIM_RUNS{i}.leader];
        end
    end
    for L=leaders
        figure('Name',['Velocità Leader ',num2str(L)],...
               'Position',[250+L*30,250+L*30,800,500]); hold on;
        for run_i=1:length(SIM_RUNS)
            rd=SIM_RUNS{run_i};
            if rd.leader==L
                t=rd.t; x=rd.x; nv=size(x,2)/2;
                v_lead= x(:,nv+L);
                plot(t,v_lead,'b-','LineWidth',2);
                if isfield(rd,'opt_t') && isfield(rd,'opt_d')
                    opt_t=rd.opt_t; opt_d=rd.opt_d;
                    opt_v=calc_opt_vel(opt_t,opt_d);
                    for kk=1:length(opt_v)
                        if kk<length(opt_v)
                            plot([opt_t(kk),opt_t(kk+1)],[opt_v(kk),opt_v(kk)],'r--','LineWidth',2);
                        end
                    end
                end
            end
        end
        xlabel('Tempo [s]'); ylabel('Velocità [m/s]');
        title(['Leader ',num2str(L)]); grid on; ylim([0,35]);
    end
end

function plot_energy_consumption()
    global SIM_RUNS
    figure('Name','Consumo Energetico','Position',[300,300,800,500]); hold on;
    platoon_energy = zeros(1,length(SIM_RUNS));
    for run_i=1:length(SIM_RUNS)
        rd=SIM_RUNS{run_i};
        t=rd.t; x=rd.x;
        n_veh=size(x,2)/2;
        b1=0.1; b2=0.01; tot_E=0;
        plist=get_platoon_vehicles(run_i);
        for v=plist
            vel_v = x(:, n_veh+v);
            dt=diff(t);
            segE=sum(dt.*(b1*vel_v(1:end-1)+b2*vel_v(1:end-1).^2));
            tot_E=tot_E+segE;
        end
        platoon_energy(run_i)=tot_E;
    end
    bar(1:length(SIM_RUNS), platoon_energy);
    xlabel('Indice Plotone'); ylabel('Consumo Energetico [J]');
    title('Consumo Energetico Plotoni'); grid on;
end

function plot_inter_vehicle_distances()
    global SIM_RUNS
    figure('Name','Distanze tra Veicoli','Position',[350,350,1000,600]);
    n_plots=length(SIM_RUNS);
    rows=ceil(sqrt(n_plots)); cols=ceil(n_plots/rows);
    for run_i=1:n_plots
        subplot(rows,cols,run_i); hold on; grid on;
        rd=SIM_RUNS{run_i};
        t=rd.t; x=rd.x;
        plt_v=get_platoon_vehicles(run_i);
        if length(plt_v)<2
            text(0.5,0.5,'Plotone con 1 veicolo','HorizontalAlignment','center');
            continue;
        end
        [~,ix]=sort(x(end,plt_v),'descend');
        sorted_v=plt_v(ix);
        for i=1:length(sorted_v)-1
            dist=x(:,sorted_v(i)) - x(:,sorted_v(i+1));
            plot(t,dist,'LineWidth',2);
        end
        title(['Plotone ',num2str(run_i)]);
        xlabel('Tempo [s]'); ylabel('Distanza [m]');
    end
end

function [all_times, all_distances, all_colors] = prepare_traffic_light_data(traffic_lights)
    global SIM_RUNS
    if isempty(SIM_RUNS), max_time=150; else
        max_time = 0;
        for i=1:length(SIM_RUNS)
            mt = max(SIM_RUNS{i}.t);
            if mt>max_time, max_time=mt; end
        end
    end
    times=0:ceil(max_time);
    all_times=[]; all_distances=[]; all_colors=[];
    for i=1:length(traffic_lights)
        for j=1:length(times)
            time=times(j);
            all_times(end+1)=time;
            all_distances(end+1)=traffic_lights(i).distance;
            if is_green(traffic_lights(i),time), all_colors(end+1,:)= [0,1,0];
            else,                                all_colors(end+1,:)= [1,0,0];
            end
        end
    end
end

function v_arr = calc_opt_vel(opt_t,opt_d)
    v_arr = zeros(1,length(opt_t)-1);
    for i=1:length(opt_t)-1
        v_arr(i)=(opt_d(i+1)-opt_d(i))/(opt_t(i+1)-opt_t(i));
    end
end

function platoon_vehicles = get_platoon_vehicles(run_i)
    global SIM_RUNS
    n_vehicles=size(SIM_RUNS{1}.x,2)/2;
    if run_i==1
        platoon_vehicles=1:n_vehicles;
        if isfield(SIM_RUNS{1},'splittedVehicles') && ~isempty(SIM_RUNS{1}.splittedVehicles)
            platoon_vehicles=setdiff(platoon_vehicles, SIM_RUNS{1}.splittedVehicles);
        end
        return;
    end
    prev_run=SIM_RUNS{run_i-1};
    if isfield(prev_run,'splittedVehicles') && ~isempty(prev_run.splittedVehicles)
        platoon_vehicles=prev_run.splittedVehicles;
        if isfield(SIM_RUNS{run_i},'splittedVehicles') && ~isempty(SIM_RUNS{run_i}.splittedVehicles)
            platoon_vehicles = setdiff(platoon_vehicles,SIM_RUNS{run_i}.splittedVehicles);
        end
    else
        platoon_vehicles=[];
    end
end