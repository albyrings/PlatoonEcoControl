%% platoon_through_lights.m
clear; clc; close all;

%% ——————————————————————————————————————————————
%% Configurazione iniziale
global SIM_RUNS
SIM_RUNS = {};

T_cycle      = 30;    % lunghezza ciclo semafori
initial_leader = 1;   % leader del plotone iniziale

leader      = initial_leader;
time_offset = 0;

%% ——————————————————————————————————————————————
%% Main loop: simulo → cerco rosso → se c'è split → ripeto
while true
    fprintf('=== Simulazione leader=%d, offset=%.2f ===\n', leader, time_offset);
    run_optimizer_and_plot(leader, time_offset);
    
    lastRun = SIM_RUNS{end};
    [v_bad, t_bad] = detect_first_red(lastRun.t, lastRun.x, lastRun.traffic_lights);
    if isempty(v_bad)
        fprintf('Nessuna violazione rimanente: fine simulazioni.\n');
        break;
    end
    
    % registro quali veicoli si staccano
    nVeh = size(lastRun.x,2)/2;
    SIM_RUNS{end}.splittedVehicles = v_bad:nVeh;
    
    fprintf('>> Veicolo %d passa col rosso a t=%.2f → nuovo leader!\n', v_bad, t_bad);
    leader      = v_bad;
    time_offset = t_bad;
end

%% plottaggio finale
final_plot();



%% =========================================================================
%% Funzione principale di ottimizzazione + simulazione
%% =========================================================================
function run_optimizer_and_plot(leader_vehicle, time_offset)
    global SIM_RUNS
    
    % svuoto i persistent interni all’ODE
    clear system_dynamics_new_platoon
    
    % parametri di simulazione
    final_time     = 150;
    final_distance = 1800;
    T              = 30;
    tf             = final_time;
    v_min          = 5;
    v_max          = 30;
    b1             = 0.1;
    b2             = 0.01;
    
    fprintf('\n[INFO] run_optimizer_and_plot(Leader=%d, offset=%.2f)\n', ...
            leader_vehicle, time_offset);
    
    % definisco i semafori
    traffic_lights = [
        create_traffic_light(300,   0, 10, T)
        create_traffic_light(600,  10, 20, T)
        create_traffic_light(900,  20, 30, T)
        create_traffic_light(1200,  0, 10, T)
        create_traffic_light(1550, 10, 20, T)
    ];
    
    % pruning dei tempi possibili
    [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance];
    nIntersections = length(traffic_lights);
    
    % costruisco i nodi di green windows
    Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
    nodeId = 1;
    Nodes(nodeId) = struct('id',nodeId,'t',0,'d',0,'int',0); nodeId = nodeId+1;
    for i=1:nIntersections
        light = traffic_lights(i);
        for k=0:ceil(tf/light.cycle_time)
            cycle_start = k*light.cycle_time + light.offset;
            if light.green_start <= light.green_end
                abs_gs = cycle_start + light.green_start;
                abs_ge = cycle_start + light.green_end;
                if abs_gs <= tf
                    ov_s = max(abs_gs, t_min(i));
                    ov_e = min(abs_ge, t_max(i));
                    if ov_s<ov_e
                        mid = ceil((ov_s+ov_e)/2);
                        Nodes(nodeId) = struct('id',nodeId,'t',mid,'d',d(i),'int',i);
                        nodeId = nodeId+1;
                    end
                end
            else
                % caso green wrap-around
                abs_gs1 = cycle_start + light.green_start;
                abs_ge1 = cycle_start + light.cycle_time;
                if abs_gs1<=tf
                    ov_s = max(abs_gs1, t_min(i));
                    ov_e = min(abs_ge1, t_max(i));
                    if ov_s<ov_e
                        mid = (ov_s+ov_e)/2;
                        Nodes(nodeId)=struct('id',nodeId,'t',mid,'d',d(i),'int',i);
                        nodeId=nodeId+1;
                    end
                end
                abs_gs2 = cycle_start;
                abs_ge2 = cycle_start + light.green_end;
                if abs_ge2<=tf
                    ov_s = max(abs_gs2, t_min(i));
                    ov_e = min(abs_ge2, t_max(i));
                    if ov_s<ov_e
                        mid = (ov_s+ov_e)/2;
                        Nodes(nodeId)=struct('id',nodeId,'t',mid,'d',d(i),'int',i);
                        nodeId=nodeId+1;
                    end
                end
            end
        end
    end
    % nodo di arrivo
    Nodes(nodeId)=struct('id',nodeId,'t',tf,'d',final_distance,'int',nIntersections+1);
    nNodes = nodeId;
    
    % costruisco gli archi validi
    Edges = struct('from',{},'to',{},'w',{});
    eC=1;
    for i=1:nNodes
      for j=1:nNodes
        if Nodes(j).int == Nodes(i).int+1 && ...
           Nodes(j).t>Nodes(i).t && Nodes(j).d>Nodes(i).d
          % se non è verde al nodo j scarto
          if Nodes(j).int<=nIntersections && ...
             ~is_green(traffic_lights(Nodes(j).int), Nodes(j).t)
              continue; 
          end
          dt = Nodes(j).t - Nodes(i).t;
          dd = Nodes(j).d - Nodes(i).d;
          v  = dd/dt;
          if v>=v_min && v<=v_max
            cost = dt*(b1*v + b2*v^2);
            Edges(eC) = struct('from',Nodes(i).id,'to',Nodes(j).id,'w',cost);
            eC = eC+1;
          end
        end
      end
    end
    
    % cerco cammino minimo
    [path, cost] = dijkstra(Nodes,Edges,1,nNodes);
    fprintf('>>> Leader=%d, costo ottimo=%.3f\n', leader_vehicle, cost);
    opt_nodes = Nodes(path);
    opt_t = [opt_nodes.t];
    opt_d = [opt_nodes.d];
    
    % ricavo speeds
    speeds = diff(opt_d)./diff(opt_t);
    
    % parametri platoon + PID
    n_vehicles = 1;
    m_vehicles = 1000*ones(1,n_vehicles);
    v_targets = speeds;
    K_p_speed=7000; K_i_speed=0; K_d_speed=0.1;
    K_p_dist=2000;  K_i_dist=0.8; K_d_dist=0.4;
    t_CTH = 1.5;  d_init = 4;
    
    x0 = zeros(2*n_vehicles,1);
    for i=1:n_vehicles
      x0(i) = -d_init*(i-1);
    end
    
    t_span = [0 final_time];
    [t_sim,x_sim] = ode45(@(t,x) system_dynamics_new_platoon( ...
        t,x,n_vehicles,m_vehicles,@(tt)0,traffic_lights, ...
        v_targets,t_CTH, ...
        K_p_speed,K_i_speed,K_d_speed, ...
        K_p_dist,K_i_dist,K_d_dist, ...
        leader_vehicle,time_offset), ...
      t_span, x0);
    
    T_abs = t_sim + time_offset;
    SIM_RUNS{end+1} = struct( ...
      'leader',        leader_vehicle, ...
      't',             T_abs, ...
      'x',             x_sim, ...
      'offset',        time_offset, ...
      'traffic_lights',traffic_lights, ...
      'v_targets',     speeds, ...
      'opt_t',         opt_t+time_offset, ...
      'opt_d',         opt_d ...
    );
end


%% =========================================================================
%% Trova la prima violazione rosso
%% =========================================================================
function [violating_vehicle, t_violation] = detect_first_red(t_abs, x_sim, traffic_lights)
    violating_vehicle = [];
    t_violation       = [];
    n_veh = size(x_sim,2)/2;
    for v=1:n_veh
      pos_v = x_sim(:,v);
      for L=1:length(traffic_lights)
        dL = traffic_lights(L).distance;
        ix = find(pos_v(1:end-1)<dL & pos_v(2:end)>=dL,1);
        if ~isempty(ix)
          tc = t_abs(ix);
          if ~is_green(traffic_lights(L),tc)
            violating_vehicle = v;
            t_violation       = tc;
            return
          end
        end
      end
    end
end


%% =========================================================================
%% Sistema dinamico del plotone (leader + follower)
%% =========================================================================
function dx = system_dynamics_new_platoon(t, x, n_veh, m, delta_func, ...
    traffic_lights, v_targets, t_CTH, ...
    Kp_s, Ki_s, Kd_s, Kp_d, Ki_d, Kd_d, ...
    leader_vehicle, time_offset)

    dx = zeros(2*n_veh,1);
    persistent t_prev e_int_speed e_old_speed e_int_dist e_old_dist
    if isempty(t_prev),       t_prev = t;     end
    if isempty(e_int_speed),  e_int_speed = 0; e_old_speed = 0; end
    if isempty(e_int_dist),   e_int_dist = zeros(n_veh,1); e_old_dist = zeros(n_veh,1); end

    dt = max(t - t_prev, 1e-6);
    t_prev = t;
    abs_t  = t + time_offset;

    for i=1:n_veh
      dx(i) = x(n_veh+i);
      if i==leader_vehicle
        vt = get_current_v_target_indexed(x(i),traffic_lights,v_targets);
        err = vt - x(n_veh+i);
        e_int_speed = e_int_speed + err*dt;
        derr = (err - e_old_speed)/dt;
        e_old_speed = err;
        U = Kp_s*err + Ki_s*e_int_speed + Kd_s*derr + delta_func(abs_t);
        dv = U/m(i);
      else
        if i>1
          dist = x(i-1)-x(i);
        else
          dist = 10;
        end
        vcur = x(n_veh+i);
        d_des = 1 + t_CTH*vcur;
        err = dist - d_des;
        e_int_dist(i) = e_int_dist(i) + err*dt;
        derr = (err - e_old_dist(i))/dt;
        e_old_dist(i) = err;
        U = Kp_d*err + Ki_d*e_int_dist(i) + Kd_d*derr;
        dv = U/m(i);
      end
      new_v = x(n_veh+i)+dv*dt;
      new_v = max(new_v,0);
      dx(n_veh+i) = (new_v - x(n_veh+i))/dt;
    end
end


%% =========================================================================
%% Seleziona target velocity in base alla prossima luce
%% =========================================================================
function vt = get_current_v_target_indexed(x_leader, traffic_lights, v_targets)
    idx = find(x_leader < [traffic_lights.distance],1);
    if isempty(idx)
      vt = v_targets(end);
    else
      vt = v_targets(min(idx,length(v_targets)));
    end
end


%% =========================================================================
%% Costruzione semaforo
%% =========================================================================
function light = create_traffic_light(distance, green_start, green_end, cycle_time)
    light.distance       = distance;
    light.green_start    = green_start;
    light.green_end      = green_end;
    light.cycle_time     = cycle_time;
    light.offset         = mod(green_start, cycle_time);
end

function st = is_green(light, time)
    tc = mod(time - light.offset, light.cycle_time);
    if light.green_start <= light.green_end
      st = (tc >= light.green_start && tc < light.green_end);
    else
      st = (tc >= light.green_start || tc < light.green_end);
    end
end


%% =========================================================================
%% Pruning dei tempi con next/prev green
%% =========================================================================
function [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_d, v_min, v_max)
    n = length(traffic_lights);
    d = [traffic_lights.distance];
    t_min = zeros(1,n); t_max = zeros(1,n);
    t_min(1)= next_green(traffic_lights(1), d(1)/v_max);
    t_max(1)= prev_green(traffic_lights(1), d(1)/v_min);
    for i=2:n
      dd = d(i)-d(i-1);
      t_min(i)= t_min(i-1)+ dd/v_max;
      t_max(i)= min(t_max(i-1)+ dd/v_min, tf - (final_d-d(i))/v_max);
      t_min(i)= next_green(traffic_lights(i), t_min(i));
      t_max(i)= prev_green(traffic_lights(i), t_max(i));
    end
    for i=n:-1:2
      needed = (d(i)-d(i-1))/v_max;
      if t_max(i) > t_max(i-1)+needed
        t_max(i-1)= prev_green(traffic_lights(i-1), t_max(i)-needed);
      end
    end
end

function t2 = next_green(light, t1)
    if is_green(light,t1), t2=t1; return; end
    c = mod(t1-light.offset, light.cycle_time);
    if c < light.green_start
      t2 = t1 + (light.green_start - c);
    else
      t2 = t1 + (light.cycle_time - c) + light.green_start;
    end
end

function t2 = prev_green(light, t1)
    if is_green(light,t1), t2=t1; return; end
    c = mod(t1-light.offset, light.cycle_time);
    if c >= light.green_end
      t2 = t1 - (c - light.green_end);
    else
      t2 = t1 - c - (light.cycle_time - light.green_end);
    end
end


%% =========================================================================
%% Dijkstra minimal‐cost path
%% =========================================================================
function [path, cost] = dijkstra(Nodes, Edges, source, target)
    nN = numel(Nodes);
    cost = inf(1,nN);
    prev = nan(1,nN);
    cost(source)=0;
    Q = 1:nN;
    while ~isempty(Q)
      [~,i] = min(cost(Q)); u = Q(i);
      Q(i)=[];
      if u==target, break; end
      for e = Edges
        if e.from==u
          alt = cost(u)+e.w;
          if alt<cost(e.to)
            cost(e.to)=alt;
            prev(e.to)=u;
          end
        end
      end
    end
    % ricostruisco path
    path = [];
    u = target;
    while ~isnan(u)
      path = [u path];
      u = prev(u);
    end
end


%% =========================================================================
%% Plottaggio finale
%% =========================================================================

function final_plot()
    global SIM_RUNS

    % 1) Filter only sub-simulations (offset>0)
    isSub    = cellfun(@(r) r.offset>0, SIM_RUNS);
    subRuns  = SIM_RUNS(isSub);
    if isempty(subRuns)
        disp('[final_plot] Nessuna sotto-simulazione da plottare.');
        return;
    end

    % 2) Extract traffic lights once
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    nLights       = numel(traffic_lights);

    % 3) Build scatter data for lights
    max_t = max(cellfun(@(r) max(r.t), subRuns));
    times = 0:ceil(max_t);
    NT    = numel(times);

    % Preallocate
    all_times  = repmat(times, 1, nLights);
    all_dist   = reshape(repmat([traffic_lights.distance], NT, 1), 1, []);
    all_colors = zeros(NT * nLights, 3);

    idx = 1;
    for i = 1:nLights
        for tt = times
            all_colors(idx,:) = is_green(traffic_lights(i), tt) * [0 1 0] ...
                              + ~is_green(traffic_lights(i), tt)* [1 0 0];
            idx = idx + 1;
        end
    end

    %% ——————————————————————————————————————————————————————
    %% Fig.1: Optimal trajectories + lights (only subRuns)
    %% ——————————————————————————————————————————————————————
    figure('Name','Traiettorie Ottimali e Semafori (sotto-simulazioni)');
    hold on;
    scatter(all_times, all_dist, 10, all_colors, 'filled', 'DisplayName','Semafori');

    markers     = {'o','s','d','^','v','>','<'};
    colors      = {'b','r','g','m','c','k',[0.8 0.4 0],[0.5 0.5 0.5],[0.2 0.6 0.8]};
    line_styles = {'-','--',':','-.'};

    legend_handles = gobjects(0);
    legend_texts   = {};

    for k = 1:numel(subRuns)
        runData = subRuns{k};
        nVeh    = size(runData.x,2)/2;

        % previous platoon members
        if k==1
            if isfield(SIM_RUNS{1},'splittedVehicles')
                prevPlatoon = SIM_RUNS{1}.splittedVehicles;
            else
                prevPlatoon = 1:nVeh;
            end
        else
            prevRun = subRuns{k-1};
            if isfield(prevRun,'splittedVehicles')
                prevPlatoon = prevRun.splittedVehicles;
            else
                prevPlatoon = 1:nVeh;
            end
        end

        % current split list
        if isfield(runData,'splittedVehicles')
            splits = runData.splittedVehicles;
        else
            splits = [];
        end

        % leader’s optimal path
        opt_t = runData.opt_t;
        opt_d = runData.opt_d;
        if isempty(opt_t)
            continue
        end
        leader = runData.leader;
        ci = mod(leader-1,numel(colors))+1;
        li = mod(k-1,numel(line_styles))+1;
        mi = mod(k-1,numel(markers))+1;

        h = plot(opt_t, opt_d, [colors{ci} line_styles{li}], 'LineWidth',3);
        scatter(opt_t, opt_d, 70, colors{ci}, markers{mi}, 'filled');

        legend_handles(end+1) = h; %#ok<AGROW>
        legend_texts{end+1}   = sprintf('Leader %d (Plotone %d)', leader, k);

        % followers’ optimal offsets
        v_targets = diff(opt_d)./diff(opt_t);
        t_CTH = 1.5; d_min = 1;
        for v = setdiff(prevPlatoon, splits)
            if v==leader, continue; end
            foll_d = zeros(size(opt_d));
            sign_  = sign(v - leader);
            foll_d(1) = opt_d(1) + sign_*abs(v-leader);
            for i=2:numel(opt_t)
                gap = d_min + t_CTH*v_targets(i-1);
                foll_d(i) = opt_d(i) + sign_*gap*(leader-v);
            end
            cfi = mod(v-1,numel(colors))+1;
            h2  = plot(opt_t, foll_d, [colors{cfi} line_styles{li}], 'LineWidth',2);
            scatter(opt_t, foll_d, 40, colors{cfi}, markers{mi}, 'filled');
            legend_handles(end+1) = h2; %#ok<AGROW>
            legend_texts{end+1}   = sprintf('Follower %d (Plotone %d)', v, k);
        end
    end

    legend(legend_handles, legend_texts, 'Location','Best');
    xlabel('Tempo [s]'); ylabel('Posizione [m]');
    title('Traiettorie ottimali e semafori');
    grid on;
    hold off;


    %% ——————————————————————————————————————————————————————
    %% Fig.2: Real trajectories (only subRuns)
    %% ——————————————————————————————————————————————————————
    figure('Name','Traiettorie Reali (sotto-simulazioni)');
    hold on;
    scatter(all_times, all_dist, 10, all_colors, 'filled');

    colors2      = {'b','r','g','m','c','y','k'};
    line_styles2 = {'-','-','--',':','-.'};
    drawnVeh = [];

    for k = 1:numel(subRuns)
        runData = subRuns{k};
        nVeh    = size(runData.x,2)/2;
        if isfield(runData,'splittedVehicles')
            skips = runData.splittedVehicles;
        else
            skips = [];
        end
        for v=1:nVeh
            if ismember(v,skips) || ismember(v,drawnVeh)
                continue
            end
            ci = mod(v-1,numel(colors2))+1;
            li = mod(k-1,numel(line_styles2))+1;
            plot(runData.t, runData.x(:,v), [colors2{ci} line_styles2{li}], 'LineWidth',2);
            drawnVeh(end+1) = v; %#ok<AGROW>
        end
    end

    xlabel('Tempo [s]'); ylabel('Posizione [m]');
    title('Traiettorie reali e semafori');
    grid on;
    hold off;


    %% ——————————————————————————————————————————————————————
    %% Fig.3: Real vs Optimal (only subRuns)
    %% ——————————————————————————————————————————————————————
    figure('Name','Confronto Reale vs Ottimali (sotto-simulazioni)');
    hold on;
    scatter(all_times, all_dist, 10, all_colors, 'filled');

    % plot real
    drawnVeh = [];
    for k = 1:numel(subRuns)
        runData = subRuns{k};
        nVeh    = size(runData.x,2)/2;
        if isfield(runData,'splittedVehicles')
            skips = runData.splittedVehicles;
        else
            skips = [];
        end
        for v=1:nVeh
            if ismember(v,skips) || ismember(v,drawnVeh)
                continue
            end
            ci = mod(v-1,numel(colors2))+1;
            plot(runData.t, runData.x(:,v), [colors2{ci} '-'], 'LineWidth',2);
            drawnVeh(end+1) = v; %#ok<AGROW>
        end
    end

    % plot optimal
    for k = 1:numel(subRuns)
        runData = subRuns{k};
        if isempty(runData.opt_t), continue; end
        opt_t = runData.opt_t; opt_d = runData.opt_d;
        nVeh  = size(runData.x,2)/2;

        % previous platoon
        if k==1
            if isfield(SIM_RUNS{1},'splittedVehicles')
                prevPlatoon = SIM_RUNS{1}.splittedVehicles;
            else
                prevPlatoon = 1:nVeh;
            end
        else
            prevRun = subRuns{k-1};
            if isfield(prevRun,'splittedVehicles')
                prevPlatoon = prevRun.splittedVehicles;
            else
                prevPlatoon = 1:nVeh;
            end
        end
        if isfield(runData,'splittedVehicles')
            splits = runData.splittedVehicles;
        else
            splits = [];
        end
        stays = setdiff(prevPlatoon, splits);

        v_targets = diff(opt_d)./diff(opt_t);
        for v = stays
            ci = mod(v-1,numel(colors2))+1;
            if v == runData.leader
                plot(opt_t, opt_d, [colors2{ci} '--'], 'LineWidth',2);
            else
                foll_d = zeros(size(opt_d));
                sign_  = sign(v - runData.leader);
                foll_d(1) = opt_d(1) + sign_*abs(v-runData.leader);
                for i=2:numel(opt_t)
                    gap = 1 + 1.5*v_targets(i-1);
                    foll_d(i) = opt_d(i) + sign_*gap*(runData.leader-v);
                end
                plot(opt_t, foll_d, [colors2{ci} '--'], 'LineWidth',2);
            end
        end
    end

    xlabel('Tempo [s]'); ylabel('Posizione [m]');
    title('Confronto Reale vs Ottimali + Semafori');
    grid on;
    hold off;


    %% ——————————————————————————————————————————————————————
    %% Fig.4: Leader speeds (only subRuns)
    %% ——————————————————————————————————————————————————————
    leaders = unique(cellfun(@(r) r.leader, subRuns));
    colors2      = {'b','r','g','m','c','y','k'};
    for L = leaders
        figure('Name',sprintf('Velocità Leader %d',L));
        hold on;
        for k=1:numel(subRuns)
            runData = subRuns{k};
            if runData.leader~=L, continue; end
            plot(runData.t, runData.x(:,end-nVeh+L), 'b-', 'LineWidth',2);

            if ~isempty(runData.opt_t)
                vt = diff(runData.opt_d)./diff(runData.opt_t);
                plot([runData.opt_t(1:end-1);runData.opt_t(2:end)], [vt;vt], ...
                     'r--','LineWidth',2);
                scatter(runData.opt_t(1:end-1), vt, 50, 'r','filled');
            end
        end
        xlabel('Tempo [s]'); ylabel('Velocità [m/s]');
        title(sprintf('Velocità Leader %d',L));
        grid on;
        ylim([0,35]);
        hold off;
    end
end


