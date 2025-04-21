% filepath: main.m
clear; clc; close all;

global SIM_RUNS N_PLATOON;
SIM_RUNS   = {};
N_PLATOON = 1;

disp('=== Avvio prima simulazione con Leader=1 ===');
reset_persistent_variables();
run_optimizer_and_plot(1, 0);
final_plot();

%% =========================================================================
%% Funzioni locali
%% =========================================================================

function run_optimizer_and_plot(leader_vehicle, time_offset, x0_init)
    global SIM_RUNS N_PLATOON

    % Pulizia variabili persistent
    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear get_current_v_target_indexed

    % Parametri di simulazione
    final_time     = 150;
    final_distance = 1800;
    T              = 30;
    tf             = final_time;
    v_min          = 5;
    v_max          = 30;
    b1             = 0.1;
    b2             = 0.01;

    fprintf('\n[INFO] run_optimizer_and_plot(Leader=%d, offset=%.2f)\n', leader_vehicle, time_offset);

    % Definizione semafori
    traffic_lights = [
        create_traffic_light(300,   0, 10, T)
        create_traffic_light(600,  10, 20, T)
        create_traffic_light(900,  20, 30, T)
        create_traffic_light(1200,  0, 10, T)
        create_traffic_light(1550, 10, 20, T)
    ];

    % Pruning temporale
    [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance];
    nIntersections = numel(traffic_lights);

    % Costruzione nodi
    Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
    nodeId = 1;
    Nodes(nodeId) = struct('id', nodeId, 't', 0, 'd', 0, 'int', 0);
    nodeId = nodeId + 1;
    for i = 1:nIntersections
        light = traffic_lights(i);
        for k = 0:ceil(tf/light.cycle_time)
            cs = k*light.cycle_time + light.offset;
            if light.green_start <= light.green_end
                gs = cs + light.green_start;
                ge = cs + light.green_end;
                if gs <= tf
                    ov_s = max(gs, t_min(i));
                    ov_e = min(ge, t_max(i));
                    if ov_s < ov_e
                        mt = ceil((ov_s + ov_e)/2);
                        Nodes(nodeId) = struct('id',nodeId,'t',mt,'d',d(i),'int',i);
                        nodeId = nodeId + 1;
                    end
                end
            else
                gs1 = cs + light.green_start;
                ge1 = cs + light.cycle_time;
                if gs1 <= tf
                    ov_s = max(gs1, t_min(i));
                    ov_e = min(ge1, t_max(i));
                    if ov_s < ov_e
                        mt = floor((ov_s + ov_e)/2);
                        Nodes(nodeId) = struct('id',nodeId,'t',mt,'d',d(i),'int',i);
                        nodeId = nodeId + 1;
                    end
                end
                gs2 = cs;
                ge2 = cs + light.green_end;
                if ge2 <= tf
                    ov_s = max(gs2, t_min(i));
                    ov_e = min(ge2, t_max(i));
                    if ov_s < ov_e
                        mt = floor((ov_s + ov_e)/2);
                        Nodes(nodeId) = struct('id',nodeId,'t',mt,'d',d(i),'int',i);
                        nodeId = nodeId + 1;
                    end
                end
            end
        end
    end
    Nodes(nodeId) = struct('id',nodeId,'t',tf,'d',final_distance,'int',nIntersections+1);
    nNodes = nodeId;

    % Costruzione archi
    Edges = struct('from',{},'to',{},'w',{});
    ec = 1;
    for i = 1:nNodes
        lvlA = Nodes(i).int;
        for j = 1:nNodes
            lvlB = Nodes(j).int;
            if lvlB == lvlA+1 && Nodes(j).t > Nodes(i).t && Nodes(j).d > Nodes(i).d
                if lvlB <= nIntersections && ~is_green(traffic_lights(lvlB), Nodes(j).t)
                    continue;
                end
                dt = Nodes(j).t - Nodes(i).t;
                dd = Nodes(j).d - Nodes(i).d;
                v_link = dd / dt;
                if v_link >= v_min && v_link <= v_max
                    w = dt * (b1 * v_link + b2 * v_link^2);
                    Edges(ec) = struct('from',Nodes(i).id,'to',Nodes(j).id,'w',w);
                    ec = ec + 1;
                end
            end
        end
    end

    % Dijkstra
    [path, cost] = dijkstra(Nodes, Edges, 1, nNodes);
    fprintf('>>> Leader=%.1f, Costo ottimo=%.3f\n', leader_vehicle, cost);
    opt_nodes = Nodes(path);
    opt_t = [opt_nodes.t];
    opt_d = [opt_nodes.d];
    speeds = diff(opt_d) ./ diff(opt_t);

    % Inizializzazione stato iniziale
    n_v    = 5;
    d_init = 4;
    if nargin < 3 || isempty(x0_init)
        x0 = zeros(2 * n_v, 1);
        for ii = 1:n_v
            x0(ii) = -(ii - 1) * d_init;
        end
    else
        x0 = x0_init(:);
    end

    % Simulazione ODE45
    tspan = [0 final_time];
    [t_sim, x_sim] = ode45(@(t,x) system_dynamics_new_platoon( ...
        t, x, n_v, 1000*ones(1,n_v), @(tt)0, traffic_lights, speeds, 1.5, ...
        7000, 0, 0.1, 2000, 0.8, 0.4, leader_vehicle, time_offset), ...
        tspan, x0);

    T_abs = t_sim + time_offset;
    SIM_RUNS{end+1} = struct( ...
        'leader', leader_vehicle, ...
        't', T_abs, ...
        'x', x_sim, ...
        'x0', x0, ...
        'traffic_lights', traffic_lights, ...
        'splittedVehicles', [], ...
        'opt_t', opt_t + time_offset, ...
        'opt_d', opt_d);

    check_red_light_violations(T_abs, x_sim, traffic_lights, T);
end

function final_plot()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[final_plot] Nessun dato da plottare.');
        return;
    end

    figure('Name','Traiettorie Ottimali e Semafori'); hold on
    TL    = SIM_RUNS{1}.traffic_lights;
    maxT  = max(cellfun(@(s)max(s.t), SIM_RUNS));
    times = 0:ceil(maxT);
    all_t = []; all_d = []; all_c = [];
    for i = 1:numel(TL)
        for tt = times
            all_t(end+1)   = tt;
            all_d(end+1)   = TL(i).distance;
            all_c(end+1,:) = is_green(TL(i),tt) * [0 1 0] + ~is_green(TL(i),tt) * [1 0 0];
        end
    end
    scatter(all_t, all_d, 10, all_c, 'filled');

    cols    = {'b','r','g','m','c','k'};
    lns     = {'-','--',':','-.'};
    markers = {'o','s','d','^','v','>','<'};
    H = []; T = {};
    n_v = size(SIM_RUNS{1}.x, 2) / 2;

    for k = 1:numel(SIM_RUNS)
        R = SIM_RUNS{k};
        if isfield(R, 'opt_t')
            ci = mod(R.leader-1, numel(cols)) + 1;
            li = mod(k-1,       numel(lns)) + 1;
            h  = plot(R.opt_t, R.opt_d, [cols{ci} lns{li}], 'LineWidth', 2);
            scatter(R.opt_t, R.opt_d, 50, cols{ci}, markers{li}, 'filled');
            H(end+1) = h;
            T{end+1} = sprintf('Leader %d (run %d)', R.leader, k);

            x0 = R.x0(1:n_v);
            for v = setdiff(1:n_v, R.leader)
                init_off = x0(v) - x0(R.leader);
                fol      = R.opt_d + init_off;
                fi       = mod(v-1, numel(cols)) + 1;
                h2       = plot(R.opt_t, fol, [cols{fi} '--'], 'LineWidth', 1.5);
                scatter(R.opt_t, fol, 30, cols{fi}, 'filled');
                H(end+1) = h2;
                T{end+1} = sprintf('Follower %d (run %d)', v, k);
            end
        end
    end

    legend(H, T, 'Location','Best');
    xlabel('Tempo [s]'); ylabel('Posizione [m]'); grid on;
end

function dx = system_dynamics_new_platoon(t, x, n_v, m, delta_f, TL, v_t, t_CTH, ...
    Kp_s, Ki_s, Kd_s, Kp_d, Ki_d, Kd_d, leader, off)

    dx = zeros(2*n_v, 1);
    persistent t_prev e_i_s e_o_s e_i_d e_o_d flag
    if isempty(t_prev)
        t_prev = t; e_i_s = 0; e_o_s = 0;
        e_i_d = zeros(n_v,1); e_o_d = zeros(n_v,1);
        flag = false;
    end
    dt = max(t - t_prev, 1e-6);
    t_prev = t;
    abs_t  = t + off;

    for i = 1:n_v
        dx(i) = x(n_v + i);
        if i == leader
            vt  = get_current_v_target_indexed(x(leader), TL, v_t);
            err = vt - x(n_v + i);
            e_i_s = e_i_s + err * dt;
            der   = (err - e_o_s) / dt; e_o_s = err;
            U     = Kp_s*err + Ki_s*e_i_s + Kd_s*der;
            vnew  = x(n_v+i) + (U + delta_f(abs_t)) / m(i) * dt;
            vnew  = min(max(vnew,0), 30);
            dx(n_v+i) = (vnew - x(n_v+i)) / dt;
        else
            if i > 1, dist = x(i-1) - x(i); else dist = 10; end
            vcur = x(n_v + i);
            d_des = 1 + t_CTH * vcur;
            err   = dist - d_des;
            e_i_d(i) = e_i_d(i) + err * dt;
            der       = (err - e_o_d(i)) / dt; e_o_d(i) = err;
            U         = Kp_d*err + Ki_d*e_i_d(i) + Kd_d*der;
            vnew      = x(n_v+i) + U / m(i) * dt;
            vnew      = max(vnew, 0);
            dx(n_v+i) = (vnew - x(n_v+i)) / dt;
        end
    end
end

function vt = get_current_v_target_indexed(xl, TL, v)
    idx = find(xl < [TL.distance], 1);
    if isempty(idx), idx = numel(v); end
    vt = v(min(idx, numel(v)));
end

function check_red_light_violations(t_abs, x_sim, TL, T)
    persistent new_flag
    if isempty(new_flag), new_flag = false; end
    n_v = size(x_sim,2) / 2;
    for v = 1:n_v
        pos = x_sim(:,v);
        for L = 1:numel(TL)
            dL = TL(L).distance;
            k  = find(pos(1:end-1) < dL & pos(2:end) >= dL, 1);
            if ~isempty(k) && ~is_green(TL(L), t_abs(k))
                if ~new_flag
                    new_flag = true;
                    x0_new   = x_sim(k, :)';
                    rerun_optimizer_for_new_leader(v, T, x0_new);
                    new_flag = false;
                end
                return;
            end
        end
    end
end

function rerun_optimizer_for_new_leader(v, T, x0)
    clear system_dynamics_new_platoon
    clear get_current_v_target_indexed
    clear check_red_light_violations
    global SIM_RUNS N_PLATOON
    offset    = N_PLATOON * T;
    N_PLATOON = N_PLATOON + 1;
    last      = numel(SIM_RUNS);
    SIM_RUNS{last}.splittedVehicles = v : size(SIM_RUNS{last}.x,2)/2;
    fprintf('\n[INFO] Ricalcolo con NUOVO LEADER=%d, offset=%.2f\n', v, offset);
    run_optimizer_and_plot(v, offset, x0);
end

function reset_persistent_variables()
    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear get_current_v_targetized
    clear next_green prev_green dijkstra
end

function light = create_traffic_light(d, gs, ge, ct)
    light.distance    = d;
    light.green_start = gs;
    light.green_end   = ge;
    light.cycle_time  = ct;
    light.offset      = mod(gs, ct);
end

function st = is_green(light, time)
    c = mod(time - light.offset, light.cycle_time);
    if light.green_start <= light.green_end
        st = (c >= light.green_start && c < light.green_end);
    else
        st = (c >= light.green_start || c < light.green_end);
    end
end

function [t_min, t_max] = velocity_pruning(TL, tf, fd, vmin, vmax)
    n   = numel(TL);
    d   = [TL.distance];
    t_min = zeros(1,n); t_max = zeros(1,n);
    t_min(1) = next_green(TL(1), d(1)/vmax);
    t_max(1) = prev_green(TL(1), d(1)/vmin);
    for i = 2:n
        dd = d(i) - d(i-1);
        t_min(i) = next_green(TL(i), t_min(i-1) + dd/vmax);
        t_max(i) = prev_green(TL(i), min(t_max(i-1) + dd/vmin, tf - (fd - d(i))/vmax));
    end
    for i = n:-1:2
        dtv = (d(i) - d(i-1)) / vmax;
        if t_max(i) > t_max(i-1) + dtv
            t_max(i-1) = prev_green(TL(i-1), t_max(i) - dtv);
        end
    end
end

function tn = next_green(light, t)
    if is_green(light, t), tn = t; return; end
    c = mod(t - light.offset, light.cycle_time);
    if c < light.green_start
        tn = t + (light.green_start - c);
    else
        tn = t + (light.cycle_time - c) + light.green_start;
    end
end

function tp = prev_green(light, t)
    if is_green(light, t), tp = t; return; end
    c = mod(t - light.offset, light.cycle_time);
    if c >= light.green_end
        tp = t - (c - light.green_end);
    else
        tp = t - c - (light.cycle_time - light.green_end);
    end
end

function [path, cost] = dijkstra(Nodes, Edges, src, tgt)
    n    = numel(Nodes);
    cost = inf(1,n);
    prev = nan(1,n);
    cost(src) = 0;
    Q = 1:n;
    while ~isempty(Q)
        [~, i] = min(cost(Q));
        u = Q(i);
        Q(i) = [];
        if u == tgt, break; end
        for e = Edges
            if e.from == u
                alt = cost(u) + e.w;
                if alt < cost(e.to)
                    cost(e.to) = alt;
                    prev(e.to) = u;
                end
            end
        end
    end
    path = [];
    u = tgt;
    while ~isnan(u)
        path = [u path];
        u = prev(u);
    end
end