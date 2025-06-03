clear;
clearAllMemoizedCaches;
clc;
close all;
reset_persistent_variables();

global SIM_RUNS N_PLATOON;
SIM_RUNS  = {};      % array di struct: (leader,t,x,offset,traffic_lights,activeVehicles,splittedVehicles,opt_t,opt_d)
N_PLATOON = 1;       % contatore plotoni

disp('=== Avvio prima simulazione con Leader=1 ===');
run_optimizer_and_plot(1, 0);   % primo plotone: leader=1, offset=0, tutti i veicoli
final_plot();



%%%% --------------------------------------------------------------------
%%%%                    F U N Z I O N I   
%%%% --------------------------------------------------------------------

function run_optimizer_and_plot(leader_vehicle, time_offset, activeVehicles)
    global SIM_RUNS

    % se non passo activeVehicles => prima run: veicoli 1:5
    if nargin<3
        activeVehicles = 1:5;
    end

    % Parametri
    final_time     = 150;
    final_distance = 1800;
    T  = 30;
    tf = final_time;
    v_min = 5;   v_max = 30;
    b1 = 0.1; b2 = 0.01; b3 = 10; b4 = 4;
    delta_func = @(t)0;

    fprintf('\n[INFO] run_optimizer_and_plot(Leader=%d, offset=%.2f)\n', leader_vehicle, time_offset);

    % semafori
    traffic_lights = [
      create_traffic_light(300,   0, 10, T)
      create_traffic_light(600,  10, 20, T)
      create_traffic_light(900,  20, 30, T)
      create_traffic_light(1200,  0, 10, T)
      create_traffic_light(1550, 10, 20, T)
    ];
    [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance];
    nInt = numel(traffic_lights);

    % === build NODES ===
    Nodes = struct('id',{},'t',{},'d',{},'int',{});
    nodeId = 1;
    Nodes(nodeId) = struct('id',nodeId,'t',0,'d',0,'int',0); nodeId = nodeId+1;
    for i=1:nInt
      light = traffic_lights(i);
      for k=0:ceil(tf/light.cycle_time)
        cycle_start = k*light.cycle_time + light.offset;
        if light.green_start<=light.green_end
          abs_start = cycle_start+light.green_start;
          abs_end   = cycle_start+light.green_end;
          if abs_start<=tf
            ov_s = max(abs_start, t_min(i));
            ov_e = min(abs_end,   t_max(i));
            if ov_s<ov_e
              mid_t = ceil((ov_s+ov_e)/2);
              Nodes(nodeId)=struct('id',nodeId,'t',mid_t,'d',d(i),'int',i);
              nodeId=nodeId+1;
            end
          end
        else
          % green wrap
          abs_s1 = cycle_start+light.green_start;
          abs_e1 = cycle_start+light.cycle_time;
          if abs_s1<=tf
            ov_s = max(abs_s1, t_min(i));
            ov_e = min(abs_e1, t_max(i));
            if ov_s<ov_e
              mid_t=(ov_s+ov_e)/2;
              Nodes(nodeId)=struct('id',nodeId,'t',mid_t,'d',d(i),'int',i);
              nodeId=nodeId+1;
            end
          end
          abs_s2 = cycle_start;
          abs_e2 = cycle_start+light.green_end;
          if abs_e2<=tf
            ov_s = max(abs_s2, t_min(i));
            ov_e = min(abs_e2, t_max(i));
            if ov_s<ov_e
              mid_t=(ov_s+ov_e)/2;
              Nodes(nodeId)=struct('id',nodeId,'t',mid_t,'d',d(i),'int',i);
              nodeId=nodeId+1;
            end
          end
        end
      end
    end
    Nodes(nodeId)=struct('id',nodeId,'t',tf,'d',final_distance,'int',nInt+1);
    nNodes = nodeId;

    % === build EDGES ===
    Edges = struct('from',{},'to',{},'w',{});
    edgeCount = 1;
    for i=1:nNodes
      for j=1:nNodes
        if Nodes(j).int==Nodes(i).int+1 && Nodes(j).t>Nodes(i).t && Nodes(j).d>Nodes(i).d
          if Nodes(j).int>0 && Nodes(j).int<=nInt
            if ~is_green(traffic_lights(Nodes(j).int), Nodes(j).t)
              continue;
            end
          end
          dt = Nodes(j).t-Nodes(i).t;
          dd = Nodes(j).d-Nodes(i).d;
          v_link = dd/dt;
          if v_link>=v_min && v_link<=v_max
            w = dt*(b1*v_link + b2*v_link^2);
            Edges(edgeCount)=struct('from',Nodes(i).id,'to',Nodes(j).id,'w',w);
            edgeCount=edgeCount+1;
          end
        end
      end
    end

    % Dijkstra
    [path, cost] = dijkstra(Nodes, Edges, 1, nNodes);
    fprintf('>>> Leader=%d, Costo ottimo=%.3f\n', leader_vehicle, cost);
    opt_nodes = Nodes(path);
    opt_t = [opt_nodes.t]';
    opt_d = [opt_nodes.d]';
    speeds = diff(opt_d)./diff(opt_t);

    % ODE simulation
    n_veh = numel(activeVehicles);
    m = 1000*ones(1,n_veh);
    v_targets = speeds;
    Kp_s=7000; Ki_s=0; Kd_s=0.1;
    Kp_d=2000; Ki_d=0.8; Kd_d=0.4;
    t_CTH=1.5; d_init=4;

    x0 = zeros(2*n_veh,1);
    for i=1:n_veh
      if activeVehicles(i)==leader_vehicle
        x0(i)=0;
      else
        x0(i)=-d_init*i;
      end
    end

    [t_sim,x_sim] = ode45(@(t,x) system_dynamics_new_platoon(...
      t,x,n_veh,m,delta_func,traffic_lights,v_targets,t_CTH, ...
      Kp_s,Ki_s,Kd_s,Kp_d,Ki_d,Kd_d,leader_vehicle,time_offset),...
      [0 tf], x0);

    T_abs = t_sim + time_offset;
    SIM_RUNS{end+1} = struct(...
      'leader', leader_vehicle, ...
      't', T_abs, ...
      'x', x_sim, ...
      'offset', time_offset, ...
      'traffic_lights', traffic_lights, ...
      'activeVehicles', activeVehicles, ...
      'splittedVehicles', [], ...
      'opt_t', opt_t+time_offset, ...
      'opt_d', opt_d);

    check_red_light_violations(T_abs, x_sim, traffic_lights, T);
end

function check_red_light_violations(t_abs, x_sim, traffic_lights, T)
    global SIM_RUNS N_PLATOON
    last = numel(SIM_RUNS);
    active = SIM_RUNS{last}.activeVehicles;
    violators = [];
    for v = active
      idxLoc = find(active==v);
      pos_v = x_sim(:, idxLoc);
      for L=1:numel(traffic_lights)
        if ~isempty(find(pos_v(1:end-1)<traffic_lights(L).distance & pos_v(2:end)>=traffic_lights(L).distance,1)) ...
           && ~is_green(traffic_lights(L), t_abs(find(pos_v(1:end-1)<traffic_lights(L).distance & pos_v(2:end)>=traffic_lights(L).distance,1)))
          violators(end+1)=v; break
        end
      end
    end
    if isempty(violators), return; end

    SIM_RUNS{last}.splittedVehicles = violators;
    newLeader = violators(1);
    fprintf('\n[WARNING] Veicoli %s col rosso: nuovo leader=%d\n', mat2str(violators), newLeader);

    offset = N_PLATOON * T; N_PLATOON = N_PLATOON+1;
    clear system_dynamics_new_platoon get_current_v_target_indexed check_red_light_violations
    run_optimizer_and_plot(newLeader, offset, violators);
end

function dx = system_dynamics_new_platoon(t, x, n_veh, m, delta_func, ...
    traffic_lights, v_targets, t_CTH, Kp_s, Ki_s, Kd_s, Kp_d, Ki_d, Kd_d, ...
    leader_vehicle, time_offset)
    dx = zeros(2*n_veh,1);
    persistent t_prev eint_s eold_s eint_d eold_d activeVeh
    if isempty(t_prev)
        t_prev = t;
        eint_s=0; eold_s=0;
        eint_d=zeros(n_veh,1); eold_d=zeros(n_veh,1);
    end
    dt = max(t - t_prev,1e-5);
    t_prev = t;
    abs_t = t + time_offset;

    % Trovo l'indice locale del leader
    % ATTENZIONE: in chiamata a ode45 usiamo x di dimensione 2*n_veh,
    % ma non abbiamo qui activeVehicles: passi activeVehicles come global se serve.
    % Se leader è sempre indice 1, altrimenti devi passare leader_idx.
    leader_idx = leader_vehicle;  % se la run era fatta con activeVehicles = 1:N

    for i=1:n_veh
        dx(i) = x(n_veh+i);
        if i==leader_idx
            % LEADER
            vt   = get_current_v_target_indexed(x(i), traffic_lights, v_targets);
            err  = vt - x(n_veh+i);
            eint_s = eint_s + err*dt;
            deriv  = (err - eold_s)/dt; eold_s=err;
            U = Kp_s*err + Ki_s*eint_s + Kd_s*deriv;
            a = (U + delta_func(abs_t))/m(i);
            v_new = x(n_veh+i) + a*dt;
            v_new = max(0,min(v_new,30));
            dx(n_veh+i) = (v_new - x(n_veh+i))/dt;
        else
            % FOLLOWER: proteggo i==1
            if i>1
                dist = x(i-1) - x(i);
            else
                dist = 10;
            end
            v_cur = x(n_veh+i);
            d_des = 1 + t_CTH*v_cur;
            err   = dist - d_des;
            eint_d(i) = eint_d(i) + err*dt;
            deriv = (err - eold_d(i))/dt; eold_d(i)=err;
            U = Kp_d*err + Ki_d*eint_d(i) + Kd_d*deriv;
            a = U/m(i);
            v_new = x(n_veh+i) + a*dt;
            v_new = max(0, v_new);
            dx(n_veh+i) = (v_new - x(n_veh+i))/dt;
        end
    end
end

function vt = get_current_v_target_indexed(x_lead, traffic_lights, v_targets)
    idx = find(x_lead < [traffic_lights.distance],1);
    if isempty(idx), vt=v_targets(end); else vt=v_targets(min(idx,numel(v_targets))); end
end

function light = create_traffic_light(distance, green_start, green_end, cycle_time)
    light.distance = distance;
    light.green_start = green_start;
    light.green_end = green_end;
    light.cycle_time = cycle_time;
    light.offset = mod(green_start,cycle_time);
end

function st = is_green(light, time)
    tc = mod(time-light.offset, light.cycle_time);
    if light.green_start<=light.green_end
      st = tc>=light.green_start && tc<light.green_end;
    else
      st = tc>=light.green_start || tc<light.green_end;
    end
end

function [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max)
    n = numel(traffic_lights);
    d = [traffic_lights.distance];
    t_min = zeros(1,n); t_max = zeros(1,n);
    t_min(1)=d(1)/v_max; t_max(1)=d(1)/v_min;
    t_min(1)=next_green(traffic_lights(1),t_min(1));
    t_max(1)=prev_green(traffic_lights(1),t_max(1));
    for i=2:n
      inc = d(i)-d(i-1);
      t_min(i)=t_min(i-1)+inc/v_max;
      t_max(i)=t_max(i-1)+inc/v_min;
      t_max(i)=min(t_max(i), tf-(final_distance-d(i))/v_max);
      t_min(i)=next_green(traffic_lights(i),t_min(i));
      t_max(i)=prev_green(traffic_lights(i),t_max(i));
    end
    for i=n:-1:2
      needed = (d(i)-d(i-1))/v_max;
      if t_max(i)>t_max(i-1)+needed
        t_max(i-1)=prev_green(traffic_lights(i-1), t_max(i)-needed);
      end
    end
end

function t_next = next_green(light, t)
    if is_green(light,t), t_next=t; return; end
    cyc = mod(t-light.offset, light.cycle_time);
    if cyc<light.green_start
      t_next = t + (light.green_start-cyc);
    else
      t_next = t + (light.cycle_time-cyc) + light.green_start;
    end
end

function t_prev = prev_green(light, t)
    if is_green(light,t), t_prev=t; return; end
    cyc = mod(t-light.offset, light.cycle_time);
    if cyc>=light.green_end
      t_prev = t - (cyc-light.green_end);
    else
      t_prev = t - cyc - (light.cycle_time-light.green_end);
    end
end

function [path,cost] = dijkstra(Nodes, Edges, source, target)
    nN = numel(Nodes);
    cost = inf(1,nN); prev = nan(1,nN);
    cost(source)=0; Q=1:nN;
    while ~isempty(Q)
      [~,k]=min(cost(Q)); u=Q(k); Q(Q==u)=[];
      if u==target, break; end
      for e=Edges
        if e.from==u
          alt = cost(u)+e.w;
          if alt<cost(e.to), cost(e.to)=alt; prev(e.to)=u; end
        end
      end
    end
    path=[]; u=target;
    while ~isnan(u)
      path=[u,path]; u=prev(u);
    end
end

function reset_persistent_variables()
    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear get_current_v_target_indexed
    clear next_green prev_green
    clear dijkstra
end