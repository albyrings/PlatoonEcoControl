clear; clearAllMemoizedCaches; clc; close all; reset_persistent_variables();
global SIM_RUNS N_PLATOON PROCESSED_VEHICLES;
SIM_RUNS = {}; N_PLATOON = 1; PROCESSED_VEHICLES = [];

disp('=== Avvio prima simulazione con Leader=1 ===');
run_optimizer_and_plot(1, 0);
final_plot();



%%%% --------------------------------------------------------------------
%%%%                    F U N Z I O N I   
%%%% --------------------------------------------------------------------

function run_optimizer_and_plot(leader_vehicle, time_offset, activeVehicles)
    global SIM_RUNS
    if nargin<3, activeVehicles = 1:20; end

    final_time     = 150;  tf = final_time;
    final_distance = 1800;
    T  = 30;
    v_min = 5;   v_max = 30;
    b1 = 0.1; b2 = 0.01; b3 = 10; b4 = 4;
    delta_func = @(t)0;

    fprintf('\n[INFO] run_optimizer_and_plot(Leader=%d, offset=%.2f)\n', leader_vehicle, time_offset);

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

    % Build nodes
    Nodes = struct('id',{},'t',{},'d',{},'int',{});
    nodeId = 1;
    Nodes(nodeId) = struct('id',nodeId,'t',0,'d',0,'int',0); nodeId=nodeId+1;
    for i=1:nInt
      L = traffic_lights(i);
      for k=0:ceil(tf/L.cycle_time)
        cs = k*L.cycle_time + L.offset;
        if L.green_start<=L.green_end
          ag=cs+L.green_start; ae=cs+L.green_end;
          if ag<=tf
            vs=max(ag,t_min(i)); ve=min(ae,t_max(i));
            if vs<ve
              mt=ceil((vs+ve)/2);
              Nodes(nodeId)=struct('id',nodeId,'t',mt,'d',d(i),'int',i);
              nodeId=nodeId+1;
            end
          end
        else
          ag1=cs+L.green_start; ae1=cs+L.cycle_time;
          if ag1<=tf
            vs=max(ag1,t_min(i)); ve=min(ae1,t_max(i));
            if vs<ve
              mt=ceil((vs+ve)/2);
              Nodes(nodeId)=struct('id',nodeId,'t',mt,'d',d(i),'int',i);
              nodeId=nodeId+1;
            end
          end
          ag2=cs; ae2=cs+L.green_end;
          if ae2<=tf
            vs=max(ag2,t_min(i)); ve=min(ae2,t_max(i));
            if vs<ve
              mt=ceil((vs+ve)/2);
              Nodes(nodeId)=struct('id',nodeId,'t',mt,'d',d(i),'int',i);
              nodeId=nodeId+1;
            end
          end
        end
      end
    end
    Nodes(nodeId)=struct('id',nodeId,'t',tf,'d',final_distance,'int',nInt+1);
    nNodes = nodeId;

    % Build edges
    Edges = struct('from',{},'to',{},'w',{});
    ec = 1;
    for i=1:nNodes
      for j=1:nNodes
        if Nodes(j).int==Nodes(i).int+1 && Nodes(j).t>Nodes(i).t && Nodes(j).d>Nodes(i).d
          if Nodes(j).int>0 && Nodes(j).int<=nInt
            if ~is_green(traffic_lights(Nodes(j).int), Nodes(j).t), continue; end
          end
          dt = Nodes(j).t-Nodes(i).t;
          dd = Nodes(j).d-Nodes(i).d;
          vlink = dd/dt;
          if vlink>=v_min && vlink<=v_max
            w = dt*(b1*vlink + b2*vlink^2);
            Edges(ec)=struct('from',Nodes(i).id,'to',Nodes(j).id,'w',w);
            ec=ec+1;
          end
        end
      end
    end

    [path,cost] = dijkstra(Nodes,Edges,1,nNodes);
    fprintf('>>> Leader=%d, Costo ottimo=%.3f\n', leader_vehicle, cost);
    optN = Nodes(path);
    opt_t = [optN.t]';
    opt_d = [optN.d]';
    speeds = diff(opt_d)./diff(opt_t);

    % ODE simulation
    nV = numel(activeVehicles);
    m = 1000*ones(1,nV);
    v_targets = speeds;
    Kp_s=7000; Ki_s=0; Kd_s=0.1;
    Kp_d=2000; Ki_d=0.8; Kd_d=0.4;
    t_CTH=1.5; d_init=4;
    x0 = zeros(2*nV,1);
    for i=1:nV
      if activeVehicles(i)==leader_vehicle, x0(i)=0;
      else x0(i)=-d_init*i;
      end
    end
    [t_sim,x_sim] = ode45(@(t,x) system_dynamics_new_platoon(...
      t,x,nV,m,delta_func,traffic_lights,v_targets,t_CTH,...
      Kp_s,Ki_s,Kd_s,Kp_d,Ki_d,Kd_d,leader_vehicle,time_offset),[0 tf],x0);

    T_abs = t_sim + time_offset;
    SIM_RUNS{end+1} = struct( ...
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

function check_red_light_violations(t_abs,x_sim,traffic_lights,T)
    global SIM_RUNS N_PLATOON PROCESSED_VEHICLES
    last=numel(SIM_RUNS);
    active=SIM_RUNS{last}.activeVehicles;
    viol=[];
    for v=active
      idx=find(active==v);
      pos=x_sim(:,idx);
      for L=1:numel(traffic_lights)
        k=find(pos(1:end-1)<traffic_lights(L).distance & pos(2:end)>=traffic_lights(L).distance,1);
        if ~isempty(k) && ~is_green(traffic_lights(L),t_abs(k))
          viol(end+1)=v; break
        end
      end
    end
    viol=setdiff(viol,PROCESSED_VEHICLES);
    if isempty(viol), return; end
    PROCESSED_VEHICLES=[PROCESSED_VEHICLES,viol];
    SIM_RUNS{last}.splittedVehicles=viol;
    newL=viol(1);
    fprintf('\n[WARNING] Veicoli %s col rosso: nuovo leader=%d\n',mat2str(viol),newL);
    offset=N_PLATOON*T; N_PLATOON=N_PLATOON+1;
    clear system_dynamics_new_platoon get_current_v_target_indexed check_red_light_violations
    run_optimizer_and_plot(newL, offset, viol);
end

function final_plot()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[final_plot] Nessun dato da plottare.');
        return;
    end

    % Parametri
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    maxT = max(cellfun(@(s) max(s.t), SIM_RUNS));

    % 1) Posizioni reali + fasi verdi semafori
    figure('Name','Posizioni e Semafori','Position',[100,100,800,600]);
    hold on; grid on;
    xlabel('Tempo [s]'); ylabel('Posizione [m]');
    title('Traiettorie reali con fasi verdi semafori');

    for j = 1:numel(traffic_lights)
        L = traffic_lights(j);
        % numero di cicli da disegnare
        numCycles = ceil( maxT ./ L.cycle_time );
        for k = 0:(numCycles-1)
            gS = k*L.cycle_time + L.green_start;
            gE = k*L.cycle_time + L.green_end;
            if gE < gS
                % wrap-around
                intervals = [gS, k*L.cycle_time+L.cycle_time; ...
                             k*L.cycle_time, gE];
            else
                intervals = [gS, gE];
            end
            for iv = 1:size(intervals,1)
                xs = intervals(iv,1):0.1:intervals(iv,2);
                ys = L.distance * ones(size(xs));
                plot(xs, ys, 'g', 'LineWidth',4);
            end
        end
    end

    % disegno traiettorie reali
    for r = 1:numel(SIM_RUNS)
        D = SIM_RUNS{r};
        for v = 1:size(D.x,2)/2
            plot(D.t, D.x(:,v), 'b-', 'LineWidth',1.5);
        end
    end

    % 2) Traiettorie ottimizzate
    figure('Name','Traiettorie Ottimizzate','Position',[150,150,800,600]);
    hold on; grid on;
    xlabel('Tempo [s]'); ylabel('Posizione [m]');
    title('Profili ottimizzati');
    for r = 1:numel(SIM_RUNS)
        D = SIM_RUNS{r};
        plot(D.opt_t, D.opt_d, 'k--','LineWidth',2);
    end

    % 3) Velocità dei leader confrontata col profilo ottimo
    leaders = unique(cellfun(@(s) s.leader, SIM_RUNS));
    for Ld = leaders
        figure('Name',['Velocità Leader ' num2str(Ld)], ...
               'Position',[200+20*Ld,200+20*Ld,800,500]);
        hold on; grid on;
        xlabel('Tempo [s]'); ylabel('Velocità [m/s]');
        title(['Leader ' num2str(Ld)]);
        for r = 1:numel(SIM_RUNS)
            D = SIM_RUNS{r};
            if D.leader==Ld
                idxL = find(D.activeVehicles==Ld);
                vel  = D.x(:, size(D.x,2)/2 + idxL);
                plot(D.t, vel, 'b-','LineWidth',2);
                % sovrapponi profilo ottimo
                ot = D.opt_t; od = D.opt_d;
                ov = diff(od)./diff(ot);
                stairs([ot; ot(end)], [ov; ov(end)], 'r--','LineWidth',2);
            end
        end
    end
end

function dx = system_dynamics_new_platoon(t,x,nV,m,delta_func,traffic_lights,v_targets,t_CTH,...
        Kp_s,Ki_s,Kd_s,Kp_d,Ki_d,Kd_d,leader_vehicle,time_offset)
    dx=zeros(2*nV,1);
    persistent t_prev eint_s eold_s eint_d eold_d
    if isempty(t_prev), t_prev=t; eint_s=0; eold_s=0; eint_d=zeros(nV,1); eold_d=zeros(nV,1); end
    dt=max(t-t_prev,1e-5); t_prev=t;
    abs_t=t+time_offset;
    for i=1:nV
      dx(i)=x(nV+i);
      if i==leader_vehicle
        vt=get_current_v_target_indexed(x(i),traffic_lights,v_targets);
        e=vt-x(nV+i); eint_s=eint_s+e*dt; de=(e-eold_s)/dt; eold_s=e;
        U=Kp_s*e+Ki_s*eint_s+Kd_s*de;
        a=(U+delta_func(abs_t))/m(i);
        vnew=x(nV+i)+a*dt; vnew=max(0,min(vnew,30));
        dx(nV+i)=(vnew-x(nV+i))/dt;
      else
        if i>1, dist=x(i-1)-x(i); else dist=10; end
        vcur=x(nV+i);
        ddes=1+t_CTH*vcur; e=dist-ddes;
        eint_d(i)=eint_d(i)+e*dt; dd=(e-eold_d(i))/dt; eold_d(i)=e;
        U=Kp_d*e+Ki_d*eint_d(i)+Kd_d*dd;
        a=U/m(i); vnew=x(nV+i)+a*dt; vnew=max(0,vnew);
        dx(nV+i)=(vnew-x(nV+i))/dt;
      end
    end
end

function vt=get_current_v_target_indexed(x_lead,traffic_lights,v_targets)
    idx=find(x_lead<[traffic_lights.distance],1);
    if isempty(idx), vt=v_targets(end);
    else vt=v_targets(min(idx,numel(v_targets))); end
end

function light=create_traffic_light(distance,gs,ge,ct)
    light.distance=distance;
    light.green_start=gs;
    light.green_end=ge;
    light.cycle_time=ct;
    light.offset=mod(gs,ct);
end

function st=is_green(light,time)
    tc=mod(time-light.offset,light.cycle_time);
    if light.green_start<=light.green_end
      st=tc>=light.green_start && tc<light.green_end;
    else
      st=tc>=light.green_start || tc<light.green_end;
    end
end

function [t_min,t_max]=velocity_pruning(traffic_lights,tf,final_distance,v_min,v_max)
    n=numel(traffic_lights); d=[traffic_lights.distance];
    t_min=zeros(1,n); t_max=zeros(1,n);
    t_min(1)=next_green(traffic_lights(1),d(1)/v_max);
    t_max(1)=prev_green(traffic_lights(1),d(1)/v_min);
    for i=2:n
      inc=d(i)-d(i-1);
      t_min(i)=t_min(i-1)+inc/v_max;
      t_max(i)=min(t_max(i-1)+inc/v_min, tf-(final_distance-d(i))/v_max);
      t_min(i)=next_green(traffic_lights(i),t_min(i));
      t_max(i)=prev_green(traffic_lights(i),t_max(i));
    end
    for i=n:-1:2
      need=(d(i)-d(i-1))/v_max;
      if t_max(i)>t_max(i-1)+need
        t_max(i-1)=prev_green(traffic_lights(i-1), t_max(i)-need);
      end
    end
end

function t_next=next_green(light,t)
    if is_green(light,t), t_next=t; return; end
    c=mod(t-light.offset,light.cycle_time);
    if c<light.green_start
      t_next=t+(light.green_start-c);
    else
      t_next=t+(light.cycle_time-c)+light.green_start;
    end
end

function t_prev=prev_green(light,t)
    if is_green(light,t), t_prev=t; return; end
    c=mod(t-light.offset,light.cycle_time);
    if c>=light.green_end
      t_prev=t-(c-light.green_end);
    else
      t_prev=t-c-(light.cycle_time-light.green_end);
    end
end

function [path,cost]=dijkstra(Nodes,Edges,source,target)
    nN=numel(Nodes);
    cost=inf(1,nN); prev=nan(1,nN);
    cost(source)=0; Q=1:nN;
    while ~isempty(Q)
      [~,k]=min(cost(Q)); u=Q(k); Q(Q==u)=[];
      if u==target, break; end
      for e=Edges
        if e.from==u
          alt=cost(u)+e.w;
          if alt<cost(e.to), cost(e.to)=alt; prev(e.to)=u; end
        end
      end
    end
    path=[]; u=target;
    while ~isnan(u), path=[u,path]; u=prev(u); end
end

function reset_persistent_variables()
    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear get_current_v_target_indexed
    clear next_green prev_green
    clear dijkstra
end