% filepath: ./optimizer.m
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
                        mt = ceil((ov_s+ov_e)/2);
                        Nodes(nodeId) = struct('id',nodeId,'t',mt,'d',d(i),'int',i);
                        nodeId = nodeId + 1;
                    end
                end
            else
                % verde avvolgente
                gs1 = cs + light.green_start;
                ge1 = cs + light.cycle_time;
                if gs1 <= tf
                    ov_s = max(gs1, t_min(i));
                    ov_e = min(ge1, t_max(i));
                    if ov_s < ov_e
                        mt = (ov_s+ov_e)/2;
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
                        mt = (ov_s+ov_e)/2;
                        Nodes(nodeId) = struct('id',nodeId,'t',mt,'d',d(i),'int',i);
                        nodeId = nodeId + 1;
                    end
                end
            end
        end
    end
    Nodes(nodeId)=struct('id',nodeId,'t',tf,'d',final_distance,'int',nIntersections+1);
    nNodes = nodeId;

    % Costruzione archi
    Edges = struct('from',{},'to',{},'w',{});
    ec = 1;
    for i = 1:nNodes
        lvlA = Nodes(i).int;
        for j = 1:nNodes
            lvlB = Nodes(j).int;
            if lvlB == lvlA+1 && Nodes(j).t>Nodes(i).t && Nodes(j).d>Nodes(i).d
                if lvlB<=nIntersections && ~is_green(traffic_lights(lvlB),Nodes(j).t)
                    continue;
                end
                dt = Nodes(j).t-Nodes(i).t;
                dd = Nodes(j).d-Nodes(i).d;
                v_link = dd/dt;
                if v_link>=v_min && v_link<=v_max
                    w = dt*(b1*v_link + b2*v_link^2);
                    Edges(ec)=struct('from',Nodes(i).id,'to',Nodes(j).id,'w',w);
                    ec = ec+1;
                end
            end
        end
    end

    [path, cost] = dijkstra(Nodes,Edges,1,nNodes);
    fprintf('>>> Leader=%.1f, Costo ottimo=%.3f\n', leader_vehicle, cost);
    opt = Nodes(path);
    opt_t = [opt.t];
    opt_d = [opt.d];
    speeds = diff(opt_d)./diff(opt_t);

    % simulazione PID
    n_v = 5;
    m_v = 1000*ones(1,n_v);
    v_targets = speeds;
    Kp_s = 7000; Ki_s = 0; Kd_s = 0.1;
    Kp_d = 2000; Ki_d = 0.8; Kd_d = 0.4;
    t_CTH = 1.5; d0 = 4;

    x0 = zeros(2*n_v,1);
    for i=1:n_v
        x0(i) = -(i-1)*d0;
    end

    tspan = [0 final_time];
    [t_sim,x_sim] = ode45(@(t,x) system_dynamics_new_platoon(...
        t,x,n_v,m_v,@(tt)0,traffic_lights,v_targets,t_CTH,...
        Kp_s,Ki_s,Kd_s,Kp_d,Ki_d,Kd_d,leader_vehicle,time_offset),...
        tspan,x0);

    T_abs = t_sim + time_offset;
    SIM_RUNS{end+1} = struct(...
        'leader',leader_vehicle,...
        't',T_abs,'x',x_sim,...
        'traffic_lights',traffic_lights,...
        'splittedVehicles',[],...
        'opt_t',opt_t+time_offset,'opt_d',opt_d);

    check_red_light_violations(T_abs,x_sim,traffic_lights,T);
end

function final_plot()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[final_plot] Nessun dato da plottare.');
        return;
    end

    % Fig 1: ottimali + semafori
    figure('Name','Posizioni Ottimali e Semafori'); hold on
    TL = SIM_RUNS{1}.traffic_lights;
    maxT = max(cellfun(@(s)max(s.t),SIM_RUNS));
    times = 0:ceil(maxT);
    all_t=[]; all_d=[]; all_c=[];
    for i=1:numel(TL)
        for tt=times
            all_t(end+1)=tt;
            all_d(end+1)=TL(i).distance;
            all_c(end+1,:) = is_green(TL(i),tt)*[0 1 0] + ~is_green(TL(i),tt)*[1 0 0];
        end
    end
    scatter(all_t,all_d,10,all_c,'filled');

    cols = {'b','r','g','m','c','k'}; lns={'-','--',':','-.'}; mks={'o','s','d','^'};
    h=[]; txt={};
    for k=1:numel(SIM_RUNS)
        R = SIM_RUNS{k};
        if isfield(R,'opt_t')
            c = mod(R.leader-1,numel(cols))+1;
            l = mod(k-1,numel(lns))+1;
            h(end+1)=plot(R.opt_t,R.opt_d,[cols{c} lns{l}],'LineWidth',2);
            txt{end+1}=sprintf('Leader %d (run %d)',R.leader,k);
        end
    end
    legend(h,txt,'Location','Best');
    xlabel('t [s]'); ylabel('d [m]'); grid on;

    % Fig 2 e 3 analoghi...
end