function run_optimizer_and_plot(leader_vehicle, time_offset)
    global SIM_RUNS N_PLATOON

    % Pulizia variabili persistent
    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear get_current_v_target_indexed
    
    % Parametri
    final_time     = 150;
    final_distance = 1800;
    v_min          = 5;
    v_max          = 30;
    b1             = 0.1;
    b2             = 0.01;
    b3 = 10;
    b4 = 4;
    delta_func = @(t) 0 * abs((b1 + b2*(sin((1/b3)*t+b4) + 0.25*rand)));

    fprintf('\n[INFO] run_optimizer_and_plot(Leader=%d, offset=%.2f)\n', ...
        leader_vehicle, time_offset);

    % Definizione semafori
    T = 30; % periodo di ciclo
    traffic_lights = [
        create_traffic_light(300,   0, 10, T)
        create_traffic_light(600,  10, 20, T)
        create_traffic_light(900,  20, 30, T)
        create_traffic_light(1200,  0, 10, T)
        create_traffic_light(1550, 10, 20, T)
    ];

    % Richiama l'ottimizzatore
    [opt_t, opt_d, speeds, path, cost, Nodes, Edges] = optimizer( ...
        final_time, final_distance, v_min, v_max, b1, b2, traffic_lights );

    fprintf('>>> Leader=%.1f, Costo ottimo=%.3f\n', leader_vehicle, cost);

    n_vehicles=8;
    m_vehicles=1000*ones(1,n_vehicles);

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
    [t_sim,x_sim] = ode45(@(t,x)system_dynamics_new_platoon( ...
        t,x,n_vehicles,m_vehicles,delta_func, traffic_lights,speeds,t_CTH, ...
        K_p_speed,K_i_speed,K_d_speed,K_p_dist,K_i_dist,K_d_dist, ...
        leader_vehicle, time_offset), t_span, x0);

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

    check_red_light_violations(T_abs, x_sim, traffic_lights, T);
end