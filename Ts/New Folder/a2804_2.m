%% Script principale
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
%% Funzioni locali
%% =========================================================================

function run_optimizer_and_plot(leader_vehicle, time_offset, start_position)
    global SIM_RUNS N_PLATOON

    % Imposta una posizione di partenza predefinita se non specificata
    if nargin < 3
        start_position = 0;  % Posizione di partenza predefinita
    end

    % Pulizia variabili persistent
    clear system_dynamics_new_platoon
    clear check_red_light_violations
    clear get_current_v_target_indexed
    
    % Parametri e soglie
    final_time     = 150;         
    final_distance = 1800;    
    T              = 30;                   
    tf             = final_time;
    v_min          = 0;   
    v_max          = 30;  
    b1             = 0.1;  
    b2             = 0.01;
    b3 = 10;
    b4 = 4;

    delta_func = @(t) -0 * (b1 + b2*(sin((1/b3)*t+b4) + 0.25*rand));
    
    % Stampa debug
    fprintf('\n[INFO] run_optimizer_and_plot(Leader=%d, offset=%.2f)\n', ...
        leader_vehicle, time_offset);

    % Definizione semafori
    traffic_lights = [
        create_traffic_light(300,   0, 10, T)
        create_traffic_light(600,  10, 20, T)
        create_traffic_light(900,  20, 30, T)
        create_traffic_light(1200,  0, 10, T)
        create_traffic_light(1550, 10, 20, T)
    ];


    % Esempio di pruning
   [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance]; 
    nIntersections = length(traffic_lights);

    % Costruzione nodi
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

    % Costruzione archi
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
                    v_link= delta_d/delta_t;
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

    n_vehicles=6;
    m_vehicles=1000*ones(1,n_vehicles);
    v_targets=speeds;  

    % PID
    K_p_speed=7000; K_i_speed=0; K_d_speed=0.1;
    K_p_dist=2000;  K_i_dist=0.8;K_d_dist=0.4;
    t_CTH=1.5;  
    d_init=4;

    % Condizioni iniziali modificate per iniziare dalla posizione specificata
    x0=zeros(2*n_vehicles,1);
    for i=1:n_vehicles
        if i==leader_vehicle
            x0(i)=start_position;  % Il leader parte dalla posizione specificata
        else
            if i < leader_vehicle
                % Veicoli davanti (se esistono) non vengono influenzati
                x0(i)=start_position + d_init*(leader_vehicle-i);
            else
                % Veicoli dietro
                x0(i)=start_position - d_init*(i-leader_vehicle);
            end
        end
    end

     t_span=[0 150];
    [t_sim,x_sim]= ode45(@(t,x)system_dynamics_new_platoon( ...
        t,x,n_vehicles,m_vehicles,delta_func, traffic_lights,v_targets,t_CTH, ...
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

    % Verifica semafori e trigger di velocità
    check_red_light_violations(T_abs, x_sim, traffic_lights, T);
    check_velocity_triggers(T_abs, x_sim, traffic_lights);  % Passa anche traffic_lights
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
    
    % Parametri della simulazione
    final_distance = 1800;  % Distanza finale della simulazione
    
    persistent e_int_speed e_old_speed
    persistent e_int_dist  e_old_dist
    if isempty(e_int_speed), e_int_speed = 0; e_old_speed = 0; end
    if isempty(e_int_dist),  e_int_dist = zeros(n_vehicles, 1); e_old_dist = zeros(n_vehicles, 1); end

    abs_t = t + time_offset;
    
    % Verifica se siamo in un semaforo rosso (per ogni veicolo)
    at_red_light = zeros(n_vehicles, 1);
    for i = 1:n_vehicles
        vehicle_pos = x(i);
        
        % Controlla se il veicolo è a un semaforo rosso
        for L = 1:length(traffic_lights)
            light_d = traffic_lights(L).distance;
            % Se il veicolo è molto vicino al semaforo (± 5m) e il semaforo è rosso
            if abs(vehicle_pos - light_d) < 5 && ~is_green(traffic_lights(L), abs_t)
                at_red_light(i) = 1;
                % Stampa debug solo se è il leader
                if i == leader_vehicle
                    fprintf('[DEBUG] Veicolo %d fermo al semaforo rosso a %.2f metri\n', i, light_d);
                end
                break;
            end
        end
        
        % Controllo fine simulazione - se un veicolo ha raggiunto la destinazione
        if vehicle_pos >= final_distance
            dx(i) = 0;  % Ferma il veicolo
            dx(n_vehicles + i) = 0;  % Velocità zero
            continue;  % Vai al prossimo veicolo
        end
        
        % Controllo semaforo rosso - se il veicolo è a un semaforo rosso
        if at_red_light(i) == 1
            dx(i) = 0;  % Ferma il veicolo
            dx(n_vehicles + i) = -3 * x(n_vehicles + i);  % Decelera rapidamente a zero
            continue;  % Vai al prossimo veicolo
        end

        dx(i) = x(n_vehicles + i);
        
        if i == leader_vehicle
            % Comportamento del leader
            % Assicurati che v_targets non sia vuoto
            if isempty(v_targets)
                vt = 10;  % Velocità predefinita sicura
            else
                vt = get_current_v_target_indexed(x(leader_vehicle), traffic_lights, v_targets);
            end
            
            vel_err = vt - x(n_vehicles + i);
            e_int_speed = e_int_speed + vel_err * dt;
            vel_deriv = (vel_err - e_old_speed) / dt;
            e_old_speed = vel_err;
            U_leader = K_p_speed*vel_err + K_i_speed*e_int_speed + K_d_speed*vel_deriv;
            dx(n_vehicles + i) = (U_leader + delta_func(abs_t)) / m(i);
            
            max_speed = 30;
            new_vel = x(n_vehicles + i) + dx(n_vehicles + i)*dt;
            if new_vel < 0, new_vel = 0; end
            if new_vel > max_speed, new_vel = max_speed; end
            dx(n_vehicles + i) = (new_vel - x(n_vehicles + i)) / dt;
        else
            % Comportamento dei follower
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
            U_dist = K_p_dist*dist_err + K_i_dist*e_int_dist(i) + K_d_dist*dist_deriv;
            dx(n_vehicles + i) = U_dist / m(i);
            
            new_vel = x(n_vehicles + i) + dx(n_vehicles + i)*dt;
            if new_vel < 0, new_vel = 0; end
            dx(n_vehicles + i) = (new_vel - x(n_vehicles + i)) / dt;
        end
    end
end

function vt = get_current_v_target_indexed(x_leader, traffic_lights, v_targets)
    % Controllo di sicurezza sui parametri in ingresso
    if isempty(v_targets)
        fprintf('[WARNING] v_targets è vuoto, uso velocità predefinita 10 m/s\n');
        vt = 10;  % Velocità di sicurezza predefinita
        return;
    end
    
    idx = find(x_leader < [traffic_lights.distance], 1);
    if isempty(idx)
        % Se non ci sono semafori davanti, usa l'ultima velocità
        vt = v_targets(end);
    else
        % Assicurati che l'indice sia valido e positivo
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
    
    % Parametri di rilevamento dei semafori (valori originali)
    look_ahead_distance = 30;  % Metri di anticipo per rilevare il semaforo
    stop_margin = 5;           % Metri di distanza di sicurezza prima del semaforo
    
    for v = 1:n_vehicles
        if ~is_in_current_platoon(v, current_run)
            continue;
        end
        
        pos_v = x_sim(:, v);
        v_vel = x_sim(:, n_vehicles + v);
        
        % Per ogni semaforo
        for L = 1:length(traffic_lights)
            light_d = traffic_lights(L).distance;
            
            % Controlla se il veicolo si sta avvicinando al semaforo
            approaching_idx = find(pos_v >= light_d - look_ahead_distance & pos_v < light_d, 1, 'last');
            
            if ~isempty(approaching_idx)
                approach_time = t_abs(approaching_idx);
                current_pos = pos_v(approaching_idx);
                current_vel = v_vel(approaching_idx);
                
                % Stima quando attraverserà il semaforo
                if abs(current_vel) < 0.1
                    time_to_light = 100; % Tempo arbitrario se fermo
                else
                    time_to_light = (light_d - current_pos) / current_vel;
                end
                estimated_crossing_time = approach_time + time_to_light;
                
                % Verifica se il semaforo è o sarà rosso quando il veicolo lo attraversa
                if ~is_green(traffic_lights(L), estimated_crossing_time)
                    if any(violations_detected == v)
                        continue;
                    end
                    
                    % Debug info
                    fprintf('\n[WARNING] Veicolo %d si ferma al semaforo %d (t=%.2f s)\n', v, L, approach_time);
                    fprintf('          Posizione attuale: %.2f m, Velocità: %.2f m/s\n', current_pos, current_vel);
                    fprintf('          Semaforo a: %.2f m, Attraversamento a: %.2f s\n', light_d, estimated_crossing_time);
                    
                    violations_detected = [violations_detected, v];
                    
                    % Calcola quando il semaforo diventerà verde
                    next_green_time = next_green(traffic_lights(L), estimated_crossing_time);
                    fprintf('>> Veicolo %d diventa leader di un nuovo plotone!\n', v);
                    fprintf('>> Semaforo diventerà verde a t=%.2f\n', next_green_time);
                    
                    % Posizione di stop è poco prima del semaforo
                    stop_position = light_d - stop_margin;
                    
                    % IMPORTANTE: Aggiorna l'elenco dei veicoli che fanno parte di questo nuovo plotone
                    SIM_RUNS{current_run}.splittedVehicles = v:n_vehicles;
                    
                    % Crea il nuovo plotone
                    rerun_optimizer_from_traffic_light(v, next_green_time, stop_position, traffic_lights(L));
                    return;
                end
            end
        end
    end
end


function run_optimizer_from_traffic_light(leader_vehicle, start_time, start_position, traffic_light)
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
    v_min          = 0;
    v_max          = 30;
    b1             = 0.1;
    b2             = 0.01;
    b3 = 10;
    b4 = 4;

    delta_func = @(t) -0 * (b1 + b2*(sin((1/b3)*t+b4) + 0.25*rand));
    
    fprintf('\n[INFO] run_optimizer_from_traffic_light(Leader=%d, tempo=%.2f, posizione=%.2f)\n', ...
        leader_vehicle, start_time, start_position);

    % Recupera la configurazione dei semafori dalla simulazione originale
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    
    % Filtra i semafori che sono ancora davanti al veicolo
    remaining_lights = [];
    for i = 1:length(traffic_lights)
        if traffic_lights(i).distance > start_position
            remaining_lights = [remaining_lights; traffic_lights(i)];
        end
    end
    
    % IMPORTANTE: Inizia il grafo dal punto in cui il semaforo è appena diventato verde
    Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
    nodeId = 1;
    
    % Il primo nodo è alla posizione di partenza e al tempo in cui il semaforo diventa verde
    Nodes(nodeId) = struct('id', nodeId, 't', start_time, 'd', start_position, 'int', 0);
    nodeId = nodeId + 1;

    % Se non ci sono più semafori davanti, aggiungi solo il nodo finale
    if isempty(remaining_lights)
        % Calcola il tempo minimo per arrivare alla destinazione finale
        min_time_to_final = (final_distance - start_position) / v_max;
        
        % Aggiungi il nodo finale
        Nodes(nodeId) = struct('id', nodeId, 't', start_time + min_time_to_final, 'd', final_distance, 'int', 1);
        nIntersections = 0;
    else
        % Ci sono ancora semafori davanti
        nIntersections = length(remaining_lights);
        
        % Per ogni semaforo rimanente, crea nodi durante le finestre verdi
        for i = 1:nIntersections
            light = remaining_lights(i);
            
            % Calcola tempo minimo e massimo per raggiungere questo semaforo
            dist_to_light = light.distance - start_position;
            t_min_arrival = start_time + dist_to_light / v_max;
            t_max_arrival = start_time + dist_to_light / v_min;
            
            % Considera tutti i cicli del semaforo nel periodo di simulazione
            for k = 0:ceil((start_time + tf - light.offset) / light.cycle_time)
                cycle_start = k * light.cycle_time + light.offset;
                
                % Gestisci il caso normale del verde
                if light.green_start <= light.green_end
                    abs_green_start = cycle_start + light.green_start;
                    abs_green_end = cycle_start + light.green_end;
                    
                    % Verifica sovrapposizione tra finestra verde e finestra di arrivo
                    if abs_green_start <= start_time + tf && abs_green_end >= t_min_arrival
                        overlap_start = max(abs_green_start, t_min_arrival);
                        overlap_end = min(abs_green_end, t_max_arrival);
                        
                        if overlap_start < overlap_end
                            % Crea un nodo nel mezzo della sovrapposizione
                            middle_time = (overlap_start + overlap_end) / 2;
                            Nodes(nodeId) = struct('id', nodeId, 't', middle_time, 'd', light.distance, 'int', i);
                            nodeId = nodeId + 1;
                        end
                    end
                else
                    % Gestisci il caso in cui il verde attraversa la mezzanotte del ciclo
                    % Prima parte del verde (da green_start a fine ciclo)
                    abs_green_start_1 = cycle_start + light.green_start;
                    abs_green_end_1 = cycle_start + light.cycle_time;
                    
                    if abs_green_start_1 <= start_time + tf && abs_green_end_1 >= t_min_arrival
                        overlap_start = max(abs_green_start_1, t_min_arrival);
                        overlap_end = min(abs_green_end_1, t_max_arrival);
                        
                        if overlap_start < overlap_end
                            middle_time = (overlap_start + overlap_end) / 2;
                            Nodes(nodeId) = struct('id', nodeId, 't', middle_time, 'd', light.distance, 'int', i);
                            nodeId = nodeId + 1;
                        end
                    end
                    
                    % Seconda parte del verde (da inizio ciclo a green_end)
                    abs_green_start_2 = cycle_start;
                    abs_green_end_2 = cycle_start + light.green_end;
                    
                    if abs_green_end_2 >= t_min_arrival && abs_green_start_2 <= start_time + tf
                        overlap_start = max(abs_green_start_2, t_min_arrival);
                        overlap_end = min(abs_green_end_2, t_max_arrival);
                        
                        if overlap_start < overlap_end
                            middle_time = (overlap_start + overlap_end) / 2;
                            Nodes(nodeId) = struct('id', nodeId, 't', middle_time, 'd', light.distance, 'int', i);
                            nodeId = nodeId + 1;
                        end
                    end
                end
            end
        end
        
        % Aggiungi il nodo finale
        Nodes(nodeId) = struct('id', nodeId, 't', start_time + tf, 'd', final_distance, 'int', nIntersections + 1);
    end
    
    nNodes = nodeId;
    
    % Se abbiamo pochi nodi, il problema potrebbe essere mal definito
    if nNodes < 2
        fprintf('[WARNING] Pochi nodi nel grafo (%d). Aggiunta una destinazione alternativa.\n', nNodes);
        nodeId = nodeId + 1;
        Nodes(nodeId) = struct('id', nodeId, 't', start_time + tf, 'd', final_distance, 'int', 1);
        nNodes = nodeId;
    end

    % Costruzione archi
    Edges = struct('from', {}, 'to', {}, 'w', {});
    edgeCount = 1;
    
    for i = 1:nNodes
        lvlA = Nodes(i).int;
        for j = 1:nNodes
            lvlB = Nodes(j).int;
            if lvlB == lvlA + 1  % Solo archi che collegano livelli consecutivi
                if Nodes(j).t > Nodes(i).t && Nodes(j).d > Nodes(i).d  % Movimento in avanti nel tempo e nello spazio
                    % Verifica che il nodo di destinazione (se è a un semaforo) sia a verde
                    if ~isempty(remaining_lights) && lvlB > 0 && lvlB <= nIntersections
                        light_idx = lvlB;
                        if light_idx <= length(remaining_lights)
                            if ~is_green(remaining_lights(light_idx), Nodes(j).t)
                                continue;
                            end
                        end
                    end
                    
                    % Calcola la velocità e il costo energetico dell'arco
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
    
    % Se non ci sono archi, creiamo un percorso diretto
    if edgeCount == 1
        fprintf('[WARNING] Nessun arco valido trovato. Creazione percorso diretto.\n');
        Edges(1) = struct('from', 1, 'to', nNodes, 'w', 100); % Costo arbitrario
    end

    % Trova il percorso ottimale
    [path, cost] = dijkstra(Nodes, Edges, 1, nNodes);
    fprintf('>>> Leader=%d, Costo ottimo=%.3f\n', leader_vehicle, cost);

    % Verifica validità del percorso trovato
    invalid_cost = isscalar(cost) && isinf(cost);
    invalid_path = isempty(path) || length(path) < 2;
    
    if invalid_cost || invalid_path
        fprintf('[WARNING] Percorso ottimo non trovato. Creo percorso diretto.\n');
        
        % Crea un percorso diretto dalla posizione attuale alla destinazione finale
        opt_t = [start_time; start_time + (final_distance - start_position)/15];
        opt_d = [start_position; final_distance];
        speeds = [15]; % Velocità costante moderata
    else
        % Percorso valido, estrai i tempi e le posizioni
        opt_nodes = Nodes(path);
        opt_t = arrayfun(@(n)n.t, opt_nodes);
        opt_d = arrayfun(@(n)n.d, opt_nodes);
        
        % Calcola le velocità target
        speeds = zeros(1, length(path) - 1);
        for k = 1:(length(path) - 1)
            d_ = opt_nodes(k+1).d - opt_nodes(k).d;
            t_ = opt_nodes(k+1).t - opt_nodes(k).t;
            speeds(k) = max(v_min, min(v_max, d_ / t_)); % Limita la velocità
        end
    end

% Verifica che speeds non sia vuoto
if isempty(speeds)
    speeds = [10]; % Velocità di sicurezza predefinita
end

    % Inizializza la simulazione con i nuovi veicoli
    n_vehicles = size(SIM_RUNS{1}.x, 2) / 2;  % Ottieni il numero di veicoli dalla prima simulazione
    
    % Identifica quali veicoli fanno parte di questo nuovo plotone
    current_run = length(SIM_RUNS);
    previous_run = current_run;  % Usa l'ultimo run disponibile
    
    % Cerca il run precedente che ha creato questo split
    for i = current_run:-1:1
        if isfield(SIM_RUNS{i}, 'splittedVehicles') && ...
           ~isempty(SIM_RUNS{i}.splittedVehicles) && ...
           ismember(leader_vehicle, SIM_RUNS{i}.splittedVehicles)
            previous_run = i;
            break;
        end
    end
    
    platoon_vehicles = SIM_RUNS{previous_run}.splittedVehicles;
    
    % Imposta le masse dei veicoli
    m_vehicles = 1000 * ones(1, n_vehicles);
    v_targets = speeds;

    % Parametri PID
    K_p_speed = 7000; K_i_speed = 0; K_d_speed = 0.1;
    K_p_dist = 2000;  K_i_dist = 0.8; K_d_dist = 0.4;
    t_CTH = 1.5;  
    d_init = 4;

    % Condizioni iniziali: tutti i veicoli del nuovo plotone partono fermi al semaforo
    x0 = zeros(2*n_vehicles, 1);
    for i = 1:n_vehicles
        if ismember(i, platoon_vehicles)
            if i == leader_vehicle
                x0(i) = start_position;  % Leader al semaforo
            else
                % I follower si posizionano dietro al leader
                idx_in_platoon = find(platoon_vehicles == i);
                leader_idx_in_platoon = find(platoon_vehicles == leader_vehicle);
                pos_diff = idx_in_platoon - leader_idx_in_platoon;
                x0(i) = start_position - d_init * pos_diff;
            end
            x0(n_vehicles + i) = 0;  % Velocità iniziale zero (fermi al semaforo)
        else
            % Veicoli che non fanno parte di questo plotone - posizione arbitraria
            x0(i) = -1000;
            x0(n_vehicles + i) = 0;
        end
    end

    % Esegui la simulazione
    t_span = [start_time, start_time + tf];
    [t_sim, x_sim] = ode45(@(t,x)system_dynamics_new_platoon( ...
        t, x, n_vehicles, m_vehicles, delta_func, traffic_lights, v_targets, t_CTH, ...
        K_p_speed, K_i_speed, K_d_speed, K_p_dist, K_i_dist, K_d_dist, ...
        leader_vehicle, start_time), t_span, x0);

    % Memorizza i risultati
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

    % Verifica semafori e trigger di velocità per il nuovo plotone
    check_red_light_violations(t_sim, x_sim, traffic_lights, T);
    check_velocity_triggers(t_sim, x_sim, traffic_lights);
end
function result = is_in_current_platoon(vehicle_id, current_run)
    global SIM_RUNS
    result = false;
    
    if current_run == 1
        % Per il primo run, verifica se il veicolo è stato splittato
        if isfield(SIM_RUNS{1}, 'splittedVehicles') && ...
           ~isempty(SIM_RUNS{1}.splittedVehicles) && ...
           ismember(vehicle_id, SIM_RUNS{1}.splittedVehicles)
            result = false;
        else
            result = true;
        end
        return;
    end
    
    % Per i run successivi, controlla se il veicolo è stato splittato dal run precedente
    prev_run = SIM_RUNS{current_run-1};
    if isfield(prev_run, 'splittedVehicles') && ...
       ~isempty(prev_run.splittedVehicles) && ...
       ismember(vehicle_id, prev_run.splittedVehicles)
        
        % Verifica che non sia stato splittato anche dal run corrente
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
    
    % Mostra informazioni sulla creazione del nuovo plotone
    current_platoon = N_PLATOON + 1;
    disp(['[INFO] Creazione NUOVO PLOTONE ' num2str(current_platoon) ' con LEADER=' ...
          num2str(violating_vehicle) ', riparto da tempo assoluto=' ...
          num2str(N_PLATOON*T) ', posizione=' num2str(stop_position)]);

    % Incrementa il contatore di plotoni prima di eseguire la simulazione
    start_offset = N_PLATOON*T;  
    N_PLATOON = N_PLATOON + 1;
    
    % Esegui la simulazione con il nuovo leader, partendo dalla posizione di stop
    run_optimizer_and_plot(violating_vehicle, start_offset, stop_position);
    
    % Pulisci lo stato delle violazioni per il nuovo plotone
    clear check_red_light_violations
end

function final_plot()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[final_plot] Nessun dato da plottare.');
        return;
    end
    
    % Creazione dei singoli grafici
    plot_optimal_trajectories_and_lights();
    plot_real_trajectories();
    plot_comparison();
    plot_leader_velocities();
    plot_energy_consumption();
    plot_inter_vehicle_distances();
end

function plot_optimal_trajectories_and_lights()
    global SIM_RUNS
    figure('Name','Posizioni Ottimali e Semafori', 'Position', [100, 100, 900, 600]);
    hold on;
    
    % Disegna semafori
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    [all_times, all_distances, all_colors] = prepare_traffic_light_data(traffic_lights);
    scatter(all_times, all_distances, 10, all_colors, 'filled', 'DisplayName', 'Semafori');
    
    % Prepara stili e colori
    markers = {'o','s','d','^','v','>','<'};
    colors = {'b','r','g','m','c','k',[0.8 0.4 0],[0.5 0.5 0.5],[0.2 0.6 0.8]};
    line_styles = {'-','--',':','-.'};
    legend_handles = [];
    legend_texts = {};
    
    % Per ogni run, disegna traiettorie ottimali
    for run_i = 1:length(SIM_RUNS)
        [legend_handles, legend_texts] = plot_run_optimal_trajectories(run_i, ...
                                          markers, colors, line_styles, ...
                                          legend_handles, legend_texts);
    end
    
    legend(legend_handles, legend_texts, 'Location', 'Best', 'NumColumns', 2);
    xlabel('Tempo [s]', 'FontSize', 12);
    ylabel('Posizione [m]', 'FontSize', 12);
    title('Traiettorie ottimali e semafori', 'FontSize', 14, 'FontWeight', 'bold');
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
    
    % Determina quali veicoli appartengono a questo plotone
    platoon_vehicles = get_platoon_vehicles(run_i);
    
    % Calcola velocità target
    v_targets = calculate_target_velocities(opt_t, opt_d);
    
    % Disegna la traiettoria del leader
    color_idx = mod(leader-1, length(colors))+1;
    line_idx = mod(run_i-1, length(line_styles))+1;
    marker_idx = mod(run_i-1, length(markers))+1;
    
    h = plot(opt_t, opt_d, [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 3);
    scatter(opt_t, opt_d, 70, colors{color_idx}, markers{marker_idx}, 'filled');
    
    legend_handles(end+1) = h;
    legend_texts{end+1} = ['Leader ' num2str(leader) ' (Plotone ' num2str(run_i) ')'];
    
    % Disegna le traiettorie ottimali dei follower
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
            legend_texts{end+1} = ['Follower ' num2str(v) ' (Plotone ' num2str(run_i) ')'];
        end
    end
end

function plot_real_trajectories()
    global SIM_RUNS
    
    figure('Name', 'Grafico Traiettorie Reali', 'Position', [150, 150, 900, 600]);
    hold on;
    
    % Disegna semafori
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    [all_times, all_distances, all_colors] = prepare_traffic_light_data(traffic_lights);
    scatter(all_times, all_distances, 10, all_colors, 'filled');
    
    % Configurazione stili e colori
    colors = {'b','r','g','m','c','y','k'};
    line_styles = {'-','-',':','-.'};
    
    % Crea una legenda per i veicoli
    legend_handles = [];
    legend_texts = {};
    
    % Per ogni plotone
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        leader = runData.leader;
        
        % Ottieni i veicoli in questo plotone
        platoon_vehicles = get_platoon_vehicles(run_i);
        
        % Traccia traiettorie per i veicoli in questo plotone
        for v = platoon_vehicles
            color_idx = mod(v-1, length(colors))+1;
            line_idx = mod(run_i-1, length(line_styles))+1;
            
            h = plot(t, x(:,v), [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 2);
            
            % Aggiungi alla legenda
            if v == leader
                legend_texts{end+1} = ['Leader ' num2str(v) ' (Plotone ' num2str(run_i) ')'];
            else
                legend_texts{end+1} = ['Veicolo ' num2str(v) ' (Plotone ' num2str(run_i) ')'];
            end
            legend_handles(end+1) = h;
        end
    end
    
    legend(legend_handles, legend_texts, 'Location', 'Best', 'NumColumns', 2);
    xlabel('Tempo [s]', 'FontSize', 12);
    ylabel('Posizione [m]', 'FontSize', 12);
    title('Traiettorie reali e semafori', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
end

function plot_comparison()
    global SIM_RUNS
    
    figure('Name', 'Confronto Reale vs Ottimali + Semafori', 'Position', [200, 200, 1000, 600]);
    hold on;
    
    % Disegna semafori
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    [all_times, all_distances, all_colors] = prepare_traffic_light_data(traffic_lights);
    scatter(all_times, all_distances, 10, all_colors, 'filled');
    
    % Configurazione stili
    colors_real = {'b','r','g','m','c','y','k'};
    colors_opt = {'b','r','g','m','c','k',[0.8 0.4 0],[0.5 0.5 0.5],[0.2 0.6 0.8]};
    
    % Crea elementi per la legenda
    legend_handles = [];
    legend_texts = {};
    
    % Per ogni plotone, traccia le traiettorie reali e ottimali
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        leader = runData.leader;
        
        % Ottieni i veicoli di questo plotone
        platoon_vehicles = get_platoon_vehicles(run_i);
        
        % Traiettorie reali (linea continua)
        for v = platoon_vehicles
            color_idx = mod(v-1, length(colors_real))+1;
            h_real = plot(t, x(:,v), [colors_real{color_idx}, '-'], 'LineWidth', 2);
            
            if v == leader
                legend_texts{end+1} = ['Leader ' num2str(v) ' (reale, Plotone ' num2str(run_i) ')'];
            else
                legend_texts{end+1} = ['Veicolo ' num2str(v) ' (reale, Plotone ' num2str(run_i) ')'];
            end
            legend_handles(end+1) = h_real;
        end
        
        % Traiettorie ottimali (linea tratteggiata)
        if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
            opt_t = runData.opt_t;
            opt_d = runData.opt_d;
            v_targets = calculate_target_velocities(opt_t, opt_d);
            
            % Leader ottimale
            color_idx = mod(leader-1, length(colors_opt))+1;
            h_opt_leader = plot(opt_t, opt_d, [colors_opt{color_idx}, '--'], 'LineWidth', 2);
            legend_texts{end+1} = ['Leader ' num2str(leader) ' (ottimale, Plotone ' num2str(run_i) ')'];
            legend_handles(end+1) = h_opt_leader;
            
            % Follower ottimali
            for v = platoon_vehicles
                if v ~= leader
                    follower_opt_d = calculate_follower_trajectory(v, leader, opt_t, opt_d, v_targets);
                    color_idx = mod(v-1, length(colors_opt))+1;
                    h_opt_follower = plot(opt_t, follower_opt_d, [colors_opt{color_idx}, '--'], 'LineWidth', 2);
                    legend_texts{end+1} = ['Veicolo ' num2str(v) ' (ottimale, Plotone ' num2str(run_i) ')'];
                    legend_handles(end+1) = h_opt_follower;
                end
            end
        end
    end
    
    legend(legend_handles, legend_texts, 'Location', 'Best', 'NumColumns', 2);
    xlabel('Tempo [s]', 'FontSize', 12);
    ylabel('Posizione [m]', 'FontSize', 12);
    title('Confronto Traiettorie (Reali vs Ottimali) + Semafori', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
end

function plot_leader_velocities()
    global SIM_RUNS
    
    % Identifica tutti i leader nelle simulazioni
    leaders = [];
    for run_i = 1:length(SIM_RUNS)
        leader = SIM_RUNS{run_i}.leader;
        if ~ismember(leader, leaders)
            leaders = [leaders, leader];
        end
    end
    
    % Crea un grafico per ogni leader
    for l = 1:length(leaders)
        current_leader = leaders(l);
        figure('Name', ['Velocità Leader ' num2str(current_leader)], 'Position', [250+l*50, 250+l*50, 800, 500]);
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
                legend_texts{end+1} = ['Velocità reale (Plotone ' num2str(run_i) ')'];
                
                % Aggiungi velocità target se disponibili
                if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
                    opt_t = runData.opt_t;
                    opt_d = runData.opt_d;
                    opt_v = calculate_target_velocities(opt_t, opt_d);
                    
                    % Modifica qui: usa un approccio diverso per plottare le velocità target
                    for i = 1:length(opt_v)
                        t_segment = [opt_t(i), opt_t(i+1)];
                        v_segment = [opt_v(i), opt_v(i)];
                        if i == 1
                            h_opt = plot(t_segment, v_segment, 'r--', 'LineWidth', 2);
                            legend_handles(end+1) = h_opt;
                            legend_texts{end+1} = ['Velocità target (Plotone ' num2str(run_i) ')'];
                        else
                            plot(t_segment, v_segment, 'r--', 'LineWidth', 2);
                        end
                    end
                    
                    % Aggiungi i punti per chiarezza
                    scatter(opt_t(1:end-1), opt_v, 50, 'r', 'filled');
                end
            end
        end
        
        legend(legend_handles, legend_texts, 'Location', 'Best');
        xlabel('Tempo [s]', 'FontSize', 12);
        ylabel('Velocità [m/s]', 'FontSize', 12);
        title(['Velocità Leader ' num2str(current_leader)], 'FontSize', 14, 'FontWeight', 'bold');
        grid on;
        ylim([0, 35]);
    end
end
function plot_energy_consumption()
    global SIM_RUNS
    
    % Calcola il consumo energetico per ogni plotone
    figure('Name', 'Consumo Energetico', 'Position', [300, 300, 800, 500]);
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
        
        % Parametri energetici
        b1 = 0.1;  
        b2 = 0.01;
        
        % Calcola il consumo energetico (approssimato)
        n_vehicles = size(x, 2)/2;
        total_energy = 0;
        
        % Determina i veicoli in questo plotone
        platoon_vehicles = get_platoon_vehicles(run_i);
        platoon_vehicle_count(run_i) = length(platoon_vehicles);
        
        for v = platoon_vehicles
            velocity = x(:, n_vehicles + v);
            dt = diff(t);
            
            % Calcola energia: E = Σ dt*(b1*v + b2*v^2)
            segment_energy = sum(dt .* (b1*velocity(1:end-1) + b2*velocity(1:end-1).^2));
            total_energy = total_energy + segment_energy;
        end
        
        platoon_energy(run_i) = total_energy;
    end
    
    % Visualizza il consumo energetico per platoon
    bar_h = bar(1:length(SIM_RUNS), platoon_energy);
    
    % Etichetta ogni barra con l'ID del leader e il numero di veicoli
    for i = 1:length(SIM_RUNS)
        text(i, platoon_energy(i) + max(platoon_energy)*0.03, ...
             ['Leader ' num2str(platoon_leaders(i)) ' (' num2str(platoon_vehicle_count(i)) ' veicoli)'], ...
             'HorizontalAlignment', 'center');
    end
    
    xlabel('Numero Plotone', 'FontSize', 12);
    ylabel('Consumo Energetico [J]', 'FontSize', 12);
    title('Consumo Energetico per Plotone', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
    set(gca, 'XTick', 1:length(SIM_RUNS));
end

function plot_inter_vehicle_distances()
    global SIM_RUNS
    
    % Figura per le distanze tra i veicoli in ogni plotone
    figure('Name', 'Distanze tra veicoli', 'Position', [350, 350, 1000, 600]);
    
    % Crea un subplot per ogni plotone
    n_plots = length(SIM_RUNS);
    rows = ceil(sqrt(n_plots));
    cols = ceil(n_plots/rows);
    
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        leader = runData.leader;
        
        % Crea subplot
        subplot(rows, cols, run_i);
        hold on;
        
        % Ottieni i veicoli di questo plotone
        platoon_vehicles = get_platoon_vehicles(run_i);
        
        % Calcola e visualizza le distanze tra i veicoli
        distances = [];
        labels = {};
        
        % Controlla se ci sono abbastanza veicoli nel plotone
        if length(platoon_vehicles) >= 2
            % Ordina i veicoli per posizione
            [~, order_idx] = sort(x(end, platoon_vehicles), 'descend');
            sorted_vehicles = platoon_vehicles(order_idx);
            
            % Calcola le distanze tra veicoli consecutivi
            for i = 1:length(sorted_vehicles)-1
                v1 = sorted_vehicles(i);
                v2 = sorted_vehicles(i+1);
                distance = x(:, v1) - x(:, v2);
                plot(t, distance, 'LineWidth', 2);
                distances(end+1) = distance(end);
                labels{end+1} = ['Distanza ' num2str(v1) '-' num2str(v2)];
            end
            
            title(['Distanze tra veicoli - Plotone ' num2str(run_i) ' (Leader ' num2str(leader) ')'], 'FontSize', 12);
            xlabel('Tempo [s]');
            ylabel('Distanza [m]');
            if ~isempty(labels)
                legend(labels, 'Location', 'Best');
            end
            grid on;
            
            % Controlla se distances ha valori validi prima di impostare i limiti dell'asse Y
            if ~isempty(distances) && all(isfinite(distances))
                ylim([0, max(max(distances), 10)]);
            else
                ylim([0, 10]); % Imposta un limite predefinito
            end
        else
            text(0.5, 0.5, 'Veicoli insufficienti per calcolare le distanze', 'HorizontalAlignment', 'center');
            axis([0 1 0 1]);
        end
    end
end

function rerun_optimizer_from_traffic_light(vehicle_id, start_time, start_position, traffic_light)
    clear system_dynamics_new_platoon
    clear get_current_v_target_indexed
    
    global SIM_RUNS
    global N_PLATOON
    
    % Mostra informazioni sulla creazione del nuovo plotone
    current_platoon = N_PLATOON + 1;
    disp(['[INFO] Creazione NUOVO PLOTONE ' num2str(current_platoon) ' con LEADER=' ...
          num2str(vehicle_id) ', attende semaforo verde a t=' ...
          num2str(start_time) ', posizione=' num2str(start_position)]);

    % IMPORTANTE: Incrementa il contatore di plotoni PRIMA di eseguire la nuova simulazione
    N_PLATOON = N_PLATOON + 1;
    
    % Esegui la simulazione con il nuovo leader, partendo dal semaforo
    run_optimizer_from_traffic_light(vehicle_id, start_time, start_position, traffic_light);
    
    % Pulisci lo stato delle violazioni per il nuovo plotone
    clear check_red_light_violations
end



function platoon_vehicles = get_platoon_vehicles(run_i)
    global SIM_RUNS
    n_vehicles = size(SIM_RUNS{1}.x, 2)/2;
    
    if run_i == 1
        % Per il primo plotone, inizialmente include tutti i veicoli
        platoon_vehicles = 1:n_vehicles;
        
        % Ma esclude i veicoli che sono stati divisi
        if isfield(SIM_RUNS{1}, 'splittedVehicles') && ~isempty(SIM_RUNS{1}.splittedVehicles)
            platoon_vehicles = setdiff(platoon_vehicles, SIM_RUNS{1}.splittedVehicles);
        end
        return;
    end
    
    % Per i plotoni successivi
    current_leader = SIM_RUNS{run_i}.leader;
    
    % Trova i veicoli che appartengono a questo plotone (quelli divisi dal plotone precedente)
    prev_run = SIM_RUNS{run_i - 1};
    if isfield(prev_run, 'splittedVehicles') && ~isempty(prev_run.splittedVehicles)
        platoon_vehicles = prev_run.splittedVehicles;
        
        % Escludi i veicoli che sono stati ulteriormente divisi
        if isfield(SIM_RUNS{run_i}, 'splittedVehicles') && ~isempty(SIM_RUNS{run_i}.splittedVehicles)
            platoon_vehicles = setdiff(platoon_vehicles, SIM_RUNS{run_i}.splittedVehicles);
        end
    else
        % Se non ci sono veicoli divisi dal plotone precedente, il plotone è vuoto
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
    
    % Parametri per il calcolo delle distanze di sicurezza
    t_CTH = 1.5;  % Costante di tempo per la distanza di sicurezza
    d_min = 1;    % Distanza minima
    
    % Offset iniziale basato sulla differenza di ID
    initial_offset = abs(follower_id - leader_id);
    
    % Inizializza la posizione del follower
    if follower_id > leader_id
        follower_opt_d(1) = opt_d(1) - initial_offset;
    else
        follower_opt_d(1) = opt_d(1) + initial_offset;
    end
    
    % Calcola la traiettoria del follower
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

function st=is_green(light, time)
    t_in_cycle= mod(time - light.offset, light.cycle_time);
    if light.green_start <= light.green_end
        st= (t_in_cycle >= light.green_start && t_in_cycle < light.green_end);
    else
        st= (t_in_cycle >= light.green_start || t_in_cycle < light.green_end);
    end
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

function [t_min, t_max] = velocity_pruning_from_position(traffic_lights, tf, final_distance, v_min, v_max, start_position, start_time)
    n = length(traffic_lights);
    d = [traffic_lights.distance];
    t_min = zeros(1, n); 
    t_max = zeros(1, n);
    
    % Per ogni semaforo, calcola il primo e ultimo tempo possibile di attraversamento
    for i = 1:n
        dist_from_start = d(i) - start_position;
        t_min(i) = start_time + dist_from_start/v_max;
        t_max(i) = start_time + dist_from_start/v_min;
        
        % Assicurati che il tempo di attraversamento coincida con semaforo verde
        t_min(i) = next_green(traffic_lights(i), t_min(i));
        t_max(i) = prev_green(traffic_lights(i), t_max(i));
        
        % Limite massimo basato sul tempo finale
        t_max(i) = min(t_max(i), tf - (final_distance - d(i))/v_max);
    end
    
    % Propaga i vincoli all'indietro
    for i = n:-1:2
        needed_t = (d(i) - d(i-1))/v_max;
        if t_max(i) > t_max(i-1) + needed_t
            t_max(i-1) = t_max(i) - needed_t;
            t_max(i-1) = prev_green(traffic_lights(i-1), t_max(i-1));
        end
    end
   end

function [t_min, t_max] = velocity_pruning_simple(start_position, final_distance, v_min, v_max)
    dist_to_final = final_distance - start_position;
    t_min = dist_to_final / v_max;
    t_max = dist_to_final / v_min;
end
function t_next=next_green(light, t)
    if is_green(light,t), t_next=t; return; end
    cyc= mod(t- light.offset, light.cycle_time);
    if cyc< light.green_start
        t_next= t+(light.green_start- cyc);
    else
        t_next= t+(light.cycle_time- cyc)+ light.green_start;
    end
end

function t_prev=prev_green(light, t)
    if is_green(light,t), t_prev=t; return; end
    cyc= mod(t- light.offset, light.cycle_time);
    if cyc>= light.green_end
        t_prev= t-(cyc- light.green_end);
    else
        t_prev= t- cyc- (light.cycle_time- light.green_end);
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

function plot_delta_velocities()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('Nessun dato disponibile per il plot dei delta delle velocità.');
        return;
    end
    
    runData = SIM_RUNS{1};
    t_sim = runData.t;
    n_vehicles = size(runData.x, 2) / 2;
    
    figure('Name','Valori per Calcolo Delta Velocità', 'Position', [100, 100, 1200, 800]);
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
        plot(t_sim, v_sim, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulata');
        plot(t_sim, v_opt, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Ottimale');
        plot(t_sim, diff_v, 'r-.', 'LineWidth', 1.5, 'DisplayName', 'Delta (calcolato)');
        xlabel('Tempo [s]');
        ylabel('Velocità [m/s]');
        title(['Veicolo ' num2str(v) ': v_{sim}, v_{ottimale} e Delta']);
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
        disp('Nessun dato da plottare per i trigger delle velocità.');
        return;
    end
    
    % Raccogliamo informazioni sugli split dei plotoni
    split_times = [];
    split_vehicles = [];
    split_reasons = {};
    
    for i = 1:length(SIM_RUNS)
        if i < length(SIM_RUNS)  % Se non è l'ultimo run
            if isfield(SIM_RUNS{i}, 'splittedVehicles') && ~isempty(SIM_RUNS{i}.splittedVehicles)
                next_leader = SIM_RUNS{i+1}.leader;
                split_time = SIM_RUNS{i+1}.offset;  % Il tempo di inizio del plotone successivo
                split_times(end+1) = split_time;
                split_vehicles(end+1) = next_leader;
                
                % Determina se lo split è avvenuto per semaforo rosso o trigger di velocità
                if split_time == i*30  % Se è un multiplo esatto del ciclo semaforico, probabilmente è per semaforo
                    split_reasons{end+1} = 'semaforo';
                else
                    split_reasons{end+1} = 'trigger';
                end
            end
        end
    end
    
    % Ora procediamo con il plotting normale
    runData = SIM_RUNS{1};
    t_sim = runData.t;
    n_vehicles = size(runData.x, 2) / 2;
    
    figure('Name', 'Trigger per Velocità e Split dei Plotoni', 'Position', [100, 100, 1200, 800]);
    
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
        
        % Plot delle velocità
        h1 = plot(t_sim, v_sim, 'b-', 'LineWidth', 1.5);
        h2 = plot(t_sim, v_opt, 'g--', 'LineWidth', 1.5);
        h3 = plot(t_sim, diff_v, 'r-.', 'LineWidth', 1.5);
        
        % Plot dei trigger di velocità
        h4 = plot(t_sim, trigger_state * (max(abs(diff_v))*0.8), 'm-', 'LineWidth', 2);
        
        % Plot dei punti di split per semaforo rosso e trigger
        h5 = [];
        h6 = [];
        
        y_lim = get(gca, 'YLim');
        max_y = y_lim(2);
        
        % Aggiungiamo indicatori verticali per gli split
        for idx = 1:length(split_times)
            if split_vehicles(idx) == v  % Questo veicolo è diventato leader
                if strcmp(split_reasons{idx}, 'semaforo')
                    % Linea verticale rossa per split da semaforo
                    h5 = plot([split_times(idx) split_times(idx)], [0 max_y], 'r-', 'LineWidth', 3);
                    text(split_times(idx), max_y*0.9, 'SPLIT (Semaforo)', 'Color', 'r', ...
                         'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'FontWeight', 'bold');
                else
                    % Linea verticale blu per split da trigger
                    h6 = plot([split_times(idx) split_times(idx)], [0 max_y], 'c-', 'LineWidth', 3);
                    text(split_times(idx), max_y*0.9, 'SPLIT (Trigger)', 'Color', 'c', ...
                         'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'FontWeight', 'bold');
                end
            end
        end
        
        % Legenda
        legend_handles = [h1, h2, h3, h4];
        legend_names = {'Velocità Simulata', 'Velocità Ottimale', 'Delta Velocità', 'Trigger Attivo'};
        
        if ~isempty(h5)
            legend_handles(end+1) = h5;
            legend_names{end+1} = 'Split per Semaforo Rosso';
        end
        if ~isempty(h6)
            legend_handles(end+1) = h6;
            legend_names{end+1} = 'Split per Trigger Velocità';
        end
        
        legend(legend_handles, legend_names, 'Location', 'best');
        
        xlabel('Tempo [s]');
        ylabel('Velocità [m/s]');
        title(['Veicolo ' num2str(v) ': Trigger e Split Detection']);
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
    
    % Tempo di inibizione del trigger dopo l'inizio di un nuovo plotone
    transitorio_inibizione = 10; % secondi di inibizione
    
    % Calcola il tempo relativo dall'inizio del plotone attuale
    current_offset = SIM_RUNS{current_run}.offset;
    tempo_relativo = t_abs(1) - current_offset;
    
    % Se siamo ancora nel transitorio, non controllare i trigger
    if tempo_relativo < transitorio_inibizione
        fprintf('[INFO] Transitorio attivo: trigger inibiti per altri %.1f secondi\n', ...
                transitorio_inibizione - tempo_relativo);
        return;
    end
    
    % Ottieni i dati ottimali dal run corrente
    runData = SIM_RUNS{current_run};
    t_sim = t_abs;
    opt_t = runData.opt_t;
    opt_d = runData.opt_d;
    
    % Per ogni veicolo nel plotone corrente
    for v = 1:n_vehicles
        % Salta i veicoli che non fanno parte di questo plotone
        if ~is_in_current_platoon(v, current_run)
            continue;
        end
        
        % Se è già il leader, salta
        if v == runData.leader
            continue;
        end
        
        % 1. VERIFICA SEMAFORI ROSSI ANTICIPATA
        % -----------------------------------
        % Verifica se il veicolo sta per passare con il rosso
        pos_v = x_sim(:, v);
        v_vel = x_sim(:, n_vehicles + v);
        
        % Previsione di posizione per i prossimi secondi
        look_ahead_time = 5; % secondi di anticipazione
        anticipation_steps = min(5, length(t_abs));
        
        % Per ogni semaforo, controlla se il veicolo lo attraverserà col rosso
        for L = 1:length(traffic_lights)
            light_d = traffic_lights(L).distance;
            
            % Calcola l'indice corrente del veicolo
            current_idx = length(pos_v);
            
            % Verifica se il veicolo è vicino ma non ha ancora superato il semaforo
            if pos_v(current_idx) < light_d && pos_v(current_idx) + v_vel(current_idx)*look_ahead_time >= light_d
                % Stima quando attraverserà il semaforo
                time_to_light = (light_d - pos_v(current_idx)) / max(v_vel(current_idx), 0.1);
                cross_time = t_abs(current_idx) + time_to_light;
                
                % Verifica se il semaforo sarà rosso quando il veicolo lo attraversa
                if ~is_green(traffic_lights(L), cross_time)
                    fprintf('\n[TRIGGER] Veicolo %d sta per passare col rosso a incrocio %d (t=%.2f s)\n', ...
                            v, L, t_abs(current_idx));
                    
                    if ~trigger_detected
                        trigger_detected = true;
                        fprintf('>> Veicolo %d diventa leader di un nuovo plotone per evitare semaforo rosso!\n', v);
                        
                        % Aggiorna i veicoli nel plotone corrente
                        SIM_RUNS{current_run}.splittedVehicles = v:n_vehicles;
                        
                        % Rilancia la simulazione con il nuovo leader
                        rerun_optimizer_for_trigger(v, t_abs(current_idx));
                        trigger_detected = false;
                        last_platoon_time = t_abs(current_idx);
                        return;
                    end
                end
            end
        end
        
        % 2. VERIFICA VELOCITÀ
        % -------------------
        % Calcola le differenze di velocità
        v_sim = x_sim(:, n_vehicles + v);
        pos_opt = interp1(opt_t, opt_d, t_sim, 'linear', 'extrap');
        v_opt = gradient(pos_opt, t_sim);
        offset_value = v_opt(end) - v_sim(end);
        diff_v = (v_opt - v_sim) - offset_value;
        
        % Calcola lo stato del trigger
        trigger_state = velocity_trigger(t_sim, diff_v, v_opt);
        
        % Se c'è almeno un trigger attivo e non è già stato rilevato
       if any(trigger_state) && ~trigger_detected
        trigger_idx = find(trigger_state, 1);
        trigger_time = t_sim(trigger_idx);
        
        % Ottieni la posizione attuale del veicolo
        current_position = x_sim(end, v);
            
        fprintf('\n[TRIGGER] Veicolo %d ha attivato un trigger di velocità a t=%.2f s, pos=%.2f m\n',...
                v, trigger_time, current_position);
        
        trigger_detected = true;
        fprintf('>> Veicolo %d si ferma e diventa leader di un nuovo plotone!\n', v);
        
        % Aggiorna i veicoli nel plotone corrente
        SIM_RUNS{current_run}.splittedVehicles = v:n_vehicles;
        
        % Rilancia la simulazione con il nuovo leader, fermandolo alla posizione attuale
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
    
    % Mostra informazioni sulla creazione del nuovo plotone
    current_platoon = N_PLATOON + 1;
    disp(['[INFO] TRIGGER ATTIVO: Creazione NUOVO PLOTONE ' num2str(current_platoon) ' con LEADER=' ...
          num2str(trigger_vehicle) ', riparto da tempo assoluto=' ...
          num2str(trigger_time) ', posizione=' num2str(stop_position)]);

    % Incrementa il contatore di plotoni
    start_offset = trigger_time;  
    N_PLATOON = N_PLATOON + 1;
    
    % Esegui la simulazione con il nuovo leader, fermandolo alla posizione specificata
    run_optimizer_and_plot(trigger_vehicle, start_offset, stop_position);
    
    % Pulisci lo stato
    clear check_red_light_violations
    clear check_velocity_triggers
end