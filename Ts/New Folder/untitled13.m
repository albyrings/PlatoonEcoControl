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
plot_speed_trigger();

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
    %delta_func = @(t) 0 * (b1 + b2*(sin((1/b3)*t+b4) + 0.25*rand)); % Forza esterna (N)


    fprintf('\n[INFO] run_optimizer_and_plot(Leader=%d, offset=%.2f)\n', leader_vehicle, time_offset);

    % Crea e configura semafori
    traffic_lights = [
        create_traffic_light(300,   0, 10, T)
        create_traffic_light(600,  10, 20, T)
        create_traffic_light(900,  20, 30, T)
        create_traffic_light(1200,  0, 10, T)
        create_traffic_light(1550, 10, 20, T)
    ];

    % Esempio di "cra" min/max su t_min/t_max
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
    n_vehicles=5;
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
        'opt_t', opt_t + time_offset, ... % Salvo anche tempi ottimali
        'opt_d', opt_d);  % e distanze ottimali

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
    if dt <= 0, dt = 0.00001; end  % Valore minimo garantito per dt
    t_prev = t;  % Aggiorno t_prev dopo aver calcolato dt

    % Variabili PID
    persistent e_int_speed e_old_speed
    persistent e_int_dist  e_old_dist
    if isempty(e_int_speed), e_int_speed = 0; e_old_speed = 0; end
    if isempty(e_int_dist),  e_int_dist = zeros(n_vehicles, 1); e_old_dist = zeros(n_vehicles, 1); end

    abs_t = t + time_offset;  % tempo assoluto

    for i = 1:n_vehicles
        % La derivata della posizione è la velocità
        dx(i) = x(n_vehicles + i);
        
        if i == leader_vehicle
            % --- LEADER: controllo velocità ---
            vt = get_current_v_target_indexed(x(leader_vehicle), traffic_lights, v_targets);
            vel_err = vt - x(n_vehicles + i);

            % Integrale dell'errore (senza anti-windup)
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
                % Calcola la distanza dal veicolo precedente (corretta)
                dist = x(i-1) - x(i);  % Positiva quando il veicolo precedente è davanti
            else
                dist = 10;  % Caso speciale, non dovrebbe mai verificarsi
            end
            
            % Velocità attuale
            v_cur = x(n_vehicles + i);
            
            % Distanza desiderata basata sulla velocità corrente
            d_min_val = 1;
            d_desired = d_min_val + t_CTH * v_cur;
            
            % Errore come (distanza attuale - distanza desiderata)
            dist_err = dist - d_desired;

            % Integrale dell'errore (senza anti-windup)
            e_int_dist(i) = e_int_dist(i) + dist_err * dt;
            
            % Derivata dell'errore
            dist_deriv = (dist_err - e_old_dist(i)) / dt;
            e_old_dist(i) = dist_err;

            % Controllo PID -> forza
            U_dist = K_p_dist * dist_err + K_i_dist * e_int_dist(i) + K_d_dist * dist_deriv;
            dx(n_vehicles + i) = U_dist / m(i);

            % Limite inferiore velocità
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
                        
                        % Aggiorna i veicoli staccati nell'ultimo run
                        global SIM_RUNS
                        last_idx = length(SIM_RUNS);
                        SIM_RUNS{last_idx}.splittedVehicles = v:n_vehicles;
                        
                        % Ricalcola ottimizzazione per il nuovo leader
                        rerun_optimizer_for_new_leader(v, T);
                        
                        % Resetta il flag dopo aver gestito lo split
                        new_leader_detected = false;
                    end
                    return;
                end
            end
        end
    end
end

function rerun_optimizer_for_new_leader(violating_vehicle, T)
    % Azzera le persistent
    clear system_dynamics_new_platoon
    clear get_current_v_target_indexed
    clear check_red_light_violations

    global SIM_RUNS
    global N_PLATOON
    
    % L'ultimo run (il n-esimo) ha finito a ~150 secondi locali, ma in tempo assoluto = actual_time + ???
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
    % FIGURA 1: Posizione vs Tempo con posizioni ottimali e semafori
    figure('Name','Posizioni Ottimali e Semafori', 'Position', [100, 100, 1000, 600]);
    hold on;
    
    % Plot dei semafori come scatter plot
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    
    % Troviamo il tempo max nelle simulazioni
    max_time = 0;
    for i=1:length(SIM_RUNS)
        max_time = max(max_time, max(SIM_RUNS{i}.t));
    end
    times = 0:ceil(max_time);
    
    % Genero tutti i punti per i semafori
    all_times = [];
    all_distances = [];
    all_colors = [];
    
    for i=1:length(traffic_lights)
        for j=1:length(times)
            time = times(j);
            all_times = [all_times, time];
            all_distances = [all_distances, traffic_lights(i).distance];
            
            if is_green(traffic_lights(i), time)
                all_colors = [all_colors; [0, 1, 0]];  % Verde
            else
                all_colors = [all_colors; [1, 0, 0]];  % Rosso
            end
        end
    end
    
    % Plot semafori
    scatter(all_times, all_distances, 10, all_colors, 'filled', 'DisplayName', 'Semafori');
    
    % Plot delle traiettorie ottimali
    markers = {'o', 's', 'd', '^', 'v', '>', '<'};
    colors = {'b', 'r', 'g', 'm', 'c', 'k', [0.8 0.4 0], [0.5 0.5 0.5], [0.2 0.6 0.8]};
    line_styles = {'-', '--', ':', '-.'};
    
    % Prepara la legenda
    legend_handles = [];
    legend_texts = {};
    
    % Numero totale di veicoli
    n_vehicles = size(SIM_RUNS{1}.x, 2)/2;
    
    % Parametri per calcolo distanza
    t_CTH = 1.5;  % Constant Time Headway
    d_min = 1;    % Distanza minima di sicurezza
    
    % Prima tracciamo tutti i plotoni con i loro leader
    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        leader = runData.leader;
        
        if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
            opt_t = runData.opt_t;
            opt_d = runData.opt_d;
            
            % Identifica i veicoli in questo plotone esattamente come nel modello
            platoon_vehicles = [];
            if run_i == 1
                % Nel primo run, tutti i veicoli sono nel plotone
                platoon_vehicles = 1:n_vehicles;
            else
                % Nei run successivi, solo i veicoli nel campo splittedVehicles
                % dell'esecuzione precedente fanno parte di questo plotone
                prev_run = SIM_RUNS{run_i-1};
                if isfield(prev_run, 'splittedVehicles') && ~isempty(prev_run.splittedVehicles)
                    platoon_vehicles = prev_run.splittedVehicles;
                end
            end
            
            % Calcola le velocità target per questo plotone
            v_targets = [];
            for i = 1:length(opt_t)-1
                v = (opt_d(i+1) - opt_d(i)) / (opt_t(i+1) - opt_t(i));
                v_targets(i) = v;
            end
            
            % Plot del leader
            color_idx = mod(leader-1, length(colors))+1;
            line_idx = mod(run_i-1, length(line_styles))+1;
            marker_idx = mod(run_i-1, length(markers))+1;
            
            % Plot con marker sui punti nodali per il leader
            h = plot(opt_t, opt_d, [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 3);
            scatter(opt_t, opt_d, 70, colors{color_idx}, markers{marker_idx}, 'filled');
            
            % Aggiungi alla legenda
            legend_handles(end+1) = h;
            legend_texts{end+1} = ['Leader ' num2str(leader) ' (Plotone ' num2str(run_i) ')'];
            
            % Ordina i veicoli per posizione nel plotone
            % (importante per calcolare le posizioni in sequenza)
            if isempty(platoon_vehicles)
                ordered_vehicles = leader;
            else
                % Identifica l'ordine dei veicoli nel plotone (dal primo all'ultimo)
                % basandosi sulla posizione iniziale nella simulazione
                x_initial = runData.x(1, 1:n_vehicles);
                [~, idx] = sort(x_initial, 'descend');  % Ordina per posizione decrescente
                ordered_vehicles = idx;
            end
            
            % Plot dei follower in questo plotone usando la stessa politica del modello reale
            for vidx = 1:length(ordered_vehicles)
                v = ordered_vehicles(vidx);
                if v ~= leader
                    % Trova l'indice del veicolo che precede questo follower
                    preceding_vehicle_idx = find(ordered_vehicles == v) - 1;
                    if preceding_vehicle_idx < 1
                        % Se per qualche motivo non trova un veicolo precedente,
                        % usa il leader come riferimento
                        preceding_vehicle = leader;
                    else
                        preceding_vehicle = ordered_vehicles(preceding_vehicle_idx);
                    end
                    
                    % Calcola la posizione del follower basata sulla posizione
                    % del veicolo che lo precede e sulla distanza di sicurezza
                    follower_opt_t = opt_t;  % Stessi tempi
                    follower_opt_d = zeros(size(opt_d));
                    
                    % Per ogni punto sulla traiettoria
                    for i = 1:length(opt_t)
                        % Calcola la velocità corrente
                        if i == 1
                            current_v = v_targets(1);
                        else
                            if i <= length(v_targets)
                                current_v = v_targets(i-1);
                            else
                                current_v = v_targets(end);
                            end
                        end
                        
                        % Calcola la distanza di sicurezza basata sulla velocità
                        safety_distance = d_min + t_CTH * current_v;
                        
                        % Se è il leader, usa direttamente opt_d
                        if preceding_vehicle == leader
                            preceding_position = opt_d(i);
                        else
                            % Altrimenti usa la posizione calcolata per il veicolo precedente
                            % Trovo l'indice del veicolo precedente in ordered_vehicles
                            prec_idx = find(ordered_vehicles == preceding_vehicle);
                            prec_vidx = ordered_vehicles(prec_idx);
                            
                            % Uso la sua posizione ottimale calcolata in precedenza
                            preceding_position = follower_opt_d(i);
                        end
                        
                        % La posizione del follower è quella del veicolo precedente
                        % meno la distanza di sicurezza
                        follower_opt_d(i) = preceding_position - safety_distance;
                    end
                    
                    % Usa colori e stili diversi per ogni follower
                    follower_color_idx = mod(v-1, length(colors))+1;
                    follower_line_idx = mod(run_i-1, length(line_styles))+1;
                    follower_marker_idx = mod(v-1, length(markers))+1;
                    
                    % Plotta traiettoria follower con stile distintivo
                    h_follower = plot(follower_opt_t, follower_opt_d, [colors{follower_color_idx}, line_styles{follower_line_idx}], 'LineWidth', 2);
                    scatter(follower_opt_t, follower_opt_d, 40, colors{follower_color_idx}, markers{follower_marker_idx}, 'filled');
                    
                    % Aggiungi alla legenda
                    legend_handles(end+1) = h_follower;
                    legend_texts{end+1} = ['Follower ' num2str(v) ' (Plotone ' num2str(run_i) ')'];
                end
            end
        end
    end
    
    % Aggiungi legenda
    legend(legend_handles, legend_texts, 'Location', 'Best');
    
    % Configurazione grafico
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
    title('Traiettorie ottimali dei veicoli e stato semafori');
    grid on;
    
    % FIGURA 2: Grafico Traiettorie reali
    figure('Name','Grafico Traiettorie Reali', 'Position', [150, 150, 1000, 600]);
    hold on;
    
    % Plot semafori di nuovo
    scatter(all_times, all_distances, 10, all_colors, 'filled');
    
    % Plot delle traiettorie dei veicoli reali
    colors = {'b', 'r', 'g', 'm', 'c', 'y', 'k'};
    line_styles = {'-', '-', ':', '-.'};
    
    plotted_vehicles = []; % Inizializza la lista dei veicoli plottati
    
    for run_i=1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        
        % Verifica se ci sono veicoli da NON plottare in questo run
        if isfield(runData, 'splittedVehicles')
            splitted = runData.splittedVehicles;
        else
            splitted = [];
        end
        
        for v=1:size(x,2)/2
            if ismember(v, splitted) || ismember(v, plotted_vehicles)
                % Non plottare veicoli staccati o già plottati
                continue;
            end
            
            color_idx = mod(v-1, length(colors))+1;
            line_idx = mod(run_i-1, length(line_styles))+1;
            
            plot(t, x(:,v), [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 2);
            plotted_vehicles = [plotted_vehicles, v]; % Aggiungi il veicolo alla lista dei plottati
        end
    end
    
    % Configurazione plot posizione reale
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
    title('Traiettorie reali dei veicoli e stato semafori');
    grid on;
    
    % PROFILI DI VELOCITÀ: UN GRAFICO PER OGNI VEICOLO
    % Determino il numero massimo di veicoli da tutti i run
    max_vehicle_id = 0;
    for run_i=1:length(SIM_RUNS)
        n_vehicles = size(SIM_RUNS{run_i}.x, 2)/2;
        max_vehicle_id = max(max_vehicle_id, n_vehicles);
    end
    
    % Identifico i veicoli che appartengono al plotone 2 (se esiste)
    vehicles_in_platoon2 = [];
    if length(SIM_RUNS) >= 2
        if isfield(SIM_RUNS{1}, 'splittedVehicles') && ~isempty(SIM_RUNS{1}.splittedVehicles)
            vehicles_in_platoon2 = SIM_RUNS{1}.splittedVehicles;
        end
    end
    
    % Per ogni veicolo, creo un grafico separato di velocità
    for v=1:max_vehicle_id
        % Determina a quale plotone appartiene questo veicolo
        in_platoon2 = ismember(v, vehicles_in_platoon2);
        
        % Determina se è un leader
        is_leader = false;
        leader_of_run = 0;
        for run_i=1:length(SIM_RUNS)
            if SIM_RUNS{run_i}.leader == v
                is_leader = true;
                leader_of_run = run_i;
                break;
            end
        end
        
        % Prepara il titolo della figura
        if is_leader
            fig_title = ['Profilo di Velocità - Veicolo ' num2str(v) ' (Leader)'];
        else
            fig_title = ['Profilo di Velocità - Veicolo ' num2str(v) ' (Follower)'];
        end
        
        figure('Name', fig_title, 'Position', [200+v*30, 200+v*30, 800, 400]);
        hold on;
        
        % Legenda
        legend_handles = [];
        legend_texts = {};
        
        % Per ogni run, verifica se questo veicolo è presente e plotta i suoi dati
        for run_i=1:length(SIM_RUNS)
            runData = SIM_RUNS{run_i};
            
            % Per i veicoli del plotone 2, disegna solo i dati del plotone 2
            if in_platoon2 && run_i == 1
                continue;  % Salta il plotone 1 per i veicoli del plotone 2
            end
            
            % Per i veicoli solo nel plotone 1, disegna solo i dati del plotone 1
            if ~in_platoon2 && run_i > 1
                continue;  % Salta i plotoni successivi per i veicoli solo nel plotone 1
            end
            
            t_sim = runData.t;
            x_sim = runData.x;
            n_vehicles = size(x_sim, 2)/2;
            leader = runData.leader;
            
            % Verifica se questo veicolo è nel run corrente
            if v <= n_vehicles
                % Velocità reale
                v_real = x_sim(:, n_vehicles + v);  % Seconda metà della matrice x contiene velocità
                h_real = plot(t_sim, v_real, 'b-', 'LineWidth', 2);
                legend_handles(end+1) = h_real;
                legend_texts{end+1} = ['Velocità Reale (Plotone ' num2str(run_i) ')'];
                
                % Velocità target
                if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
                    % Calcola le velocità ottimali del leader
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
                    
                    % Per il leader, usa direttamente le velocità target
                    if v == leader
                        v_targets = leader_v_targets;
                        
                        % Prepara i punti per il grafico (formato a scalini)
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
                        
                        % Plotta i profili di velocità target
                        h_opt = plot(time_points, velocity_points, 'r--', 'LineWidth', 2);
                        scatter(opt_t(1:end-1), v_targets, 50, 'r', 'filled');
                        
                        legend_handles(end+1) = h_opt;
                        legend_texts{end+1} = ['Velocità Target (Plotone ' num2str(run_i) ')'];
                    else
                        % Per i follower, usa stesse velocità del leader
                        v_targets = leader_v_targets;
                        
                        % Prepara i punti per il grafico (formato a scalini)
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
                        
                        % Plotta i profili di velocità target
                        h_opt = plot(time_points, velocity_points, 'r--', 'LineWidth', 2);
                        
                        legend_handles(end+1) = h_opt;
                        legend_texts{end+1} = ['Velocità Target (da Leader ' num2str(leader) ')'];
                    end
                end
            end
        end
        
        % Configura grafico
        if ~isempty(legend_handles)
            legend(legend_handles, legend_texts, 'Location', 'Best');
        end
        
        % Altri dettagli grafico
        xlabel('Tempo [s]');
        ylabel('Velocità [m/s]');
        title(['Profilo di Velocità - Veicolo ' num2str(v)]);
        grid on;
        ylim([0, 35]);  % Limiti ragionevoli per la velocità
    end
    
    % Grafici di posizione per ogni veicolo
    for v=1:max_vehicle_id
        % Determina a quale plotone appartiene questo veicolo
        in_platoon2 = ismember(v, vehicles_in_platoon2);
        
        % Determina se è un leader
        is_leader = false;
        for run_i=1:length(SIM_RUNS)
            if SIM_RUNS{run_i}.leader == v
                is_leader = true;
                break;
            end
        end
        
        % Prepara il titolo della figura
        if is_leader
            fig_title = ['Posizione - Veicolo ' num2str(v) ' (Leader)'];
        else
            fig_title = ['Posizione - Veicolo ' num2str(v) ' (Follower)'];
        end
        
        figure('Name', fig_title, 'Position', [250+v*30, 250+v*30, 800, 400]);
        hold on;
        
        % Plot semafori di nuovo per il riferimento
        scatter(all_times, all_distances, 10, all_colors, 'filled');
        
        % Legenda
        legend_handles = [];
        legend_texts = {};
        
        % Per ogni run, verifica se questo veicolo è presente e plotta i suoi dati
        for run_i=1:length(SIM_RUNS)
            runData = SIM_RUNS{run_i};
            
            % Per i veicoli del plotone 2, disegna solo i dati del plotone 2
            if in_platoon2 && run_i == 1
                continue;  % Salta il plotone 1 per i veicoli del plotone 2
            end
            
            % Per i veicoli solo nel plotone 1, disegna solo i dati del plotone 1
            if ~in_platoon2 && run_i > 1
                continue;  % Salta i plotoni successivi per i veicoli solo nel plotone 1
            end
            
            t_sim = runData.t;
            x_sim = runData.x;
            n_vehicles = size(x_sim, 2)/2;
            leader = runData.leader;
            
            % Verifica se questo veicolo è nel run corrente
            if v <= n_vehicles
                % Posizione reale
                pos_real = x_sim(:, v);  % Prima metà della matrice x contiene posizioni
                h_real = plot(t_sim, pos_real, 'b-', 'LineWidth', 2);
                legend_handles(end+1) = h_real;
                legend_texts{end+1} = ['Posizione Reale (Plotone ' num2str(run_i) ')'];
                
                % Posizione ottimizzata
                if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
                    % Ordina i veicoli per posizione nel plotone
                    % (importante per calcolare le posizioni in sequenza)
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
                        % Identifica l'ordine dei veicoli nel plotone (dal primo all'ultimo)
                        % basandosi sulla posizione iniziale nella simulazione
                        x_initial = runData.x(1, 1:n_vehicles);
                        [~, idx] = sort(x_initial, 'descend');  % Ordina per posizione decrescente
                        ordered_vehicles = idx;
                    end
                    
                    % Calcola posizioni ottimali
                    opt_t = runData.opt_t;
                    opt_d = runData.opt_d;
                    
                    % Calcola le velocità target
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
                    
                    % Per il leader, usa direttamente le posizioni target
                    if v == leader
                        h_opt = plot(opt_t, opt_d, 'r--', 'LineWidth', 2);
                        scatter(opt_t, opt_d, 50, 'r', 'filled');
                        
                        legend_handles(end+1) = h_opt;
                        legend_texts{end+1} = ['Posizione Target (Plotone ' num2str(run_i) ')'];
                    else
                        % Per i follower, trova la posizione più ottimale
                        % seguendo lo stesso approccio della figura 1
                        
                        % Trova l'indice del veicolo che precede questo follower
                        follower_idx = find(ordered_vehicles == v);
                        preceding_vehicle_idx = follower_idx - 1;
                        
                        if preceding_vehicle_idx < 1 || preceding_vehicle_idx > length(ordered_vehicles)
                            % Se non trova un veicolo precedente, usa il leader
                            preceding_vehicle = leader;
                        else
                            preceding_vehicle = ordered_vehicles(preceding_vehicle_idx);
                        end
                        
                        % Calcola la posizione del follower basata sulla distanza di sicurezza
                        follower_opt_t = opt_t;  % Stessi tempi
                        follower_opt_d = zeros(size(opt_d));
                        
                        % Per ogni punto sulla traiettoria
                        for i = 1:length(opt_t)
                            % Calcola la velocità corrente
                            if i == 1
                                current_v = v_targets(1);
                            else
                                if i <= length(v_targets)
                                    current_v = v_targets(i-1);
                                else
                                    current_v = v_targets(end);
                                end
                            end
                            
                            % Calcola la distanza di sicurezza basata sulla velocità
                            safety_distance = d_min + t_CTH * current_v;
                            
                            % Se il veicolo precedente è il leader, usa la sua posizione
                            if preceding_vehicle == leader
                                preceding_position = opt_d(i);
                            else
                                % Costruisci una mappa per trovare la posizione ottimale dei follower precedenti
                                if exist('follower_positions', 'var') && isfield(follower_positions, ['v' num2str(preceding_vehicle)])
                                    preceding_position = follower_positions.(['v' num2str(preceding_vehicle)])(i);
                                else
                                    % Se non abbiamo la posizione calcolata, approssima
                                    % dalla posizione del leader meno la distanza di sicurezza
                                    rank_diff = abs(find(ordered_vehicles == preceding_vehicle) - find(ordered_vehicles == leader));
                                    preceding_position = opt_d(i) - safety_distance * rank_diff;
                                end
                            end
                            
                            % La posizione del follower è quella del veicolo precedente
                            % meno la distanza di sicurezza
                            follower_opt_d(i) = preceding_position - safety_distance;
                        end
                        
                        % Salva per uso futuro da altri follower
                        if ~exist('follower_positions', 'var')
                            follower_positions = struct();
                        end
                        follower_positions.(['v' num2str(v)]) = follower_opt_d;
                        
                        % Plotta la posizione ottimale del follower
                        h_opt = plot(follower_opt_t, follower_opt_d, 'r--', 'LineWidth', 2);
                        
                        legend_handles(end+1) = h_opt;
                        legend_texts{end+1} = ['Posizione Target (Plotone ' num2str(run_i) ')'];
                    end
                end
            end
        end
        
        % Configura grafico
        if ~isempty(legend_handles)
            legend(legend_handles, legend_texts, 'Location', 'Best');
        end
        
        % Altri dettagli grafico
        xlabel('Tempo [s]');
        ylabel('Posizione [m]');
        title(['Posizione - Veicolo ' num2str(v)]);
        grid on;
    end
    plot_vehicle_differences_full_platoon()
end

function plot_vehicle_differences_full_platoon()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[plot_vehicle_differences_full_platoon] Nessun dato da plottare.');
        return;
    end

    % Parametri per la distanza di sicurezza (usati per i follower)
    t_CTH = 1.5;  % Constant Time Headway
    d_min = 1;    % Distanza minima di sicurezza

    % Determina il numero massimo di veicoli in tutti i run
    max_vehicle_id = 0;
    for run_i = 1:length(SIM_RUNS)
        n_vehicles = size(SIM_RUNS{run_i}.x, 2) / 2;
        max_vehicle_id = max(max_vehicle_id, n_vehicles);
    end

    % Determina quali veicoli appartengono a un plotone diverso dal primo.
    vehicles_in_platoon2 = [];
    if length(SIM_RUNS) >= 2 && isfield(SIM_RUNS{1}, 'splittedVehicles')
        vehicles_in_platoon2 = SIM_RUNS{1}.splittedVehicles;
    end

    % Per ogni veicolo, scegli il run in cui il veicolo "passa dall'inizio alla fine"
    for v = 1:max_vehicle_id
        % Se il veicolo non appartiene al plotone 2, consideriamo il run 1;
        % altrimenti, scegliamo il primo run successivo al run 1 in cui è presente.
        if ~ismember(v, vehicles_in_platoon2)
            valid_run_i = 1;
        else
            valid_run_i = [];
            for run_i = 2:length(SIM_RUNS)
                n_vehicles = size(SIM_RUNS{run_i}.x, 2) / 2;
                if v <= n_vehicles
                    valid_run_i = run_i;
                    break;
                end
            end
            if isempty(valid_run_i)
                continue; % Non trovato un run valido
            end
        end

        runData = SIM_RUNS{valid_run_i};
        t_sim = runData.t;
        n_vehicles = size(runData.x, 2) / 2;
        
        % Se per qualche motivo il veicolo non è presente in questo run, salta
        if v > n_vehicles
            continue;
        end

        pos_sim = runData.x(:, v);           % posizione simulata
        vel_sim = runData.x(:, n_vehicles+v);  % velocità simulata

        % Uso i dati ottimali se disponibili
        if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
            opt_t = runData.opt_t;
            opt_d = runData.opt_d;
            % Se il veicolo è il leader del plotone, usa direttamente il profilo ottimale
            if v == runData.leader
                pos_opt = interp1(opt_t, opt_d, t_sim, 'linear', 'extrap');
                % Calcola la velocità ottimale come derivata approssimata dei nodi ottimali
                dt_opt = mean(diff(opt_t));
                vel_opt_calc = gradient(opt_d, dt_opt);
                vel_opt = interp1(opt_t, vel_opt_calc, t_sim, 'linear', 'extrap');
            else
                % Per un follower, assumiamo che il profilo ottimale sia una traslazione
                % del profilo del leader in base a una safety gap.
                dt_opt = mean(diff(opt_t));
                vel_leader = gradient(opt_d, dt_opt);
                safety_gap = d_min + t_CTH * mean(vel_leader);
                pos_opt = interp1(opt_t, opt_d, t_sim, 'linear', 'extrap') - safety_gap;
                vel_opt = interp1(opt_t, vel_leader, t_sim, 'linear', 'extrap');
            end

            % Calcolo delle differenze
            diff_vel = vel_opt - vel_sim;
            diff_pos = pos_opt - pos_sim;

            % Crea una figura con due subplot per il veicolo v
            figure('Name', ['Differenze per Veicolo ' num2str(v) ' (Plotone ' num2str(valid_run_i) ')'], ...
                'Position', [200+v*30, 200+v*30, 800, 600]);
            
            % Subplot 1: Δ Velocità
            subplot(2,1,1);
            hold on; grid on;
            plot(t_sim, diff_vel, 'LineWidth', 1.5, 'DisplayName', ['Run ' num2str(valid_run_i)]);
            title(['Δ Velocità (Ottimale - Simulata) per Veicolo ' num2str(v)]);
            xlabel('Tempo [s]');
            ylabel('Δ Velocità [m/s]');
            legend('show');
            
            % Subplot 2: Δ Posizione
            subplot(2,1,2);
            hold on; grid on;
            plot(t_sim, diff_pos, 'LineWidth', 1.5, 'DisplayName', ['Run ' num2str(valid_run_i)]);
            title(['Δ Posizione (Ottimale - Simulata) per Veicolo ' num2str(v)]);
            xlabel('Tempo [s]');
            ylabel('Δ Posizione [m]');
            legend('show');
        end
    end
end













function analyze_velocity_differences()
    global SIM_RUNS
    
    % Definiamo la soglia di triggering (m/s)
    trigger_threshold = 2.0;
    
    % Figura per il grafico delle differenze rispetto alla soglia
    figure('Name', 'Differenze Velocità rispetto alla Soglia', 'Position', [300, 300, 1200, 600]);
    hold on;
    title(['Differenze tra profili di velocità (rispetto a soglia = ' num2str(trigger_threshold) ' m/s)']);
    xlabel('Tempo [s]');
    ylabel('Differenza di velocità - Soglia [m/s]');
    grid on;
    
    % Colori per i diversi veicoli
    colors = {'b', 'r', 'g', 'm', 'c', 'k', [0.8 0.4 0], [0.5 0.5 0.5], [0.2 0.6 0.8]};
    line_styles = {'-', '--', ':', '-.'};
    
    % Legenda
    legend_handles = [];
    legend_texts = {};
    
    % Numero totale di veicoli
    n_vehicles = size(SIM_RUNS{1}.x, 2)/2;
    
    % Per ogni veicolo, analizza le differenze rispetto ai profili ottimali
    for v = 1:n_vehicles
        % Trova i run in cui questo veicolo è presente
        for run_i = 1:length(SIM_RUNS)
            runData = SIM_RUNS{run_i};
            
            % Verifica se questo veicolo fa parte di questo run e non è stato staccato
            is_in_this_run = true;
            
            % Verifica se il veicolo è stato staccato in un run precedente
            for prev_i = 1:run_i-1
                if isfield(SIM_RUNS{prev_i}, 'splittedVehicles') && ...
                   ismember(v, SIM_RUNS{prev_i}.splittedVehicles) && ...
                   v ~= SIM_RUNS{prev_i}.splittedVehicles(1)  % Non è il leader del nuovo plotone
                    is_in_this_run = false;
                    break;
                end
            end
            
            % Se è staccato in questo run, considera solo se è il leader
            if isfield(runData, 'splittedVehicles') && ...
               ismember(v, runData.splittedVehicles) && ...
               v ~= runData.leader
                is_in_this_run = false;
            end
            
            if ~is_in_this_run
                continue;
            end
            
            % Verifica che siano disponibili i dati ottimali
            if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d')
                continue;
            end
            
            % Tempo e velocità reali
            t_real = runData.t;
            x_sim = runData.x;
            v_real = x_sim(:, n_vehicles + v);
            leader = runData.leader;
            
            % Calcola velocità ottimale dai punti nodali per il leader
            opt_t = runData.opt_t;
            opt_d = runData.opt_d;
            opt_v = [];
            opt_t_mid = [];
            
            for i = 1:length(opt_t)-1
                vel = (opt_d(i+1) - opt_d(i)) / (opt_t(i+1) - opt_t(i));
                opt_v = [opt_v, vel];
                opt_t_mid = [opt_t_mid, (opt_t(i) + opt_t(i+1))/2];
            end
            
            % Interpola il profilo ottimale sul tempo reale
            v_opt_interp = interp1(opt_t_mid, opt_v, t_real, 'previous', 'extrap');
            
            % Calcola la differenza tra profilo ottimale e reale rispetto alla soglia
            v_diff = abs(v_opt_interp - v_real) - trigger_threshold;
            
            % Colori e stili
            color_idx = mod(v-1, length(colors))+1;
            line_idx = mod(run_i-1, length(line_styles))+1;
            
            % Plot della differenza
            h = plot(t_real, v_diff, 'Color', colors{color_idx}, 'LineStyle', line_styles{line_idx}, 'LineWidth', 1.5);
            
            % Aggiungi alla legenda
            if v == leader
                vehicle_type = 'Leader';
            else
                vehicle_type = 'Follower';
            end
            
            legend_handles(end+1) = h;
            legend_texts{end+1} = [vehicle_type ' ' num2str(v) ' (Plotone ' num2str(run_i) ')'];
        end
    end
    
    % Linea dello zero (indica soglia)
    xl = xlim;
    plot(xl, [0 0], 'k--', 'LineWidth', 1.5);
    text(xl(1) + 0.02*(xl(2)-xl(1)), 0.1, 'Soglia', 'FontSize', 10);
    
    % Aggiungi legenda
    legend(legend_handles, legend_texts, 'Location', 'EastOutside');
    
    % Regola limiti Y in modo sensato
    yl = ylim;
    if yl(1) > -2
        yl(1) = -2;
    end
    ylim(yl);
end

function analyze_velocity_differences_subplot()
    global SIM_RUNS
    
    % Definiamo la soglia di triggering (m/s)
    trigger_threshold = 2.0;
    
    % Numero totale di veicoli
    n_vehicles = size(SIM_RUNS{1}.x, 2)/2;
    
    % Determina disposizione subplot
    n_rows = ceil(sqrt(n_vehicles));
    n_cols = ceil(n_vehicles / n_rows);
    
    % Figura per il grafico delle differenze rispetto alla soglia
    figure('Name', 'Differenze Velocità per ogni veicolo', 'Position', [100, 100, 1200, 800]);
    
    % Colori per i diversi run di simulazione
    colors = {'b', 'r', 'g', 'm', 'c', 'k', [0.8 0.4 0], [0.5 0.5 0.5], [0.2 0.6 0.8]};
    line_styles = {'-', '--', ':', '-.'};
    
    % Per ogni veicolo, crea un subplot
    for v = 1:n_vehicles
        subplot(n_rows, n_cols, v);
        hold on;
        
        % Titolo del subplot
        title(['Veicolo ' num2str(v)]);
        
        % Legenda per questo subplot
        legend_handles = [];
        legend_texts = {};
        
        % Trova i run in cui questo veicolo è presente
        vehicle_data_found = false;
        
        for run_i = 1:length(SIM_RUNS)
            runData = SIM_RUNS{run_i};
            
            % Verifica se questo veicolo fa parte di questo run e non è stato staccato
            is_in_this_run = true;
            
            % Verifica se il veicolo è stato staccato in un run precedente
            for prev_i = 1:run_i-1
                if isfield(SIM_RUNS{prev_i}, 'splittedVehicles') && ...
                   ismember(v, SIM_RUNS{prev_i}.splittedVehicles) && ...
                   v ~= SIM_RUNS{prev_i}.splittedVehicles(1)  % Non è il leader del nuovo plotone
                    is_in_this_run = false;
                    break;
                end
            end
            
            % Se è staccato in questo run, considera solo se è il leader
            if isfield(runData, 'splittedVehicles') && ...
               ismember(v, runData.splittedVehicles) && ...
               v ~= runData.leader
                is_in_this_run = false;
            end
            
            if ~is_in_this_run
                continue;
            end
            
            % Verifica che siano disponibili i dati ottimali
            if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d')
                continue;
            end
            
            % Abbiamo trovato dati per questo veicolo
            vehicle_data_found = true;
            
            % Tempo e velocità reali
            t_real = runData.t;
            x_sim = runData.x;
            v_real = x_sim(:, n_vehicles + v);
            leader = runData.leader;
            
            % Calcola velocità ottimale dai punti nodali
            opt_t = runData.opt_t;
            opt_d = runData.opt_d;
            opt_v = [];
            opt_t_mid = [];
            
            for i = 1:length(opt_t)-1
                vel = (opt_d(i+1) - opt_d(i)) / (opt_t(i+1) - opt_t(i));
                opt_v = [opt_v, vel];
                opt_t_mid = [opt_t_mid, (opt_t(i) + opt_t(i+1))/2];
            end
            
            % Interpola il profilo ottimale sul tempo reale
            v_opt_interp = interp1(opt_t_mid, opt_v, t_real, 'previous', 'extrap');
            
            % Calcola la differenza tra profilo ottimale e reale rispetto alla soglia
            v_diff = abs(v_opt_interp - v_real) - trigger_threshold;
            
            % Colori e stili
            color_idx = mod(run_i-1, length(colors))+1;
            line_idx = mod(run_i-1, length(line_styles))+1;
            
            % Plot della differenza
            h = plot(t_real, v_diff, 'Color', colors{color_idx}, 'LineStyle', line_styles{line_idx}, 'LineWidth', 1.5);
            
            % Aggiungi alla legenda
            if v == leader
                legend_handles(end+1) = h;
                legend_texts{end+1} = ['Leader in Plotone ' num2str(run_i)];
            else
                legend_handles(end+1) = h;
                legend_texts{end+1} = ['Follower in Plotone ' num2str(run_i)];
            end
        end
        
        % Se non abbiamo trovato dati per questo veicolo, mostra un messaggio
        if ~vehicle_data_found
            text(0.5, 0.5, 'Nessun dato disponibile', 'HorizontalAlignment', 'center');
            axis([0 1 0 1]);
            continue;
        end
        
        % Linea dello zero (indica soglia)
        xl = xlim;
        plot(xl, [0 0], 'k--', 'LineWidth', 1.0);
        
        % Etichette asse x solo per l'ultima riga
        if v > (n_rows-1)*n_cols
            xlabel('Tempo [s]');
        end
        
        % Etichette asse y solo per la prima colonna
        if mod(v-1, n_cols) == 0
            ylabel('Diff velocità - Soglia [m/s]');
        end
        
        % Aggiungi legenda con dimensione carattere ridotta
        if ~isempty(legend_handles)
            legend(legend_handles, legend_texts, 'Location', 'Best', 'FontSize', 8);
        end
        
        % Regola limiti Y in modo sensato
        ylim([-3, 3]);
        
        % Aggiungi linea di annotazione per la soglia
        text(xl(1) + 0.05*(xl(2)-xl(1)), 0.2, ['Soglia = ' num2str(trigger_threshold) ' m/s'], 'FontSize', 7);
        
        % Aggiungi griglia
        grid on;
    end
    
    % Titolo globale
    sgtitle(['Differenze di velocità rispetto alla soglia (' num2str(trigger_threshold) ' m/s) per ogni veicolo'], 'FontSize', 14);
    
    % Regola spazi per maggiore leggibilità
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
    
    % Anche in funzioni ausiliarie che potrebbero avere variabili persistent
    clear next_green
    clear prev_green
    
    % Funzioni aggiuntive con variabili persistent
    clear dijkstra
end

function trigger_events = speed_trigger(t, diff_vel, opt_vel)
    % Parametri impostabili:
    trigger_threshold = 5;      % Soglia di attivazione trigger [m/s]
    disable_duration  = 10;     % Durata di disattivazione trigger in seguito a un cambio repentino [s]
    rapid_change_thresh = 2;    % Soglia per considerare un cambiamento come repentino (in m/s)
    initial_delay = 5;          % Periodo iniziale (in secondi) durante il quale il trigger è spento

    % Inizializza il vettore di output: 1 se trigger attivo, 0 altrimenti
    trigger_events = zeros(size(t));
    
    % Variabile che tiene traccia del tempo fino al quale il trigger è disabilitato
    disable_until = -inf;
    
    % Loop sui campioni temporali
    for i = 1:length(t)
        % Se siamo nel periodo iniziale, il trigger è spento
        if t(i) < initial_delay
            trigger_events(i) = 0;
        % Se siamo in un periodo di disattivazione dopo un cambio repentino
        elseif t(i) < disable_until
            trigger_events(i) = 0;
        else
            if abs(diff_vel(i)) > trigger_threshold
                trigger_events(i) = 1;
            else
                trigger_events(i) = 0;
            end
        end
        
        % Se non siamo al primo campione, controlla se l'ottimizzatore ha subito un cambio repentino
        if i > 1
            delta_opt = abs(opt_vel(i) - opt_vel(i-1));
            if delta_opt > rapid_change_thresh
                % Disattiva il trigger per 'disable_duration' secondi
                disable_until = t(i) + disable_duration;
            end
        end
    end
end

function plot_speed_trigger()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[plot_triggers_per_vehicle] Nessun dato da plottare.');
        return;
    end

    % Parametri per calcolare la safety gap (per i follower)
    t_CTH = 1.5;  % Constant Time Headway
    d_min = 1;    % Distanza minima di sicurezza

    % Determina il numero massimo di veicoli presenti in tutti i run
    max_vehicle_id = 0;
    for run_i = 1:length(SIM_RUNS)
        n_vehicles = size(SIM_RUNS{run_i}.x, 2) / 2;
        max_vehicle_id = max(max_vehicle_id, n_vehicles);
    end

    % Se esiste un plotone 2, recupera i veicoli splittati dal run 1
    vehicles_in_platoon2 = [];
    if length(SIM_RUNS) >= 2 && isfield(SIM_RUNS{1}, 'splittedVehicles')
        vehicles_in_platoon2 = SIM_RUNS{1}.splittedVehicles;
    end

    % Cicla per ogni veicolo
    for v = 1:max_vehicle_id
        % Se v non è presente nel primo plotone, scegli il primo run in cui v è presente.
        if ~ismember(v, vehicles_in_platoon2)
            valid_run_i = 1;
        else
            valid_run_i = [];
            for run_i = 2:length(SIM_RUNS)
                n_vehicles = size(SIM_RUNS{run_i}.x, 2) / 2;
                if v <= n_vehicles
                    valid_run_i = run_i;
                    break;
                end
            end
            if isempty(valid_run_i)
                continue;  % Non troviamo un run valido per questo veicolo
            end
        end

        runData = SIM_RUNS{valid_run_i};
        t_sim = runData.t;
        n_vehicles = size(runData.x, 2) / 2;
        if v > n_vehicles
            continue;
        end

        % Estrae la velocità simulata del veicolo
        vel_sim = runData.x(:, n_vehicles+v);

        % Calcolo del profilo ottimale
        if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
            opt_t = runData.opt_t;
            opt_d = runData.opt_d;
            % Per il leader usa direttamente il profilo ottimale
            if v == runData.leader
                pos_opt = interp1(opt_t, opt_d, t_sim, 'linear', 'extrap');
                dt_opt = mean(diff(opt_t));
                vel_opt_calc = gradient(opt_d, dt_opt);
                vel_opt = interp1(opt_t, vel_opt_calc, t_sim, 'linear', 'extrap');
            else
                % Per i follower, applichiamo una traslazione in base alla safety gap
                dt_opt = mean(diff(opt_t));
                vel_leader = gradient(opt_d, dt_opt);
                safety_gap = d_min + t_CTH * mean(vel_leader);
                pos_opt = interp1(opt_t, opt_d, t_sim, 'linear', 'extrap') - safety_gap;
                % Assumiamo la stessa velocità ottimale del leader
                vel_opt = interp1(opt_t, vel_leader, t_sim, 'linear', 'extrap');
            end

            % Calcolo della differenza tra velocità ottimale e simulata
            diff_vel = vel_opt - vel_sim;

            % Calcola lo stato del trigger per questo veicolo
            trigger_state = speed_trigger(t_sim, diff_vel, vel_opt);

            % Crea un grafico per il veicolo v
            figure('Name', ['Trigger per Veicolo ' num2str(v) ' (Run ' num2str(valid_run_i) ')'], ...
                   'Position', [200+v*30, 200+v*30, 900, 500]);
            hold on; grid on;
            % Plot della differenza di velocità
            h_diff = plot(t_sim, diff_vel, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Δ Velocità');
            % Plot del trigger, moltiplicato per una costante per renderlo visibile
            h_trig = plot(t_sim, trigger_state * (max(diff_vel)*0.8), 'r--', 'LineWidth', 2, ...
                          'DisplayName', 'Trigger Attivo');
            xlabel('Tempo [s]');
            ylabel('Differenza Velocità [m/s]');
            title(['Veicolo ' num2str(v) ' - Δ Velocità e Trigger']);
            legend([h_diff, h_trig], 'Location', 'best');
        end
    end
end