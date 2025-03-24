clear;
clc;
close all;
% Parametri di PSO per i follower
n_particles = 2000;  % Numero di particelle
n_iter = 10000;       % Numero di iterazioni
dim = 6;             % Dimensione del vettore dei parametri PID per i follower

% Limiti per i parametri PID (minimo e massimo per ciascun parametro)
lb = [0, 0, 0, 0, 0, 0];  % Limiti inferiori per PID (K_p_speed, K_i=0, K_d_speed, K_p_dist, K_i=0, K_d_dist)
ub = [10000, 1000, 1000, 10000, 1000, 1000];  % Limiti superiori per PID

% Inizializzazione delle particelle
positions = lb + (ub - lb) .* rand(n_particles, dim);  % Posizioni casuali delle particelle
velocities = zeros(n_particles, dim);  % Velocità iniziale

% Migliori globali e locali
pbest = positions;  % Migliori soluzioni locali
pbest_score = arrayfun(@(i) pid_error_follower(positions(i, :)), 1:n_particles);  % Errori locali
[gbest_score, gbest_idx] = min(pbest_score);  % Migliore soluzione globale
gbest = pbest(gbest_idx, :);  % Miglior posizione globale

% PSO Loop per ottimizzare i parametri PID dei follower
for iter = 1:n_iter
    for i = 1:n_particles
        % Calcola il nuovo errore per la particella i
        error = pid_error_follower(positions(i, :));
        
        % Aggiornamento del miglior personale (pbest)
        if error < pbest_score(i)
            pbest_score(i) = error;
            pbest(i, :) = positions(i, :);
        end
        
        % Aggiornamento della migliore globale (gbest)
        if error < gbest_score
            gbest_score = error;
            gbest = positions(i, :);
        end
    end
    
    % Aggiornamento delle velocità e posizioni
    w = 0.5;  % Peso di inerzia
    c1 = 1.5; % Coefficiente di accelerazione per pbest
    c2 = 1.5; % Coefficiente di accelerazione per gbest
    
    for i = 1:n_particles
        velocities(i, :) = w * velocities(i, :) + c1 * rand(1, dim) .* (pbest(i, :) - positions(i, :)) + c2 * rand(1, dim) .* (gbest - positions(i, :));
        positions(i, :) = positions(i, :) + velocities(i, :);
        
        % Limita la posizione per rimanere nei limiti definiti
        positions(i, :) = max(min(positions(i, :), ub), lb);
    end
    
    % Visualizzazione del progresso
    disp(['Iterazione: ' num2str(iter) ' - Migliore errore: ' num2str(gbest_score)]);
    disp(gbest);
end

% Risultato finale
disp('Parametri PID Ottimizzati per i Follower:');
disp(gbest);

% Funzione di errore per la minimizzazione (ISE) per i follower
function error = pid_error_follower(K_pid)

    % Estrazione dei parametri PID ottimizzati per i follower
    K_p_speed = K_pid(1);
    K_i_speed = K_pid(2);
    K_d_speed = K_pid(3);
    
    K_p_dist = K_pid(4);
    K_i_dist = K_pid(5);  % K_i è fisso a zero
    K_d_dist = K_pid(6);

    % Parametri del veicolo (come nel tuo codice)
    n_vehicles = 2;  % Solo il leader e un follower
    m = 1000 * ones(1, n_vehicles);
    delta = @(t) 0 * cos(t);
    d_min = 4;
    v_target = 9;  % Velocità target
    t_CTH = 1.5;  % Tempo di separazione tra i veicoli
    x0 = zeros(2 * n_vehicles, 1);
    
    % Simulazione del sistema con i parametri PID attuali
    t_span = [0 500];
    [t, x] = ode45(@(t, x) system_dynamics_follower( ...
        t, x, n_vehicles, m, delta, d_min, v_target, t_CTH, ...
        K_p_speed, K_i_speed, K_d_speed, K_p_dist, K_i_dist, K_d_dist), ...
        t_span, x0);
    
    % Calcolare l'errore in base alla distanza tra i veicoli e alla velocità del follower
    distance_error = x(:, 1) - x(:, 2);  % Distanza tra il leader e il follower
    velocity_error = v_target - x(n_vehicles + 1, :);  % Velocità del follower
    error = sum(sum(distance_error.^2)) + sum(velocity_error.^2);  % Errore quadratico integrale per velocità e distanza

end

% Funzione che descrive la dinamica del sistema (per il leader e follower)
function dx = system_dynamics_follower( t, x, n_vehicles, m, delta, d_min, v_target, t_CTH, K_p_speed, K_i_speed, K_d_speed, K_p_dist, K_i_dist, K_d_dist)

    dx = zeros(2 * n_vehicles, 1);

    % Calcolo del passo temporale (dt) in base al tempo t
    persistent t_prev
    if isempty(t_prev)
        t_prev = t; % Inizializzo t_prev al primo valore di t
    end
    t_prev = t;  % Aggiorno t_prev per il prossimo passo

    % Variabili PID
    persistent error_integral_speed previous_error_speed
    persistent error_integral_dist previous_error_dist
    
    if isempty(error_integral_speed) || isempty(error_integral_dist)
        error_integral_speed = 0;
        previous_error_speed = 0;
        error_integral_dist = 0;
        previous_error_dist = 0;
    end
    
    % Calcolo della velocità del leader e del follower
    dx(1) = x(n_vehicles + 1);  % Velocità del leader
    dx(2) = x(n_vehicles + 2);  % Velocità del follower
    
    % Controllo PID per la velocità del follower
    velocity_error = v_target - x(n_vehicles + 2);
    
    % Integrale e derivata dell'errore per la velocità
    error_integral_speed = error_integral_speed + velocity_error * 0.01;
    velocity_derivative  = (velocity_error - previous_error_speed) / 0.01;
    
    % Controllo PID della velocità del follower
    U_follower_speed = K_p_speed * velocity_error ...
                     + K_i_speed * error_integral_speed ...
                     + K_d_speed * velocity_derivative;
    
    % Aggiorniamo lo stato "previous"
    previous_error_speed = velocity_error;
    
    % Calcolo della distanza tra il leader e il follower
    distance = x(1) - x(2);
    
    % Errore nella distanza
    dist_error = distance - d_min;
    
    % Integrale e derivata dell'errore per la distanza
    error_integral_dist = error_integral_dist + dist_error * 0.01;
    distance_derivative = (dist_error - previous_error_dist) / 0.01;
    
    % Controllo PID della distanza del follower
    U_follower_dist = K_p_dist * dist_error ...
                    + K_i_dist * error_integral_dist ...
                    + K_d_dist * distance_derivative;
    
    % Aggiorniamo lo stato "previous"
    previous_error_dist = dist_error;
    
    % Accelerazione del follower
    dx(n_vehicles + 2) = U_follower_speed / m(2) + U_follower_dist / m(2);
    
end
