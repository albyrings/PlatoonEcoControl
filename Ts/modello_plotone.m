clear;
clc;
close all;

% Parametri dei veicoli
n_vehicles = 3;          % Numero di veicoli
m = 1000 * ones(1, n_vehicles); 
b1 = 450;
b2 = 450;
b3 = 1;
b4 = 3;

% Masse uguali per tutti i veicoli
delta =  @(t) 0 * (b1 + b2*(sin((1/b3)*t+b4) + 0.25*rand)); % Forza esterna (N)

% Distanza target
d_init = 4;
d_min = 1; 

% Parametri PID per il leader (controllo velocità)
K_p_speed = 7000;     
K_i_speed = 0;
K_d_speed = 0.7;

% Parametri PID per i follower (controllo distanza)
K_p_dist = 2000;  
K_i_dist = 0.8;  
K_d_dist = 0.4;  


% Velocità set point (target) del leader
v_target = 9;  % Velocità desiderata in m/s

% Tempo di separazione (CTH)
t_CTH = 1.5; % Tempo di separazione tra i veicoli

% Condizioni iniziali: posizioni e velocità
x0 = zeros(2 * n_vehicles, 1);
x0(1) = 0; % Il leader parte dalla posizione 0
for i = 2:n_vehicles
    x0(i) = -d_init * (i - 1);  
end

% Tempo di simulazione
t_span = [0 30];

% Risoluzione numerica
[t, x] = ode45(@(t, x) system_dynamics( ...
    t, x, n_vehicles, m, delta, d_min, v_target, t_CTH, ...
    K_p_speed, K_i_speed, K_d_speed, ...
    K_p_dist, K_i_dist, K_d_dist), ...
    t_span, x0);

% Grafico dei risultati
figure;
for i = 1:n_vehicles
    % Velocità
    subplot(2, n_vehicles, i);
    plot(t, x(:, n_vehicles + i));
    title(['Vel v' num2str(i)]);
    xlabel('Tempo [s]');
    ylabel('Velocità [m/s]');
    
    % Posizione
    subplot(2, n_vehicles, n_vehicles + i);
    plot(t, x(:, i));
    title(['Pos p' num2str(i)]);
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
end

% Calcolo delle distanze tra i veicoli
distances = zeros(n_vehicles-1, length(t));
for i = 2:n_vehicles
    distances(i-1, :) = x(:, 1) - x(:, i);  % distanza tra veicolo i e il leader
end

% Grafico delle distanze tra veicoli
figure;
for i = 1:n_vehicles-1
    plot(t, distances(i, :));
    hold on;
end
title('Distanza tra i veicoli');
xlabel('Tempo [s]');
ylabel('Distanza [m]');
legend(arrayfun(@(x) ['Distanza p' num2str(x) ' - p' num2str(x+1)], 1:n_vehicles-1, 'UniformOutput', false));

% Aggiunta del grafico con le posizioni dei veicoli
figure;
hold on;
for i = 1:n_vehicles
    plot(t, x(:, i), 'DisplayName', ['Posizione p' num2str(i)]);
end
title('Posizioni dei veicoli');
xlabel('Tempo [s]');
ylabel('Posizione [m]');
legend('show');

%% Funzione di dinamica

function dx = system_dynamics( ...
    t, x, n_vehicles, m, delta, d_min, v_target, t_CTH, ...
    K_p_speed, K_i_speed, K_d_speed, ...
    K_p_dist,  K_i_dist,  K_d_dist)

    dx = zeros(2 * n_vehicles, 1);

    % Calcolo del passo temporale (dt) in base al tempo t
    % dt è la differenza tra t e t-1 (il passo precedente)
    persistent t_prev
    if isempty(t_prev)
        t_prev = t; % Inizializzo t_prev al primo valore di t
    end
    
    
    t_prev = t;  % Aggiorno t_prev per il prossimo passo

    
    % Variabili PID come 'persistent'
    % (una per il leader, una per ogni follower)
    persistent error_integral_speed previous_error_speed
    persistent error_integral_dist  previous_error_dist
    
    if isempty(error_integral_speed) || isempty(error_integral_dist)
        error_integral_speed = 0;           % Leader (solo 1 valore)
        previous_error_speed  = 0;
        error_integral_dist  = zeros(n_vehicles, 1);  % follower
        previous_error_dist   = zeros(n_vehicles, 1);
    end
    
    % Stima del dt 
    dt = 0.01;  %   -- Non è "fisicamente" l'actual step di ode45
   
   
    
    for i = 1:n_vehicles
        
        % 1) La derivata della posizione è la velocità
        dx(i) = x(n_vehicles + i);
        
        if i == 1
            %% --- LEADER: PID SULLA VELOCITÀ ---
            velocity_error = v_target - x(n_vehicles + 1);
            
            % Integrale e derivata dell'errore
            error_integral_speed = error_integral_speed + velocity_error * dt;
            velocity_derivative  = (velocity_error - previous_error_speed) / dt;
            
            % Controllo PID della velocità -> "forza" di controllo
            U_leader = K_p_speed * velocity_error ...
                     + K_i_speed * error_integral_speed ...
                     + K_d_speed * velocity_derivative;
                 
            % Aggiorniamo lo stato "previous"
            previous_error_speed = velocity_error;
            
            % a = (U_leader + delta(t)) / m(1)
            dx(n_vehicles + 1) = (U_leader + delta(t)) / m(1);
            
            % Limiti sulla velocità (per sicurezza)
            max_speed = 30;  
            current_speed = x(n_vehicles + 1) + dx(n_vehicles + 1)*dt; 
            if current_speed < 0
                current_speed = 0;
            elseif current_speed > max_speed
                current_speed = max_speed;
            end
            
            % Sovrascrivo la derivata di velocità
            dx(n_vehicles + 1) = (current_speed - x(n_vehicles + 1)) / dt;
            
        else
            %% --- FOLLOWER i: PID SULLA DISTANZA RISPETTO AL PRIMO VEICOLO ---
            % Calcolo della distanza dal primo veicolo (leader)
            distance = x(i) - x(i-1);
            if distance > 0
                disp("distance > 0")
            end
            v_1 = x(n_vehicles + i);  % Velocità 
            d_CHT = -(d_min + t_CTH * v_1);

            
            % Errore: voglio distance = d_min (distanza dal leader)
            dist_error = d_CHT - distance; 
            
            % Aggiorno integrale e derivata per i-esimo veicolo
            error_integral_dist(i) = error_integral_dist(i) + dist_error * dt;
            distance_derivative = (dist_error - previous_error_dist(i)) / dt;
            previous_error_dist(i) = dist_error;
            
            % Calcolo forza di correzione (secondo PID)
            U_dist = K_p_dist * dist_error ...
                   + K_i_dist * error_integral_dist(i) ...
                   + K_d_dist * distance_derivative;
            
            % Accelerazione = (U_dist + delta(t)) / m(i)
            dx(n_vehicles + i) = (U_dist) / m(i);
            
            % Passo 1: calcola una velocità "ipotetica"
            hypotetical_speed = x(n_vehicles + i) + dx(n_vehicles + i)*dt;
            
            if hypotetical_speed < 0  
                % hypotetical_speed = 0; 
             disp("hypotetical_speed < 0  ")
            end
            
            % Finalmente scriviamo la derivata
            dx(n_vehicles + i) = (hypotetical_speed - x(n_vehicles + i)) / dt;
        end
    end
end
