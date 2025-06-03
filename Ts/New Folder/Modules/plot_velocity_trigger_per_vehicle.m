function plot_velocity_trigger_per_vehicle()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('Nessun dato da plottare per i trigger delle velocità.');
        return;
    end
    
    runData = SIM_RUNS{1};
    t_sim = runData.t;
    n_vehicles = size(runData.x, 2) / 2;
    
    figure('Name','Trigger per Velocità per Veicolo', 'Position',[100,100,1200,800]);
    for v = 1:n_vehicles
        v_sim = runData.x(:, n_vehicles + v);
        if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d')
            continue;
        end
        opt_t = runData.opt_t;
        opt_d = runData.opt_d;
        
        % Velocità ottimale
        pos_opt = interp1(opt_t, opt_d, t_sim, 'linear', 'extrap');
        v_opt = gradient(pos_opt, t_sim);
        
        % Offset (evita picchi al termine)
        offset_value = v_opt(end) - v_sim(end);
        diff_v = (v_opt - v_sim) - offset_value;

        % Calcolo del trigger
        trigger_state = velocity_trigger(t_sim, diff_v, v_opt);
        
        subplot(n_vehicles, 1, v);
        hold on; grid on;
        plot(t_sim, v_sim, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Velocità Simulata');
        plot(t_sim, v_opt, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Velocità Ottimale');
        plot(t_sim, diff_v, 'r-.', 'LineWidth', 1.5, 'DisplayName', 'Delta Velocità');
        plot(t_sim, trigger_state * (max(diff_v)*0.8), 'm-', 'LineWidth', 2, 'DisplayName', 'Trigger Attivo');
        xlabel('Tempo [s]');
        ylabel('Velocità [m/s]');
        title(['Veicolo ' num2str(v) ': Vel. Simulata, v_{ottimale} e Delta']);
        legend('Location','best');
    end
end