function final_plot()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[final_plot] Nessun dato da plottare.');
        return;
    end

    

    figure('Name','Posizioni Ottimali e Semafori');
    hold on;
    traffic_lights = SIM_RUNS{1}.traffic_lights;
    max_time = 0;
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
    scatter(all_times, all_distances, 10, all_colors, 'filled', 'DisplayName', 'Semafori');

    markers = {'o','s','d','^','v','>','<'};
    colors = {'b','r','g','m','c','k',[0.8 0.4 0],[0.5 0.5 0.5],[0.2 0.6 0.8]};
    line_styles = {'-','--',':','-.'};
    legend_handles = [];
    legend_texts = {};
    n_vehicles = size(SIM_RUNS{1}.x, 2)/2;

    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        leader  = runData.leader;
        if isfield(runData,'opt_t') && isfield(runData,'opt_d')
            opt_t = runData.opt_t;
            opt_d = runData.opt_d;

            color_idx = mod(leader-1, length(colors))+1;
            line_idx = mod(run_i-1, length(line_styles))+1;
            marker_idx = mod(run_i-1, length(markers))+1;
            h = plot(opt_t, opt_d, [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 3);
            scatter(opt_t, opt_d, 70, colors{color_idx}, markers{marker_idx}, 'filled');
            legend_handles(end+1) = h;
            legend_texts{end+1} = ['Leader ' num2str(leader) ' (Plotone ' num2str(run_i) ')'];
        end
    end

    legend(legend_handles, legend_texts, 'Location', 'Best');
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
    title('Traiettorie ottimali e semafori');
    grid on;

    figure('Name','Grafico Traiettorie Reali');
    hold on;
    scatter(all_times, all_distances, 10, all_colors, 'filled');
    colors = {'b','r','g','m','c','y','k'};
    line_styles = {'-','-',':','-.'};
    plotted_vehicles = [];
    for run_i=1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        if isfield(runData,'splittedVehicles'), splitted = runData.splittedVehicles;
        else, splitted = []; end
        
        for v=1:size(x,2)/2
            if ismember(v, splitted) || ismember(v, plotted_vehicles), continue; end
            color_idx = mod(v-1, length(colors))+1;
            line_idx = mod(run_i-1, length(line_styles))+1;
            plot(t, x(:,v), [colors{color_idx}, line_styles{line_idx}], 'LineWidth', 2);
            plotted_vehicles = [plotted_vehicles, v];
        end
    end
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
    title('Traiettorie reali e semafori');
    grid on;

    figure('Name','Confronto Reale vs Ottimali + Semafori');
    hold on;
    scatter(all_times, all_distances, 10, all_colors, 'filled');

    colors_real = {'b','r','g','m','c','y','k'};
    line_styles_real = {'-'};
    plotted_vehicles = [];

    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        t = runData.t;
        x = runData.x;
        if isfield(runData,'splittedVehicles')
            splitted = runData.splittedVehicles;
        else
            splitted = [];
        end
        for v = 1:size(x,2)/2
            if ismember(v, splitted) || ismember(v, plotted_vehicles)
                continue;
            end
            color_idx = mod(v-1, length(colors_real))+1;
            plot(t, x(:,v), [colors_real{color_idx}, line_styles_real{1}], 'LineWidth', 2);
            plotted_vehicles = [plotted_vehicles, v];
        end
    end

    colors_opt = {'b','r','g','m','c','k',[0.8 0.4 0],[0.5 0.5 0.5],[0.2 0.6 0.8]};
    line_style_opt = '--';
    n_vehicles = size(SIM_RUNS{1}.x,2)/2;

    for run_i = 1:length(SIM_RUNS)
        runData = SIM_RUNS{run_i};
        if ~isfield(runData, 'opt_t') || ~isfield(runData, 'opt_d'), continue; end
        opt_t = runData.opt_t; opt_d = runData.opt_d;
        leader = runData.leader;

        for v = 1:n_vehicles
            color_idx = mod(v-1, length(colors_opt)) + 1;
            if v == leader
                plot(opt_t, opt_d, [colors_opt{color_idx}, line_style_opt], 'LineWidth', 2);
            else
                follower_opt_d = zeros(size(opt_d));
                t_CTH = 1.5; 
                d_min = 1;
                offset = abs(v - leader);
                if v > leader, follower_opt_d(1) = opt_d(1) - offset;
                else,          follower_opt_d(1) = opt_d(1) + offset;
                end
                v_targets_local = [];
                for idx_opt = 1:length(opt_t)-1
                    v_ = (opt_d(idx_opt+1) - opt_d(idx_opt)) / (opt_t(idx_opt+1) - opt_t(idx_opt));
                    v_targets_local(idx_opt) = v_;
                end
                for idx_opt = 2:length(opt_t)
                    if v > leader
                        desired_gap = d_min + t_CTH * v_targets_local(idx_opt-1);
                        follower_opt_d(idx_opt) = opt_d(idx_opt) - desired_gap*(v-leader);
                    else
                        desired_gap = d_min + t_CTH * v_targets_local(idx_opt-1);
                        follower_opt_d(idx_opt) = opt_d(idx_opt) + desired_gap*(leader-v);
                    end
                end
                plot(opt_t, follower_opt_d, [colors_opt{color_idx}, line_style_opt], 'LineWidth', 2);
            end
        end
    end
    xlabel('Tempo [s]');
    ylabel('Posizione [m]');
    title('Confronto Traiettorie (Reali vs Ottimali) + Semafori');
    grid on;

    leaders = [];
    for run_i=1:length(SIM_RUNS)
        if ~ismember(SIM_RUNS{run_i}.leader, leaders)
            leaders = [leaders, SIM_RUNS{run_i}.leader];
        end
    end

    for l=1:length(leaders)
        current_leader = leaders(l);
        figure('Name',['Velocità Leader ' num2str(current_leader)]);
        hold on;
        for run_j=1:length(SIM_RUNS)
            runData = SIM_RUNS{run_j};
            if runData.leader == current_leader
                t = runData.t;
                x = runData.x;
                nv = size(x,2)/2;
                leader_velocity = x(:, nv + current_leader);
                plot(t, leader_velocity, 'b-', 'LineWidth', 2);

                if isfield(runData, 'opt_t') && isfield(runData, 'opt_d')
                    opt_t_local = runData.opt_t;
                    opt_d_local = runData.opt_d;
                    opt_v = [];
                    for i = 1:length(opt_t_local)-1
                        vv = (opt_d_local(i+1) - opt_d_local(i)) / ...
                             (opt_t_local(i+1) - opt_t_local(i));
                        opt_v = [opt_v, vv];
                    end
                    plot([opt_t_local(1:end-1); opt_t_local(2:end)], ...
                        [opt_v; opt_v], 'r--', 'LineWidth', 2);
                    scatter(opt_t_local(1:end-1), opt_v, 50, 'r', 'filled');
                end
            end
        end
        xlabel('Tempo [s]'); ylabel('Velocità [m/s]');
        title(['Velocità Leader ' num2str(current_leader)]);
        grid on; ylim([0, 35]);
    end
end