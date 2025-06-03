
plot_pruning_diamonds();

% filepath: plot_pruning_diamonds.m
function plot_pruning_diamonds()
    % Esempio di dati
    distanze   = [300, 600, 900];   % Distanze semafori o punti di interesse
    t_min_vals = [10,  24,  40];    % Tempi minimi (dal pruning)
    t_max_vals = [20,  35,  60];    % Tempi massimi (dal pruning)
    
    figure('Name','Rombi di Pruning','Position',[100,100,800,500]);
    hold on; grid on;
    xlabel('Tempo [s]');
    ylabel('Distanza [m]');
    title('Visualizzazione Rombi di Pruning');
    
    for i = 1:length(distanze)
        % Costruzione coordinate del "rombo"
        time_mid = (t_min_vals(i) + t_max_vals(i)) / 2;
        dist_mid = distanze(i);
        
        % Vertici del rombo
        x_coords = [t_min_vals(i), time_mid,  t_max_vals(i), time_mid];
        y_coords = [dist_mid,      dist_mid+5, dist_mid,     dist_mid-5];
        
        patch(x_coords, y_coords, 'y', 'FaceAlpha', 0.3, ...
              'EdgeColor', 'r', 'LineWidth', 1.5, ...
              'DisplayName', ['Rombo @ distanza = ' num2str(dist_mid)]);
        
        % Linee guida
        plot([t_min_vals(i), t_min_vals(i)], [dist_mid-5, dist_mid+5],'k--','HandleVisibility','off');
        plot([t_max_vals(i), t_max_vals(i)], [dist_mid-5, dist_mid+5],'k--','HandleVisibility','off');
    end
    
    legend('Location','best');
end