%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Real trajectories + traffic lights
%  – ogni veicolo compare in tutti i run a cui partecipa.
%    Se in un run è destinato a staccarsi, la sua linea termina al tempo
%    di split (offset del run successivo) e ricomincia nel nuovo run.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plot_real_trajectories()
    global SIM_RUNS
    if isempty(SIM_RUNS)
        disp('[plot_real_trajectories] No data in SIM_RUNS.');
        return;
    end

    figure('Name','Real Trajectories and Traffic Lights',...
           'Position',[200,150,900,600]); hold on; grid on;

    %── semafori ──────────────────────────────────────────────────────────
    [tt,dd,cc] = prepare_traffic_light_data(SIM_RUNS{1}.traffic_lights);
    scatter(tt,dd,10,cc,'filled');

    %── colori ciclici per i veicoli ──────────────────────────────────────
    veh_colors = [0 0 1; 1 0 0; 0 0.8 0; 0.8 0 0.8; 1 0.5 0];

    legH = []; legT = {};

    %----------------------------------------------------------------------
    for r = 1:length(SIM_RUNS)
        run        = SIM_RUNS{r};
        t_sim      = run.t;
        x_sim      = run.x;
        platoonV   = get_platoon_vehicles(r);

        % informazioni sul possibile split di *questo* run
        splitV   = [];
        splitT   = Inf;
        if r < length(SIM_RUNS) && isfield(run,'splittedVehicles') ...
                                 && ~isempty(run.splittedVehicles)
            splitV = run.splittedVehicles;
            splitT = SIM_RUNS{r+1}.offset;   % istante di split
        end

        %–– veicoli da disegnare (anche quelli che si staccheranno) ––––––
        pv_all = union(platoonV, splitV);
        pv_all = pv_all(:).';   % garantisce vettore riga (evita loop su colonna)

        %–– plot per ciascun veicolo “attivo” o destinato al distacco ––––
        for idxV = 1:numel(pv_all)
            v = pv_all(idxV);
            col = veh_colors(mod(v-1,size(veh_colors,1))+1,:);

            if ismember(v,splitV)    % veicolo che si staccherà
                mask = t_sim <= splitT;        % taglia al tempo di split
            else                      % veicolo che resterà nel plotone
                mask = true(size(t_sim));
            end

            h = plot(t_sim(mask), x_sim(mask,v), '-', ...
                     'LineWidth', 2, 'Color', col);
            legH = [legH h];
            legT = [legT {sprintf('V%d (Real, run %d)', v, r)}];
        end
    end

    %── rifiniture ────────────────────────────────────────────────────────
    xlim([0, max(cellfun(@(s) max(s.t), SIM_RUNS))]);
    ylim([0, max(cellfun(@(s) max(s.x(:,1)), SIM_RUNS))]);
    xlabel('Time [s]');  ylabel('Position [m]');
    title('Real Trajectories and Traffic Lights');
    legend(legH, legT, 'Location','Best');
end



function plot_optimal_trajectories_and_lights()
    global SIM_RUNS
    if isempty(SIM_RUNS),  disp('[plot_optimal_trajectories_and_lights] No data.');  return; end

    figure('Name','Optimal Trajectories and Traffic Lights',...
           'Position',[100,100,900,600]); hold on; grid on;

    % semafori
    [tt,dd,cc] = prepare_traffic_light_data(SIM_RUNS{1}.traffic_lights);
    scatter(tt,dd,10,cc,'filled');

    veh_colors = [0 0 1; 1 0 0; 0 0.8 0; 0.8 0 0.8; 1 0.5 0];

    legH = []; legT = {};

    %----------------------------------------------------------------------
    for r = 1:length(SIM_RUNS)
        run = SIM_RUNS{r};
        if ~isfield(run,'opt_t')||~isfield(run,'opt_d'), continue; end
        ot  = run.opt_t;  od = run.opt_d;   L = run.leader;

        % leader
        colL = veh_colors(mod(L-1,size(veh_colors,1))+1,:);
        hL   = plot(ot, od,'--','Color',colL,'LineWidth',2);
        legH = [legH hL];  legT = [legT sprintf('L%d (Opt, run %d)',L,r)];

        % followers rimasti
        pv = get_platoon_vehicles(r);
        for v = pv
            if v==L, continue; end
            col = veh_colors(mod(v-1,size(veh_colors,1))+1,:);
            offset = -4*(v-L);
            plot(ot, od+offset,'--','Color',col,'LineWidth',2);
            legH = [legH plot(nan,nan)]; %#ok trick per label una sola volta
            legT = [legT sprintf('V%d (Opt, run %d)',v,r)];
        end

        % followers che si staccheranno
        splitV = []; splitT = Inf;
        if r < length(SIM_RUNS) && isfield(run,'splittedVehicles') ...
                                 && ~isempty(run.splittedVehicles)
            splitV = run.splittedVehicles;
            splitT = SIM_RUNS{r+1}.offset;   % istante di split
        end
        if ~isempty(splitV)
            for v = splitV
                if v==L, continue; end
                col = veh_colors(mod(v-1,size(veh_colors,1))+1,:);
                offset = -4*(v-L);
                mask   = ot <= splitT;
                h = plot(ot(mask), od(mask)+offset, ':', 'Color',col,'LineWidth',2);
                legH = [legH h];
                legT = [legT sprintf('V%d (Opt, pre-split run %d)',v,r)];
            end
        end
    end

    xlim([0 max(cellfun(@(s) max(s.opt_t), SIM_RUNS))]);
    ylim([0 max(cellfun(@(s) max(s.opt_d), SIM_RUNS))]);
    xlabel('Time [s]'); ylabel('Position [m]');
    legend(legH, legT,'Location','Best');
end

