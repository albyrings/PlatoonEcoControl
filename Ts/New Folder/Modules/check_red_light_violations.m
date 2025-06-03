function check_red_light_violations(t_abs, x_sim, traffic_lights, T)
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

                        global SIM_RUNS
                        last_idx = length(SIM_RUNS);
                        SIM_RUNS{last_idx}.splittedVehicles = v:n_vehicles;

                        rerun_optimizer_for_new_leader(v, T);
                        new_leader_detected = false;
                    end
                    return;
                end
            end
        end
    end
end