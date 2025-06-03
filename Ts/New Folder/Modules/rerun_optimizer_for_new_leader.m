function rerun_optimizer_for_new_leader(violating_vehicle, T)
    clear system_dynamics_new_platoon
    clear get_current_v_target_indexed
    clear check_red_light_violations
    
    global SIM_RUNS
    global N_PLATOON
    
    disp(['[INFO] Ricalcolo con NUOVO LEADER=', num2str(violating_vehicle), ...
          ', riparto da tempo assoluto=', num2str(N_PLATOON*T)]);

    start_offset = N_PLATOON*T;
    N_PLATOON = N_PLATOON + 1;
    run_optimizer_and_plot(violating_vehicle, start_offset);
end