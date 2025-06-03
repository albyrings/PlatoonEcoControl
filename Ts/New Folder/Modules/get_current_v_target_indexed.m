function vt = get_current_v_target_indexed(x_leader, traffic_lights, v_targets)
    idx = find(x_leader < [traffic_lights.distance], 1);
    if isempty(idx)
        vt = v_targets(end);
    else
        idx = min(idx, length(v_targets));
        vt = v_targets(idx);
    end
end