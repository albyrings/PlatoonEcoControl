function [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max)
    n = length(traffic_lights);
    d = [traffic_lights.distance];
    t_min = zeros(1,n); 
    t_max = zeros(1,n);
    t_min(1) = d(1)/v_max; 
    t_max(1) = d(1)/v_min;
    t_min(1) = next_green(traffic_lights(1), t_min(1));
    t_max(1) = prev_green(traffic_lights(1), t_max(1));

    for i=2:n
        dist_inc = d(i) - d(i-1);
        t_min(i) = t_min(i-1) + dist_inc/v_max;
        t_max(i) = t_max(i-1) + dist_inc/v_min;
        t_max(i) = min(t_max(i), tf - (final_distance - d(i))/v_max);
        t_min(i) = next_green(traffic_lights(i), t_min(i));
        t_max(i) = prev_green(traffic_lights(i), t_max(i));
    end

    for i=n:-1:2
        needed_t = (d(i) - d(i-1))/v_max;
        if t_max(i) > t_max(i-1) + needed_t
            t_max(i-1) = t_max(i) - needed_t;
            t_max(i-1) = prev_green(traffic_lights(i-1), t_max(i-1));
        end
    end
end