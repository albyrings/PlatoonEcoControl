function t_prev = prev_green(light, t)
    if is_green(light,t), t_prev=t; return; end
    cyc = mod(t - light.offset, light.cycle_time);
    if cyc >= light.green_end
        t_prev = t - (cyc - light.green_end);
    else
        t_prev = t - cyc - (light.cycle_time - light.green_end);
    end
end