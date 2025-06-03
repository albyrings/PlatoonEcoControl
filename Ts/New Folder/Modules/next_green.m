function t_next = next_green(light, t)
    if is_green(light,t), t_next=t; return; end
    cyc = mod(t - light.offset, light.cycle_time);
    if cyc < light.green_start
        t_next = t + (light.green_start - cyc);
    else
        t_next = t + (light.cycle_time - cyc) + light.green_start;
    end
end