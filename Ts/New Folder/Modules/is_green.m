function st = is_green(light, time)
    t_in_cycle = mod(time - light.offset, light.cycle_time);
    if light.green_start <= light.green_end
        st = (t_in_cycle >= light.green_start && t_in_cycle < light.green_end);
    else
        st = (t_in_cycle >= light.green_start || t_in_cycle < light.green_end);
    end
end