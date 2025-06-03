function trigger_events = velocity_trigger(t, diff_v, opt_v)
    % Threshold per scatenare un “trigger”
    trigger_threshold = 5;
    % Intervallo di disabilitazione dopo un “evento”
    disable_duration  = 10;
    % Soglia di variazione rapida nell'ottimo
    rapid_change_thresh = 0.5;
    % Ritardo iniziale prima di abilitare i trigger
    initial_delay = 5;
    
    trigger_events = zeros(size(t));
    disable_until = -inf;
    
    for i = 1:length(t)
        if t(i) < initial_delay || t(i) < disable_until
            trigger_events(i) = 0;
        else
            if abs(diff_v(i)) > trigger_threshold
                trigger_events(i) = 1;
            else
                trigger_events(i) = 0;
            end
        end

        % Se la velocità obiettivo cambia rapidamente, disabilita nuovi eventi
        if i > 1
            delta_opt = abs(opt_v(i) - opt_v(i-1));
            if delta_opt > rapid_change_thresh
                disable_until = t(i) + disable_duration;
            end
        end
    end
end