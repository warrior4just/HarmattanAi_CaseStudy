function capacity_mAh = calculate_battery_capacity(time_sec, current_amp)
    current_amp = abs(current_amp(:)); %     
    % Using the trapezoidal rule to compute the integral
    amp_seconds = trapz(time_sec, current_amp);    
    capacity_mAh = 1000 * (amp_seconds / 3600);    
    fprintf('Calculated Battery Capacity: %.1f mAh \n', capacity_mAh);
end