
function ocv = ocv_discharge(voltage, current, dt, r0, r1, c1)
    %  Construct OCV from a discharging battery data.
    % 
    % Inputs:
    %   voltage    - Vector of measured terminal voltage (V)
    %   current - Vector of discharge current (A) -> assumed positive (>0)
    %   dt      - Sampling time interval (seconds)
    %   r0      - Ohmic resistance (Ohms)
    %   r1      - Polarization resistance (Ohms)
    %   c1      - Polarization capacitance (Farads)

    num_samples = length(voltage);
    v1 = zeros(num_samples, 1);
    ocv = zeros(num_samples, 1);
    
    % Pre-calculate exponential decay coefficients for performance
    alpha = exp(-dt / (r1 * c1));
    beta = r1 * (1 - alpha);    

    ocv(1) = voltage(1) + ( current(1) * r0);
    % Time-stepping loop to track the internal capacitor voltage (V1)
    for k = 2:num_samples
        if current(k) <= 1e-3
            v1(k) = v1(k-1);
            ocv(k) = voltage(k) + v1(k);
        else
            v1(k) = v1(k-1) * alpha + current(k) * beta;            
            ocv(k) = voltage(k) + (current(k) * r0) + v1(k);
        end
    end
    
end


