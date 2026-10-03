function soc = soc_discharge(battery_current,Q_nom,dt)

charge = false;
discharge_current = battery_current;
if ~charge
    discharge_current = - battery_current;
end

num_samples = length(discharge_current);
soc = zeros(num_samples, 1);
soc(1) = 0.999;

for k=2:num_samples
    soc(k) = soc(k-1) + (discharge_current(k) * dt) / (Q_nom * 3600);
    soc(k) = max(0, min(1, soc(k))); % Bound check
end

end