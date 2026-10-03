clear;
close all;
fld = "C:\Users\SamChakour\Downloads\Harttaman AI\";
addpath(genpath(fld));
run explore.m


% Battery 6S1P
% Let R0 = 25 mOhm
battery1.param.R0 = 25 * 1e-3; % Ohm
battery1.param.R1 = battery1.param.R0/2; % Ohm
battery1.param.tau = 111; % sec
battery1.param.C1 = battery1.param.tau/(battery1.param.R1);
battery1.param.dt = 1 ;% sec
battery1.param.b1_num_cells = 6; % Serial topology 
battery1.param.Q_6S1P = 1760 * 1e-3  ; % Ah

% Get Voltage of a cell and the current (Serial)
b1_cell_voltage_V = battery_discharge.voltage__V_/battery1.param.b1_num_cells; % Serial, voltage drop
b1_cell_current_A = battery_discharge.current__A_; % Serial, same currect


% Estimate the Ocv via Kirchoff law using the Tevenin model of a cell
ocv_estimated_b1 = ocv_discharge(b1_cell_voltage_V, b1_cell_current_A, ...
    battery1.param.dt, battery1.param.R0, battery1.param.R1, battery1.param.C1);

% Estimate the SoC from Coulomb Counting (rudimentary)
soc_cc_b1 = soc_discharge(b1_cell_current_A,battery1.param.Q_6S1P,battery1.param.dt);
write_lut('ocv_soc_lut.bin',soc_cc_b1, ocv_estimated_b1)
poly_order = 7;
coefficients = polyfit(soc_cc_b1, ocv_estimated_b1, poly_order);
% Verify the fit by calculating the fitted curve
fitted_ocv = polyval(coefficients, soc_cc_b1);


figure(9);
plot(soc_cc_b1 * 100, ocv_estimated_b1);
hold on;
plot(soc_cc_b1 * 100 ,fitted_ocv);
xlabel("Soc (%)");
ylabel("OCV (V)"); 
title('The OCV(SoC) curve');
legend(["Estimated OCV(SoC)" "7th-order polynomial fitted OCV(SoC)"])
grid on;


figure(10);
subplot(2,1,1);
plot(battery_discharge.time__s_,ocv_estimated_b1);
xlabel("Time (sec)");
ylabel("OCV (V)");
title("The OCV(SoC) curve from 'battery\_discharge.csv' ");
grid on;
subplot(2,1,2);
plot(battery_discharge.time__s_,soc_cc_b1);
xlabel("Time (sec)");
ylabel("SoC (%)");
title('State of Charge % (Coulomb Counting)');
grid on;
%% Adapting to new topology
% Battery 4S1P
% Let R0 = 25 mOhm
battery2.param.R0 = 25 * 1e-3; % Ohm
battery2.param.R1 = battery2.param.R0/2; % Ohm
battery2.param.tau = 2.4; % sec, at 3.12+0.632*(3.319-3.12) = 3.2458 
battery2.param.C1 = battery2.param.tau/(battery2.param.R1);
battery2.param.dt = 1 ;% sec
battery2.param.b2_num_cells = 4; % Serial topology 

% Estimate Battery Capacity 
battery2.param.Q_4S1P =  1e-3 * calculate_battery_capacity(battery_flight.time__s_, battery_flight.current__A_);

% Get Voltage of a cell and the current (Serial)
b2_cell_voltage_V = battery_flight.voltage__V_/battery2.param.b2_num_cells; % Serial, voltage drop
b2_cell_current_A = battery_flight.current__A_; % Serial, same currect

% Battery 2: Estimate the Ocv via Kirchoff law using the Tevenin model of a cell
ocv_estimated_b2 =  ocv_discharge(b2_cell_voltage_V,b2_cell_current_A, ...
    battery2.param.dt, battery2.param.R0, battery2.param.R1, battery2.param.C1);

% Estimate the SoC from Coulomb Counting (rudimentary)
soc_cc_b2 = 100 * soc_discharge(b2_cell_current_A,battery2.param.Q_4S1P,battery2.param.dt);

figure(11);
plot(soc_cc_b2,ocv_estimated_b2);
xlabel("Soc (%)");
ylabel("OCV (V)");
title("The OCV(SoC) curve from 'battery\_flight.csv' ");
grid on;


figure(12);
subplot(2,1,1);
plot(battery_flight.time__s_,ocv_estimated_b2);
xlabel("Time (sec)");
ylabel("OCV (V)");
title('OCV plot');
grid on;
subplot(2,1,2);
plot(battery_flight.time__s_,soc_cc_b2);
xlabel("Time (sec)");
ylabel("SoC (%)");
title('State of Charge % (Coulomb Counting)');
grid on;

%% EKF 
% Covariance parameters tuning matrices
battery2.param.Q_cov = [100, 0; 
                        0,    1];
battery2.param.R_cov = 0.01;     % Expecting a standard ~10mV sensor noise deviation
battery2.param.soc_init = 0.85;  % WRONG initial guess to test filter convergence (True starting SoC is 95%)

chargeState = -1; % 1 : Charge , -1 :  discharge

%% Execute the Kalman Filter Function
[soc_estimated, v1_estimated, P_hist] = ekf_soc_estimator(battery_flight.time__s_, ...
    chargeState * battery_flight.current__A_, ocv_estimated_b2, battery2.param);

%% Plot Evaluation
figure('Color', [1 1 1]);

subplot(2,1,1);
plot(battery_flight.time__s_, soc_cc_b2, 'k-', 'LineWidth', 2.5); 
hold on;
plot(battery_flight.time__s_, soc_estimated * 100 , 'r--', 'LineWidth', 1);
ylabel('State of Charge (%)'); grid on;
legend('True SoC', 'EKF Estimated SoC', 'Location', 'best');
title('Extended Kalman Filter Performance');

subplot(2,1,2);
%plot(battery_flight.time__s_, ocv_estimated_b2, 'Color', [0.7 0.7 0.7]); 
%hold on;
plot(soc_estimated, ocv_estimated_b2, 'b-', 'LineWidth', 1.5);
ylabel('OCV (V)'); xlabel('Time (seconds)'); grid on;
%legend('Noisy Measured Pin Reading', 'Ideal Analytical Voltage', 'Location', 'best');

