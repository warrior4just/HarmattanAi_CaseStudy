clc;
clear;
fld = "C:\Users\SamChakour\Downloads\Harttaman AI\";
addpath(genpath(fld));

% Read the motion CSV file into a table
motion = readtable('motion.csv');
thrust = readtable('thrust.csv');
motor = readtable('motor.csv');
battery_discharge = readtable('battery_discharge.csv');
battery_flight = readtable('battery_flight.csv');

% Display the first 5 rows
%head(motion, 50);
%head(thrust,10);
%head(battery_discharge,5);
%head(battery_flight,5)

timestamp  = (motion.timestamp - motion.timestamp(1)) * 1e-6;
dt = diff(timestamp(1:2)); % 100 Hz

gyro_x = motion.gyro_rad_0_;
gyro_y = motion.gyro_rad_1_;
gyro_z = motion.gyro_rad_2_;

accel_x = motion.accelerometer_m_s2_0_;
accel_y = motion.accelerometer_m_s2_1_;
accel_z = motion.accelerometer_m_s2_2_;

figure(1);
subplot(3,2,1);
plot(timestamp,gyro_x);
xlabel("time (sec * 1e-6)");
ylabel("Roll-rate (p) (rad/s)");

subplot(3,2,3);
plot(timestamp,gyro_y);
xlabel("time (sec * 1e-6)");
ylabel("Pitch-rate (q) (rad/s)");

subplot(3,2,5);
plot(timestamp,gyro_z);
xlabel("time (sec * 1e-6)");
ylabel("angRate-z (r) (rad/s)");


subplot(3,2,2);
plot(timestamp,accel_x);
xlabel("time (sec * 1e-6)");
ylabel("Accel-x (m/s^2)");

subplot(3,2,4);
plot(timestamp,accel_y);
xlabel("time (sec * 1e-6)");
ylabel("Accel-y (m/s^2)");

subplot(3,2,6);
plot(timestamp,accel_z);
xlabel("time (sec * 1e-6)");
ylabel("Accel-z (m/s^2)")
sgtitle("Measured Gyro and Accelerometer log (motion.csv)")


% 
rpm_i = 1000:50:36000;
thrust_i = polyval(polyfit(thrust.RPM,thrust.Thrust_N,2), rpm_i);
figure(2);
subplot(2,1,1);
plot(thrust.RPM,thrust.Thrust_N);
hold on;
plot(rpm_i,thrust_i);
xlabel("RPM (rounds.m^{-1})");
ylabel("Thrust (N)");
legend(["Thrust","Thrust-fitted (regression)"]);
title(" Thrust vs RPM (thrust.csv)")

subplot(2,1,2);
plot(motor.RPM_sp);
hold on;
plot(motor.RPM_mes);
xlabel("Tick");
ylabel("RPM (round.m^{-1})");
legend(["RPM sp (rpm)","RPM mes (rpm)"]);
title(" RPM setpoint vs RPM measured (motor.csv)");



figure(3);
subplot(2,1,1);
plot(battery_discharge.time__s_,battery_discharge.voltage__V_)
xlabel("time (sec)");
ylabel("Voltage (V)");
title("Discharge voltage log from 'battery\_discharge.csv'")
subplot(2,1,2);
plot(battery_discharge.time__s_,battery_discharge.current__A_)
xlabel("time (sec)");
ylabel("Current (A)");
title("Discharge current log from 'battery\_discharge.csv'")

figure(4);
subplot(2,1,1);
plot(battery_flight.time__s_,battery_flight.voltage__V_)
xlabel("time (sec)");
ylabel("Voltage (V)");
title("Discharge voltage log from 'battery\_flight.csv'")
subplot(2,1,2);
plot(battery_flight.time__s_,battery_flight.current__A_)
xlabel("time (sec)");
ylabel("Current (A)");
title("Discharge Current log from 'battery\_flight.csv'")


