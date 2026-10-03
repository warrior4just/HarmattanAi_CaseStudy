
%% 1- Hovering RPM

% Conversion
rpm_2_rad_s = 2*pi/60;

% Config
num_motors = 4; % Number of motors
k_rad_s = 1.255e-6; % Assumed Thrust ocefficnett (N/(rad/s)^2)
Length = 144 * 1e-3 ; % Distance between two motors in X configuration
Lx = Length/2 ; % m
Ly = Lx ; % m
Width = 44 * 1e-3 ; %m

% Environment
g_acc = 9.81; % m.s^-2 
% Mass properties:
motor_mass_kg = 35 * 1e-3 ; % kg
body_mass_kg = 1000 * 1e-3 ; % kg
total_mass_kg = num_motors * motor_mass_kg + body_mass_kg;


J_motors= 4 * (motor_mass_kg * Ly^2); % Moment of inertia of motors
J_body = (1/12) * body_mass_kg * Width^2;
Jx = J_body + J_motors; % kg.m^2
Jy = Jx; % kg.m^2


% Total forces equilibrium
Thrust_prop = 2.795; % N
weight = total_mass_kg * g_acc; % N
hover_rpm = 14250;

w0 = hover_rpm * rpm_2_rad_s ; % rad/s

%% - MISO models
% Pitch MISO
% Lumped gains
G_pitch = (2 * Lx * k_rad_s * w0) / Jy; 
G_roll  = (2 * Ly * k_rad_s * w0) / Jx;

% Continuous-Time Pitch State-Space Formulation
A_pitch = [0 1; 
           0 0];
B_pitch = [   0,        0,        0,        0;
           G_pitch,  G_pitch, -G_pitch, -G_pitch];

C_pitch = [1 0];
D_pitch = zeros(1, 4);


sys_miso_pitch = ss(A_pitch, B_pitch, C_pitch, D_pitch, ...
    'InputName', {'u1_FL','u2_FR','u3_RR','u4_RL'}, ...
    'OutputName', 'Pitch_Angle');


% Roll MISO
% Continuous-Time Roll State-Space Formulation
A_roll = [0 1; 
          0 0];
      
B_roll = [   0,        0,        0,        0;
          G_roll,  -G_roll,  -G_roll,   G_roll];
      
C_roll = [1 0];
D_roll = zeros(1, 4);

sys_miso_roll = ss(A_roll, B_roll, C_roll, D_roll, ...
    'InputName', {'u1_FL','u2_FR','u3_RR','u4_RL'}, ...
    'OutputName', 'Roll_Angle');


%% Optional: Discretize model 
fs = 500; % Loop rate 
Ts = 1/fs;

sys_pitch_d = c2d(sys_pitch_c, Ts, 'tustin');
sys_roll_d  = c2d(sys_roll_c, Ts, 'tustin');

% Matrix Display Outputs
fprintf('=== DISCRETE PITCH MISO MATRICES (Tustin @ %d Hz) ===\n', fs);
disp('Discrete A_d Matrix:'); disp(sys_pitch_d.A);
disp('Discrete B_d Matrix:'); disp(sys_pitch_d.B);
disp('Discrete C_d Matrix:'); disp(sys_pitch_d.C);
disp('Discrete D_d Matrix:'); disp(sys_pitch_d.D);

