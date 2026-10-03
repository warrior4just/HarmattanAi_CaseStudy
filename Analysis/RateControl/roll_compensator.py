# %%
import numpy as np
import control as ct

import matplotlib.pyplot as plt

# Complex frequency variable for Laplace transforms
s = ct.tf('s')

# Conversion
rpm_2_rad_s = 2 * np.pi / 60

# Sampling time for discrete-time systems
fs = 1000.0  # Hz
Ts = 1.0/fs  # seconds

# Config
num_motors = 4  # Number of motors
K_thrust = 1.255e-6  # Assumed Thrust coefficient (N/(rad/s)^2)
Len_x = 140 * 1e-3  # Distance between two motors in X configuration
Len_y = 180 * 1e-3  # Distance between two motors in Y configuration
Lx = Len_x / 2  # m
Ly = Len_y / 2  # m
Width = 40 * 1e-3  # m
Length = 80 * 1e-3 # m

# Motor time constant
# Assumed,since the provided plots of motor do not provide time stamp
# if sample time is 1 ms, the motors ramp to ss in 50 ticks or about 50 ms
# which is slow response for drone motor. So we assume 50 ms baseline
T_motor = 0.05  # Motor time constant (s). 


# Environment
g_acc = 9.81  # m.s^-2
# Mass properties:
motor_mass_kg = 35 * 1e-3  # kg
body_mass_kg = 1000 * 1e-3  # kg
total_mass_kg = num_motors * motor_mass_kg + body_mass_kg

Jxx_motors = 4 * (motor_mass_kg * Ly**2)  # Moment of inertia of motors
Jyy_motors = 4 * (motor_mass_kg * Lx**2)  # Moment of inertia of motors
Jxx_body = (1 / 12) * body_mass_kg * Width**2
Jyy_body = (1 / 12) * body_mass_kg * Length**2
Jxx = Jxx_body + Jxx_motors  # kg.m^2
Jyy = Jyy_body + Jyy_motors  # kg.m^2
print(f" Jxx: {Jxx}")
print(f" Jyy: {Jyy}")

# %%
# Total forces equilibrium
Thrust_hover = 2.795  # N
weight = total_mass_kg * g_acc  # N
hover_rpm = 14250 # Obtained from the plot

w0 = hover_rpm * rpm_2_rad_s  # rad/s


# Roll MISO
# Lumped gain
K_plant_roll = (2 * Ly * K_thrust * w0) / Jxx


# Obtaining the motor transfer functions
tf_motor_c = ct.tf([1],[T_motor, 1])
print("Motor Transfer Function\n")
print(tf_motor_c)

# Buit the roll SISO transfer function
sys_roll_siso_c = ct.tf([K_plant_roll],[1, 0])
print("Roll SISO Transfer Function\n")


# Build uncompensated plant 
tf_roll_c = tf_motor_c * sys_roll_siso_c
print("Plant Transfer Function-1\n")
print(tf_roll_c)    


G_plant_roll  = K_plant_roll/(s * (T_motor * s + 1))
print("Plant Transfer Function-2 \n")
print(G_plant_roll)


# Mixer Matrix 
# Our layout : 1:FL(CW), 2:FR(CCW), 3:RR(CW), 4:RL(CCW)
Mixer = np.array([
    [ 1 ,  1],  # Motor 1 (FL)
    [-1 ,  1],  # Motor 2 (FR)
    [-1 , -1],  # Motor 3 (RR)
    [ 1 , -1]   # Motor 4 (RL)
])

print("Actuator Mixer Matrix (Virtual Inputs -> Physical Motors):")
print(Mixer)



# %%
## Nyquist plot 
# Roll
ct.nyquist_plot(G_plant_roll, omega=np.logspace(-2, 3, 1000), primary_style='-')

# Polish the Plot Visuals
plt.title('Open-Loop Nyquist Plot - Roll System', fontsize=12, fontweight='bold')
plt.xlabel('Real Axis')
plt.ylabel('Imaginary Axis')
plt.grid(True, which='both', linestyle='--', alpha=0.5)
plt.axhline(0, color='black', linewidth=1)
plt.axvline(0, color='black', linewidth=1)

# # Highlight the critical stability point (-1, 0)
# plt.plot(-1, 0, 'ro', label='Critical Point (-1, 0j)', markersize=8)
# plt.legend()
# plt.show()


# Analysis of Plant and Open-loop transfer function
# Obtaining the poles of the roll SISO transfer functions
poles_roll_plant  = ct.poles(G_plant_roll)
gm_roll, pm_roll, wcg_roll, wcp_roll = ct.margin(G_plant_roll)
print(f"Roll SISO Poles: {poles_roll_plant}")
print(f"Gain Margin (GM):     {gm_roll:.2f} (or {20*np.log10(gm_roll):.2f} dB) at {wcg_roll:.2f} rad/s")
print(f"Phase Margin (PM):    {pm_roll:.2f}° at {wcp_roll:.2f} rad/s")

ct.bode_plot(G_plant_roll, dB=True, Hz=False, margins=True, grid=True)
plt.gcf().suptitle("Plant Bode Diagram: Stability & Margin Profile", fontsize=14, fontweight='bold')
plt.show()

# %%
# Controller Design
# Lead-Lag Compensator Design for Roll
# Lag compensator introduces a low-frequency gain to improve steady-state error but introduces also phase lag (reduces stability margins), which can reduce stability margins.
# while the lead compensator adds phase lead to improve transient response and stability margins.
# Since this is type-1 system, it already tracks step inputs with zero steady-state error. However, it may have a steady-state error for ramp inputs. To improve the steady-state error for ramp inputs, we can add a lag compensator to increase the low-frequency gain.
# As initial compensator, lead compensator can be used to improve the transient response and stability margins. It adds phase lead to the system, which can help to counteract the phase lag introduced by the lag compensator.
nticks_rise = 80
# ticks for rise time
nticks_settle = 220 # ticks for settling time

# Specifications for the compensator design
e_ss = 0.05 # Steady state error
t_rise = nticks_rise * Ts # Rise time (s)
t_settle = nticks_settle * Ts # Settling time (s)

safety = 5.0       # Safety buffer for phase lead boost (degrees)
wc_buffer_factor = 1.5 # factor for wc


wc = 2.2/t_rise    # Target open-loop crossover frequency (rad/s)
wc = wc_buffer_factor * wc # Target open-loop crossover frequency (rad/s)
PM_target = 60.0   # Desired closed-loop Phase Margin (degrees)
GM_target = 10.0   # Desired closed-loop Gain Margin (dB)


# Continuous plant phase equation: 
phase_plant_continuous_deg = -90.0 - np.degrees(np.arctan(wc * T_motor))

# A delay introduced by the later discretization e.g. zoh
# this is an additional phase drop of : -wc * (Ts / 2) in radians, mapped to degrees
phase_zoh_deg = -np.degrees(wc * (Ts / 2.0))

# Combined uncompensated phase margin relative to the -180° stability boundary
PM_uncomp = 180.0 + (phase_plant_continuous_deg + phase_zoh_deg)

# Required phase boost
phi_m_deg = PM_target - PM_uncomp + safety
phi_m_rad = np.radians(phi_m_deg)

# Lead attenuation alpha scaling parameter
alpha = (1.0 - np.sin(phi_m_rad)) / (1.0 + np.sin(phi_m_rad))
# Symmetrical frequency separation around our chosen crossover point (wc)
z = wc * np.sqrt(alpha) # zero placement
p = wc / np.sqrt(alpha) # pole placement

# Magnitude of the unscaled physical plant at crossover frequency
mag_plant = K_plant_roll / (wc * np.sqrt((wc * T_motor)**2 + 1.0))
mag_array_plant, _, _ = ct.freqresp(G_plant_roll, [wc])
mag_plant = float(mag_array_plant.squeeze())

# Built the unscaled lead compensator transfer function
G_lead_unsc = ((s / z) + 1) / ((s / p) + 1)

# Extract raw lead template magnitude at wc
mag_array_lead, _, _ = ct.freqresp(G_lead_unsc, [wc])
mag_lead = float(mag_array_lead.squeeze())

# Apply Magnitude Condition: |G_lead(j_wc) * G(j_wc)| = 1
Kc = 1.0 / (mag_lead * mag_plant)


# Evaluate total loop magnitude at your target wc frequency 
_ , mag, _ = ct.freqresp(G_lead_unsc * G_plant_roll, [wc])
loop_magnitude = mag[0]

# Controller transfer function with gain applied
G_lead = Kc * G_lead_unsc
print("Lead Compensator Transfer Function:")
print(G_lead)


# Analysis of the open loop transfer function with the lead compensator
L_roll_rate = Kc * G_lead_unsc * G_plant_roll
print("Open-loop transfer function with lead compensator (Roll):")
print(L_roll_rate)



# Open-loop transfer function GM and PM
gm_roll, pm_roll, wcg_roll, wcp_roll = ct.margin(L_roll_rate)
print(f"Gain Margin (GM):     {gm_roll:.2f} (or {20*np.log10(gm_roll):.2f} dB) at {wcg_roll:.2f} rad/s")
print(f"Phase Margin (PM):    {pm_roll:.2f}° at {wcp_roll:.2f} rad/s")
print("=" * 60 + "\n")

# Bode plots
ct.bode_plot(L_roll_rate, dB=True, Hz=False, margins=True, grid=True)
plt.gcf().suptitle("Open-Loop Bode Diagram: Stability & Margin Profile", fontsize=14, fontweight='bold')
plt.show()


# %%
# System Analysis: Time-Domain Responses
t = np.linspace(0, 2, 5000)

# Closed-loop transfer functions: T(s) = L(s) / (1 + L(s))
T_roll = ct.feedback(L_roll_rate, 1)
Open_Loop_response = ct.feedback(G_plant_roll, 1)

# Step Response (Sudden stick movement to a fixed tracking rate)
t_step, y_step_roll = ct.step_response(T_roll, t)
t_ol_step, y_ol_step = ct.step_response(Open_Loop_response, t)

# Impulse Response (Sudden structural shock, like hitting a wind gust)
t_imp, y_imp_roll = ct.impulse_response(T_roll, t)
t_ol_imp, y_ol_imp = ct.impulse_response(Open_Loop_response, t)

# Ramp Response (Smooth, constantly accelerating tracking rate command)
G_ramp_integrator = 1 / s
t_ramp, y_ramp_roll = ct.step_response(T_roll * G_ramp_integrator, t)
t_ol_ramp, y_ol_ramp = ct.step_response(Open_Loop_response * G_ramp_integrator, t)

# =====================================================================
# Plotting the time-domain responses
fig, axes = plt.subplots(1, 3, figsize=(18, 5))
fig.suptitle("Quadcopter Rate Loop Transient Performance (500 Hz Update Rate)", fontsize=14, fontweight='bold')

# Panel 1: Step Response
axes[0].plot(t_step, np.ones_like(t_step), 'r--', label='Reference Input')
axes[0].plot(t_step, y_step_roll, 'b-', linewidth=2, label='Roll Rate Response')
axes[0].plot(t_ol_step, y_ol_step, 'k:', linewidth=2, label='Open-Loop Response')
axes[0].set_title('Step Response (Tracking Target Rate)')
axes[0].set_xlabel('Time (seconds)')
axes[0].set_ylabel('Angular Rate (rad/s)')
axes[0].grid(True)
axes[0].legend()

# Panel 2: Impulse Response
axes[1].plot(t_imp, np.zeros_like(t_imp), 'r--', label='Equilibrium')
axes[1].plot(t_imp, y_imp_roll, 'b-', linewidth=2, label='Roll Rate Response')
axes[1].plot(t_ol_imp, y_ol_imp, 'k:', linewidth=2, label='Open-Loop Response')
axes[1].set_title('Impulse Response (Wind Gust Rejection)')
axes[1].set_xlabel('Time (seconds)')
axes[1].set_ylabel('Angular Velocity Variance')
axes[1].grid(True)
axes[1].legend()

# Panel 3: Ramp Response
axes[2].plot(t_ramp, t_ramp, 'r--', label='Reference Ramp')
axes[2].plot(t_ramp, y_ramp_roll, 'b-', linewidth=2, label='Roll Rate Response')
axes[2].plot(t_ol_ramp, y_ol_ramp, 'k:', linewidth=2, label='Open-Loop Response')
axes[2].set_title('Ramp Response (Smooth Acceleration Lag)')
axes[2].set_xlabel('Time (seconds)')
axes[2].set_ylabel('Angular Displacement Reference')
axes[2].grid(True)
axes[2].legend()

plt.tight_layout()
plt.show()

# Analyze performance from time-domain responses
# =====================================================================
print("=================== Transient Response Verification ===================")
# Evaluate Roll Step Characteristics
rise_time_idx = np.where(y_step_roll >= 0.90)[0][0]
t_rise_final = t_step[rise_time_idx]
ess_step = abs(1.0 - y_step_roll[-1]) * 100

print(f"Roll Rise Time (to 90%): {t_rise_final*1000:.1f} ms  (Target: < {t_rise * 1e3} ms)")
print(f"Roll Steady-State Step Error: {ess_step:.2f}%     (Target: < 5%)")

# Evaluate Ramp Tracking Stability (Steady state tracking offset)
ramp_error_roll = t[-1] - y_ramp_roll[-1]
print(f"Roll Velocity Lag Error (Ramp):  {ramp_error_roll:.4f} rad/s")
print("=======================================================================")

