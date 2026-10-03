# %%
import csv
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# Gets the absolute global path of the folder containing THIS running file
parent_dir = Path(__file__).resolve().parent

# MATLAB-like addpath(genpath(folder)) behavior: add parent project folders to Python path
for candidate in [parent_dir, *parent_dir.parents]:
    if candidate.exists():
        sys.path.append(str(candidate))

# Look for the Telemetry folder near the script, like MATLAB's project path layout
project_root = None
for candidate in [parent_dir, *parent_dir.parents]:
    if (candidate / 'Telemetry').exists():
        project_root = candidate
        break

if project_root is None:
    project_root = parent_dir

data_dir = project_root / 'Telemetry'

file_set = [
    'motion.csv',
    'thrust.csv',
    'motor.csv',
    'battery_discharge.csv',
    'battery_flight.csv',
]

for name in file_set:
    if not (data_dir / name).exists():
        raise FileNotFoundError(f"Missing file: {data_dir / name}")


def load_csv_table(csv_path):
    with open(csv_path, 'r', newline='', encoding='utf-8') as f:
        rows = [row for row in csv.reader(f) if row and any(cell.strip() for cell in row)]

    if not rows:
        raise ValueError(f"CSV file is empty: {csv_path}")

    # Exported files sometimes include an initial index column.
    first_row = rows[0]
    if first_row and first_row[0].strip() in ('', 'Unnamed: 0', 'index', 'Index'):
        rows = [row[1:] for row in rows]

    header = rows[0]
    table = {name: [] for name in header}

    for row in rows[1:]:
        for key, value in zip(header, row):
            table[key].append(value)

    for key in table:
        try:
            table[key] = np.asarray(table[key], dtype=float)
        except ValueError:
            table[key] = np.asarray(table[key], dtype=str)

    return table


# Read the files in the Telemetry folder
motion = load_csv_table(data_dir / 'motion.csv')
thrust = load_csv_table(data_dir / 'thrust.csv')
motor = load_csv_table(data_dir / 'motor.csv')
battery_discharge = load_csv_table(data_dir / 'battery_discharge.csv')
battery_flight = load_csv_table(data_dir / 'battery_flight.csv')

# Display the first few rows (equivalent to head in MATLAB)
# print({k: v[:5] for k, v in motion.items()})
# print({k: v[:5] for k, v in thrust.items()})
# print({k: v[:5] for k, v in battery_discharge.items()})

# Process timestamps and dt
timestamp = (motion['timestamp'] - motion['timestamp'][0]) * 1e-6
dt = np.diff(timestamp[:2])[0]  # 100 Hz

# Extract sensor columns using the actual CSV headers from the Telemetry data files
gyro_x = motion['gyro_rad[0]']
gyro_y = motion['gyro_rad[1]']
gyro_z = motion['gyro_rad[2]']

accel_x = motion['accelerometer_m_s2[0]']
accel_y = motion['accelerometer_m_s2[1]']
accel_z = motion['accelerometer_m_s2[2]']

# -------------------------------------------------------------
# Figure 1: IMU Data
# -------------------------------------------------------------
fig1, axs1 = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
fig1.suptitle('Measured Gyro and Accelerometer log (motion.csv)')

# Gyro X
axs1[0, 0].plot(timestamp, gyro_x)
axs1[0, 0].set_ylabel('angRate-x (p) (rad/s)')

# Gyro Y
axs1[0, 1].plot(timestamp, gyro_y)
axs1[0, 1].set_ylabel('angRate-y (q) (rad/s)')

# Gyro Z
axs1[1, 0].plot(timestamp, gyro_z)
axs1[1, 0].set_ylabel('angRate-z (r) (rad/s)')

# Accel X
axs1[1, 1].plot(timestamp, accel_x)
axs1[1, 1].set_ylabel('Accel-x (m/s2)')

# Accel Y
axs1[2, 0].plot(timestamp, accel_y)
axs1[2, 0].set_ylabel('Accel-y (m/s2)')
axs1[2, 0].set_xlabel('time (sec * 1e-6)')

# Accel Z
axs1[2, 1].plot(timestamp, accel_z)
axs1[2, 1].set_ylabel('Accel-z (m/s2)')
axs1[2, 1].set_xlabel('time (sec * 1e-6)')

plt.tight_layout()

# -------------------------------------------------------------
# Figure 2: Thrust & Motor RPM
# -------------------------------------------------------------
fig2, axs2 = plt.subplots(2, 1, figsize=(10, 8))

# Polynomial fitting (2nd order regression)
rpm_i = np.arange(1000, 36100, 100)
coefs = np.polyfit(thrust['RPM'], thrust['Thrust_N'], 2)
thrust_i = np.polyval(coefs, rpm_i)

# Subplot 1: Thrust vs RPM
axs2[0].plot(thrust['RPM'], thrust['Thrust_N'], label='Thrust')
axs2[0].plot(rpm_i, thrust_i, label='Thrust-fitted (regression)')
axs2[0].set_xlabel('RPM (rounds.m$^{-1}$)')
axs2[0].set_ylabel('Thrust (N)')
axs2[0].legend()
axs2[0].set_title('Thrust vs RPM (thrust.csv)')

# Subplot 2: Motor demands
axs2[1].plot(motor['RPM_sp'], label='RPM sp (rpm)')
axs2[1].plot(motor['RPM_mes'], label='RPM mes (rpm)')
axs2[1].set_xlabel('Tick')
axs2[1].set_ylabel('RPM (round.m$^{-1}$)')
axs2[1].legend()
axs2[1].set_title('Thrust and RPM demands (motor.csv)')

plt.tight_layout()

# -------------------------------------------------------------
# Figure 3: Battery Discharge Log
# -------------------------------------------------------------
fig3, axs3 = plt.subplots(2, 1, figsize=(10, 8))

axs3[0].plot(battery_discharge['time_[s]'], battery_discharge['voltage_[V]'])
axs3[0].set_ylabel('Voltage (V)')
axs3[0].set_title("Discharge voltage log from 'battery_discharge.csv'")

axs3[1].plot(battery_discharge['time_[s]'], battery_discharge['current_[A]'])
axs3[1].set_xlabel('time (sec)')
axs3[1].set_ylabel('Current (A)')
axs3[1].set_title("Discharge current log from 'battery_discharge.csv'")

plt.tight_layout()

# -------------------------------------------------------------
# Figure 4: Battery Flight Log
# -------------------------------------------------------------
fig4, axs4 = plt.subplots(2, 1, figsize=(10, 8))

axs4[0].plot(battery_flight['time_[s]'], battery_flight['voltage_[V]'])
axs4[0].set_ylabel('Voltage (V)')
axs4[0].set_title("Discharge voltage log from 'battery_flight.csv'")

axs4[1].plot(battery_flight['time_[s]'], battery_flight['current_[A]'])
axs4[1].set_xlabel('time (sec)')
axs4[1].set_ylabel('Current (A)')
axs4[1].set_title("Discharge Current log from 'battery_flight.csv'")

plt.tight_layout()

# Render all windows
plt.show()

# %%
