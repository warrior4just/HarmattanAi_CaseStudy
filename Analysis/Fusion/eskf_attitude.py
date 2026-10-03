import csv
from pathlib import Path

import numpy as np

import matplotlib
import matplotlib.pyplot as plt

#matplotlib.use("Agg")

gravity = 9.80665
gyro_cols = [f"gyro_rad[{axis}]" for axis in range(3)]
accel_cols = [f"accelerometer_m_s2[{axis}]" for axis in range(3)]



# Ontain skwew-symmetric matrix from a 3D vec
def skew(vector):
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


# Normalize a vec
def normalize(vector):
    norm = np.linalg.norm(vector)
    if norm < 1e-10:
        raise ValueError("Cannot normalize a zero-length vector")
    return vector/norm

# Normalze quaternion
def quaternion_normalize(quaternion):
    normed_q =quaternion/np.linalg.norm(quaternion)
    return normed_q

# Rotation via quaternion multiplication
def quaternion_multiply(left, right):
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array([
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ])


def quaternion_from_rotation_vector(rotation_vector):
    angle = np.linalg.norm(rotation_vector)
    if angle < 1e-10:
        return quaternion_normalize(np.r_[1.0, 0.5 * rotation_vector])
    return np.r_[np.cos(angle / 2.0),
                 np.sin(angle / 2.0) * rotation_vector / angle]


def quaternion_from_two_vectors(source, target):
    source = normalize(source)
    target = normalize(target)
    dot = np.clip(np.dot(source, target), -1.0, 1.0)
    if dot < -1.0 + 1e-10:
        axis = normalize(np.cross(source, [1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-8:
            axis = normalize(np.cross(source, [0.0, 1.0, 0.0]))
        return np.r_[0.0, axis]
    return quaternion_normalize(np.r_[1.0 + dot, np.cross(source, target)])

# Rotation from body to inertial frame (NED)
def rotation_matrix(quaternion):
    w, x, y, z = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])

# Obtian euler angles from quaternion
def euler_angles(quaternion):
    rotation = rotation_matrix(quaternion)
    roll = np.arctan2(rotation[2, 1], rotation[2, 2])
    pitch = np.arctan2(-rotation[2, 0], np.hypot(rotation[2, 1], rotation[2, 2]))
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    return np.array([roll, pitch, yaw])


def attitude_eskf_estimate(timestamp, gyro, accel, init_seconds=0.5, apply_gate=True):
    Nt = len(timestamp)
    init_mask = timestamp - timestamp[0] <= init_seconds
    init_count = int(np.count_nonzero(init_mask))
    if init_count < 2:
        raise ValueError("Issue with initialization. Window contains fewer than two samples")

    gyro_bias = np.mean(gyro[init_mask], axis=0)
    initial_accel = np.mean(accel[init_mask], axis=0)
    if np.linalg.norm(initial_accel) < 1e-6:
        raise ValueError("Initial accelerometer average is too small to initialize attitude")

    # The data stationary samples point approximately along negative body Z.
    gravity_ned = np.array([0.0, 0.0, -1.0])
    quaternion = quaternion_from_two_vectors(initial_accel, gravity_ned)
    integrated_gyro_quaternion = quaternion.copy()

    # Error state: attitude error (rad), gyro-bias error (rad/s)].
    covariance = np.diag([np.deg2rad(10.0) ** 2] * 3 + [0.03 ** 2] * 3)
    # Gyro measurement noise and process noise parameters. 
    gyro_noise_density = 0.015
    bias_random_walk = 0.0005
    
    # Uncertainty in the meaasured vector direction of acceleration (rad),
    # used to compute the measurement covariance R.
    accel_align_uncertainty_std = np.deg2rad(5.0)
    
    I_eye = np.eye(6)

    attitude_est = np.zeros((Nt, 3))
    integrated_gyro = np.zeros((Nt, 3))

    
    attitude_est[0] = euler_angles(quaternion)
    integrated_gyro[0] = euler_angles(integrated_gyro_quaternion)

    for index in range(1, Nt):
        dt = timestamp[index] - timestamp[index - 1]
        angular_rate = gyro[index] - gyro_bias

        quaternion = quaternion_normalize(quaternion_multiply(
            quaternion, quaternion_from_rotation_vector(angular_rate * dt)))
        
        gyro_gtruth = (gyro[index] - np.mean(gyro[init_mask], axis=0)) * dt
        integrated_gyro_quaternion = quaternion_normalize(
            quaternion_multiply(integrated_gyro_quaternion,
            quaternion_from_rotation_vector(gyro_gtruth)))

        transition = np.zeros((6, 6))
        transition[:3, :3] = np.eye(3) - skew(angular_rate) * dt
        transition[:3, 3:] = -np.eye(3) * dt
        transition[3:, 3:] = np.eye(3)
        process_noise = np.diag(
            [gyro_noise_density ** 2 * dt] * 3 + [bias_random_walk ** 2 * dt] * 3)
        covariance = transition @ covariance @ transition.T + process_noise

        accel_norm = np.linalg.norm(accel[index])
        
        measured_direction = accel[index] / accel_norm
        predicted_direction = rotation_matrix(quaternion).T @ gravity_ned
        
        # r
        innovation = measured_direction - predicted_direction
        
        # H
        measurement_jacobian = np.zeros((3, 6))
        measurement_jacobian[:, :3] = skew(predicted_direction)

        norm_error = abs(accel_norm - gravity)
        measurement_std = accel_align_uncertainty_std * (1.0 + norm_error / 0.5)
        # R
        measurement_covariance = np.eye(3) * measurement_std ** 2
        # S
        innovation_covariance = (
            measurement_jacobian @ covariance @ measurement_jacobian.T
            + measurement_covariance)
        kalman_gain = np.linalg.solve(
            innovation_covariance,
            measurement_jacobian @ covariance).T
        #nis = innovation @ np.linalg.solve(innovation_covariance, innovation)
        #if nis <= innovation_gate:
        correction = kalman_gain @ innovation
        quaternion = quaternion_normalize(quaternion_multiply(
            quaternion,
            quaternion_from_rotation_vector(correction[:3])))
        gyro_bias = gyro_bias + correction[3:]

        residual_map = I_eye - kalman_gain @ measurement_jacobian
        
        # P_joseph
        covariance = (
            residual_map @ covariance @ residual_map.T
            + kalman_gain @ measurement_covariance @ kalman_gain.T)
        reset_jacobian = I_eye.copy()
        reset_jacobian[:3, :3] -= 0.5 * skew(correction[:3])
        # P update
        covariance = reset_jacobian @ covariance @ reset_jacobian.T
        #accepted[index] = True

        # Stabilize covariance for itive semi-definiteness
        covariance = (1/2) * (covariance + covariance.T)
        attitude_est[index] = euler_angles(quaternion)
        integrated_gyro[index] = euler_angles(integrated_gyro_quaternion)

    return attitude_est, integrated_gyro, gyro_bias, init_count


def plot_estimation(timestamp, accel, attitude_est, integrated_gyro, output_path):
    time = timestamp - timestamp[0]

    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    fig.suptitle("Pitch and Roll Attitude Estimate (motion.csv)")
    for axis, state_index, title in [(axes[0], 0, "Roll"), (axes[1], 1, "Pitch")]:
        axis.plot(time, np.rad2deg(attitude_est[:, state_index]), label="ESKF attitude Estimate", linewidth=1.4)
        axis.plot(time, np.rad2deg(integrated_gyro[:, state_index]),
                  label="Integrated Gyro", linewidth=1.0, alpha=0.75)
        axis.set_ylabel(f"{title} (deg)")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right")
    axes[-1].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.show()
    plt.close(fig)


def find_project_root(script_dir):
    for candidate in (script_dir, *script_dir.parents):
        if (candidate / "Telemetry").is_dir():
            return candidate
    return script_dir.parent.parent



def load_csv(path):
    with path.open(newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        if not reader.fieldnames:
            raise ValueError(f"CSV has no header: {path}")
        rows = list(reader)

    required = ["timestamp", *gyro_cols, *accel_cols]
    missing = [column for column in required if column not in reader.fieldnames]
    if missing:
        raise ValueError(f"Missing required motion.csv columns: {', '.join(missing)}")

    try:
        timestamp = np.array([float(row["timestamp"]) for row in rows]) * 1e-6
        gyro = np.array([[float(row[column]) for column in gyro_cols] for row in rows])
        accel = np.array([[float(row[column]) for column in accel_cols] for row in rows])
    except (TypeError, ValueError) as error:
        raise ValueError("Check file, it may contain non-numeric values") from error

    valid = np.isfinite(timestamp) & np.all(np.isfinite(gyro), axis=1) & np.all(np.isfinite(accel), axis=1)
    timestamp, gyro, accel = timestamp[valid], gyro[valid], accel[valid]
    if len(timestamp) < 2:
        raise ValueError("Provide sufficient valid samples")
    if np.any(np.diff(timestamp) <= 0.0):
        raise ValueError("Timestamps monotonicity issue")
    return timestamp, gyro, accel





script_dir = Path(__file__).resolve().parent
project_root = find_project_root(script_dir)

csv_path = project_root / "Telemetry" / "motion.csv"
plot_path = script_dir / "eskf_attitude.png"
init_seconds = 0.5
apply_gate = True

# Load the raw measurements
timestamp, gyro, accel = load_csv(csv_path)

# Run the ESKF attitude estimation
attitude_est, integrated_gyro, gyro_bias, init_count = attitude_eskf_estimate(
    timestamp, gyro, accel, init_seconds, apply_gate)
plot_path.parent.mkdir(parents=True, exist_ok=True)

# We plot the result of the estimation
plot_estimation(timestamp, accel, attitude_est, integrated_gyro, plot_path)

dt = np.diff(timestamp)
print(f"Loaded {len(timestamp)} samples over {timestamp[-1] - timestamp[0]:.2f} s")

print(f"Median sample interval: {np.median(dt) * 1e3:.2f} ms")
print(f"Initialization samples: {init_count}")
print(f"Estimated gyro bias (rad/s): {gyro_bias}")
print(f"Final roll/pitch/yaw (deg): {np.rad2deg(attitude_est[-1])}")


