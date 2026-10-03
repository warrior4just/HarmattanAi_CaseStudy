"""Estimate battery SOC from terminal voltage and current using an EKF."""

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

#plt.switch_backend("Agg")


class Config:
    """Configuration for the battery SOC EKF estimator."""

    def __init__(
        self,
        csv_path,
        
        #capacity_ah=1760 * 1e-3, # Ah 
        capacity_ah= 10000 * 1e-3, # Ah 
        series_cells= 4,
        #series_cells= 6,
        
        r0_ohm=0.025, # Ohm
        r1_ohm=0.0125, # Ohm
        tau_s=111.0, # sec
        soc_initial=0.999, 
        
        output_path=None,
        
        process_covariance = np.diag([1e-4, 1e-7]),
        measurement_variance = 1.96e-6,
    ):
        self.csv_path = Path(csv_path)
        
        # Battery params
        self.capacity_ah = capacity_ah
        self.series_cells = series_cells
        self.r0_ohm = r0_ohm
        self.r1_ohm = r1_ohm
        self.tau_s = tau_s
        self.soc_initial = soc_initial
        
        self.output_path = Path(output_path) if output_path is not None else None
        
        self.process_covariance = process_covariance
        self.measurement_variance = measurement_variance

# The coefficients of the OCV polynomial with respect to SOC, used in the EKF measurement model.
ocv_coefficients = np.array([156.9860, -599.6490,  924.6785, -738.4316,  326.2501,  -78.4950,    9.6148,    3.2842])
# The derivative coefficients of the OCV polynomial with respect to SOC, used in the EKF measurement Jacobian. 
docv_coefficients = np.polyder(ocv_coefficients)


def estimate_soc(
    time_s,
    current_a,
    terminal_voltage_v,
    capacity_ah,
    r0_ohm,
    r1_ohm,
    c1_f,
    soc_initial=0.999,
    process_covariance=None,
    measurement_variance=1e-4,
):
    """Return SOC and RC polarization voltage; positive current means charging."""
    time_s = np.asarray(time_s, dtype=float)
    current_a = np.asarray(current_a, dtype=float)
    terminal_voltage_v = np.asarray(terminal_voltage_v, dtype=float)

    # if time_s.ndim != 1 or current_a.ndim != 1 or terminal_voltage_v.ndim != 1:
    #     raise ValueError("time, current, and voltage must be one-dimensional")
    # if not (len(time_s) == len(current_a) == len(terminal_voltage_v)):
    #     raise ValueError("time, current, and voltage must have equal lengths")
    
    # if len(time_s) == 0:
    #     raise ValueError("the input data is empty")
    
    # if len(time_s) > 1 and np.any(np.diff(time_s) <= 0):
    #     raise ValueError("timestamps must be strictly increasing")
    # if capacity_ah <= 0 or r0_ohm < 0 or r1_ohm < 0 or c1_f <= 0:
    #     raise ValueError("capacity and C1 must be positive; resistances cannot be negative")
    # if measurement_variance <= 0:
    #     raise ValueError("measurement_variance must be positive")
        
    if process_covariance is None:
        # Per-sample starting values; tune against the cell and sensor noise.
        process_covariance = np.diag([1e-4, 1e-7])
    process_covariance = np.asarray(process_covariance, dtype=float)
    if process_covariance.shape != (2, 2):
        raise ValueError("process_covariance must be a 2x2 matrix")

    sample_count = len(time_s)
    soc = np.empty(sample_count)
    soc_cc = np.empty(sample_count)
    v1 = np.empty(sample_count)
    state = np.array([0.0, np.clip(soc_initial, 0.0, 1.0)])
    
    covariance = np.diag([0.01, 0.05])
    soc[0] = state[1]
    soc_cc[0] = state[1]
    v1[0] = state[0]
    capacity_coulombs = capacity_ah * 3600.0
    tau_s = r1_ohm * c1_f

    if tau_s <= 0:
        raise ValueError("R1 * C1 must be positive")

    for index in range(1, sample_count):
        dt = time_s[index] - time_s[index - 1]
        # Zero-order hold: use the previous sample over the elapsed interval.
        interval_current = current_a[index - 1]
        decay = np.exp(-dt / tau_s)
        
        # Predict the next state and covariance based on the previous state and the elapsed time.
        predicted_soc = state[1] + dt * interval_current / capacity_coulombs
        predicted_soc_clipped = not 0.0 <= predicted_soc <= 1.0

        predicted_state = np.array(
            [
                decay * state[0] + r1_ohm * (1.0 - decay) * interval_current,
                np.clip(predicted_soc, 0.0, 1.0),
            ]
        )
        transition = np.array([[decay, 0.0], [0.0, 1.0]])
        predicted_covariance = (
            transition @ covariance @ transition.T + process_covariance
        )
        if predicted_soc_clipped:
            predicted_covariance[1, :] = 0.0
            predicted_covariance[:, 1] = 0.0

        predicted_soc = predicted_state[1]
        ocv = np.polyval(ocv_coefficients, predicted_soc)
        docv_dsoc = np.polyval(docv_coefficients, predicted_soc)
        measurement_jacobian = np.array([[1.0, docv_dsoc]])

        # Voltage is the measured terminal voltage, not an OCV-corrected value.
        predicted_voltage = (
            ocv + predicted_state[0] + r0_ohm * current_a[index]
        )
        
        # Correction step: update the state and covariance based on the measured terminal voltage.
        innovation = terminal_voltage_v[index] - predicted_voltage
        innovation_variance = (
            measurement_jacobian
            @ predicted_covariance
            @ measurement_jacobian.T
            + measurement_variance
        ).item()
        gain = (
            predicted_covariance
            @ measurement_jacobian.T
            / innovation_variance
        )

        state = predicted_state + (gain[:, 0] * innovation)
        updated_soc_clipped = not 0.0 <= state[1] <= 1.0
        state[1] = np.clip(state[1], 0.0, 1.0)
        identity_minus_kh = np.eye(2) - gain @ measurement_jacobian
        covariance = (
            identity_minus_kh
            @ predicted_covariance
            @ identity_minus_kh.T
            + measurement_variance * (gain @ gain.T)
        )
        if updated_soc_clipped:
            covariance[1, :] = 0.0
            covariance[:, 1] = 0.0
        covariance = 0.5 * (covariance + covariance.T)

        soc[index] = state[1]
        soc_cc[index] = predicted_state[1]
        v1[index] = state[0]

    return soc, soc_cc, v1


def load_discharge_csv(csv_path):
    """Load the telemetry CSV columns used by this estimator."""
    with csv_path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        required = {"time_[s]", "voltage_[V]", "current_[A]"}
        headers = set(reader.fieldnames or [])
        missing = required - headers
        if missing:
            raise ValueError(f"CSV is missing required columns: {', '.join(sorted(missing))}")
        rows = list(reader)

    if not rows:
        raise ValueError(f"CSV contains no data rows: {csv_path}")

    try:
        time_s = np.array([float(row["time_[s]"]) for row in rows])
        pack_voltage_v = np.array([float(row["voltage_[V]"]) for row in rows])
        discharge_current_a = np.array([float(row["current_[A]"]) for row in rows])
    except (TypeError, ValueError) as error:
        raise ValueError("CSV time, voltage, and current values must be numeric") from error

    return time_s, pack_voltage_v, discharge_current_a


# %%
project_root = Path(__file__).resolve().parents[2]
input_csv = project_root / "Telemetry" / "battery_flight.csv"
plot_path = project_root / "Analysis" / "pics" / f"{input_csv.stem}_ekf_estimate.png"
lut_path = project_root / "Analysis" / "Battery" / "ocv_lut.csv"

config = Config(
    csv_path=input_csv,
    output_path=plot_path,
)

# %%
time_s, pack_voltage_v, discharge_current_a = load_discharge_csv(config.csv_path)
cell_voltage_v = pack_voltage_v / config.series_cells

charge_current_a = -discharge_current_a
c1_f = config.tau_s / config.r1_ohm

# %%
soc, soc_cc, v1 = estimate_soc(
    time_s,
    charge_current_a,
    cell_voltage_v,
    capacity_ah=config.capacity_ah,
    r0_ohm=config.r0_ohm,
    r1_ohm=config.r1_ohm,
    c1_f=c1_f,
    soc_initial=config.soc_initial,
    process_covariance= config.process_covariance,
    measurement_variance= config.measurement_variance,
)
estimated_cell_voltage_v = (
    np.polyval(ocv_coefficients, soc)
    + v1
    + config.r0_ohm * charge_current_a
)
estimated_pack_voltage_v = estimated_cell_voltage_v * config.series_cells
# %%
figure, (voltage_axis, residual_axis,soc_axis) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
voltage_axis.plot(time_s, pack_voltage_v,color="red", label="Measured pack voltage", linewidth=1.5)
voltage_axis.plot(
    time_s,
    estimated_pack_voltage_v,
    label="EKF modelled pack voltage",
    linewidth=1,color="black"
)
voltage_axis.set_ylabel("Voltage [V]")
voltage_axis.set_title("Battery EKF Voltage and State-of-Charge Estimate")
voltage_axis.grid(True, alpha=0.5)
voltage_axis.legend()
residual_axis.plot(
    time_s,
    estimated_pack_voltage_v - pack_voltage_v,
    label="Estimation residual voltage",
    linewidth=1,color="green"
)
residual_axis.set_ylabel("Voltage [V]")
residual_axis.set_title("Voltage Model Residual Voltage")
residual_axis.grid(True, alpha=0.5)
residual_axis.legend()

soc_axis.plot(time_s, soc * 100.0, label="EKF SOC estimate", linewidth=1)
soc_axis.plot(time_s, soc_cc * 100.0, label="Counting Coulomb SOC", linewidth=1)
soc_axis.axhline(100.0, color="black", linestyle="--", alpha=0.4, label="100%")
soc_axis.axhline(0.0, color="black", linestyle="--", alpha=0.4, label="0%")
soc_axis.set_xlabel("Time [s]")
soc_axis.set_ylabel("SOC [%]")
soc_axis.set_ylim(-2, 102)
soc_axis.grid(True, alpha=0.5)
soc_axis.legend()
plt.tight_layout()
figure.savefig(plot_path, dpi=200)
plt.show()
plt.close()

print(f"Saved SOC plot to {plot_path}")
print(f"SOC: {soc[0] * 100:.2f}% -> {soc[-1] * 100:.2f}%")
# %%
