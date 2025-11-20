# Script to read CSV data and plot the data

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from os import listdir

counts_per_rev = 1440  # Counts per revolution of the encoder
rad_per_count = 2 * np.pi / counts_per_rev  # Radians per count

# Dictionary to go from run number to effort values
run_num_to_effort = {
    1: 0,
    2: 10,
    3: 20,
    4: 30,
    5: 40,
    6: 50,
    7: 60,
    8: 70,
    9: 80,
    10: 90,
    11: 100
}

eff_to_df = {}

# Read CSV files from the 'runs/csvs' directory
for file_name in listdir('runs/csvs'):
    run_number = int(file_name.split('_')[0].replace('run', ''))
    effort = run_num_to_effort.get(run_number, None)

    df = pd.read_csv(f'runs/csvs/{file_name}')

    # Detect and remove rows where all numeric columns are zero
    numeric_cols = df.select_dtypes(include=[np.number]).columns
    if len(numeric_cols) > 0:
        all_zero_mask = (df[numeric_cols] == 0).all(axis=1)
        n_all_zero = int(all_zero_mask.sum())
        if n_all_zero > 0:
            print(f"{file_name}: {n_all_zero} all-zero rows removed")
            df = df.loc[~all_zero_mask].reset_index(drop=True)

    # Detect and remove large spikes: sudden large changes between consecutive rows
    # Uses median absolute difference (MAD) of the per-column diffs as a robust scale.
    spike_factor = 15  # multiplier for median-abs-diff to mark a spike (adjustable)
    min_mad = 1e-9
    if len(numeric_cols) > 0 and len(df) > 1:
        diffs = df[numeric_cols].diff().abs()
        median_abs_diff = diffs.median()
        # Replace zeros to avoid zero threshold
        mad = median_abs_diff.replace(0, min_mad)
        spike_mask = (diffs > (spike_factor * mad)).any(axis=1)
        # Ignore NaN in the first diff row
        spike_mask = spike_mask.fillna(False)
        n_spikes = int(spike_mask.sum())
        if n_spikes > 0:
            print(f"{file_name}: {n_spikes} spike rows removed (spike_factor={spike_factor})")
            df = df.loc[~spike_mask].reset_index(drop=True)

    # If DataFrame is empty after cleaning, skip this file
    if df.empty:
        print(f"{file_name}: all rows are zero or no data after cleaning — skipping plot for effort {effort}")
        continue

    # Store cleaned DataFrame in dictionary with effort as key
    eff_to_df[effort] = df


plt.figure(1)
plt.title('Left Motor Velocity vs Time')

plt.figure(2)
plt.title('Left Motor Position vs Time')

# Collect fit parameters for each effort
fits = []

for effort, df in eff_to_df.items():
    # `_time` in CSV is in milliseconds; convert to seconds
    time = df['_time'] / 1000.0
    left_pos = df['_left_pos'] * rad_per_count
    right_pos = df['_right_pos'] * rad_per_count
    left_vel = df['_left_vel'] * rad_per_count
    right_vel = df['_right_vel'] * rad_per_count

    plt.figure(1)
    plt.plot(time, left_vel, label=f'Effort: {effort}%')

    plt.figure(2)
    plt.plot(time, left_pos, label=f'Effort: {effort}%')

plt.figure(1)
plt.xlabel('Time (s)')
plt.ylabel('Left Motor Velocity (rad/s)')
plt.legend()
plt.figure(2)
plt.xlabel('Time (s)')
plt.ylabel('Left Motor Position (rad)')
plt.legend()
# Compute steady-state (average of last N samples) of left velocity per effort
steady_n = 100
steady_by_eff = {}
for effort, df in eff_to_df.items():
    # Skip missing effort keys or the 0% effort to produce 10 discrete points (10-100)
    if effort is None or effort == 0:
        continue

    if '_left_vel' not in df.columns:
        print(f"effort {effort}: '_left_vel' column missing — skipping")
        continue

    left_vel = (df['_left_vel'] * rad_per_count).dropna()
    if left_vel.empty:
        print(f"effort {effort}: no left velocity data after cleaning — skipping")
        continue

    last_vals = left_vel.tail(min(steady_n, len(left_vel)))
    steady = float(last_vals.mean())
    steady_by_eff[int(effort)] = steady

# Prepare voltage (V) from effort percent: voltage = effort% * 8V
voltage_by_eff = {e: (e / 100.0) * 8 for e in steady_by_eff.keys()}

# Sort by voltage (effort) for plotting
efforts_sorted = sorted(steady_by_eff.keys())
voltages = [voltage_by_eff[e] for e in efforts_sorted]
steady_vals = [steady_by_eff[e] for e in efforts_sorted]

plt.figure(3)
plt.title('Steady-State Left Velocity vs Voltage')
if len(voltages) == 0:
    print('No steady-state points to plot (no cleaned data found).')
else:
    plt.plot(voltages, steady_vals, 'o', markersize=4)
    plt.xlabel('Voltage (V)')
    plt.ylabel('Steady-State Left Velocity (rad/s)')
    plt.grid(True)
    if len(voltages) != 10:
        print(f'Warning: expected 10 discrete points, found {len(voltages)}')

    # Fit a line (1st degree polynomial) to steady-state points
    m = b = None
    if len(voltages) >= 2:
        coeffs = np.polyfit(voltages, steady_vals, 1)
        m, b = float(coeffs[0]), float(coeffs[1])
        print(f'Fitted line: steady_velocity = {m:.6e} * voltage + {b:.6e} -> K = {m:.4f} rad/s/V')

        # Plot fitted line over voltage range
        xs = np.linspace(min(voltages), max(voltages), 200)
        ys = m * xs + b
        plt.plot(xs, ys, '-', label=f'Fit: v={m:.4e}*V+{b:.4e}')
        plt.legend()

    else:
        print('Not enough points to fit a line (need >=2).')

# Plot ln(1 - omega/omega_max) vs time for each run (overlayed)
plt.figure(4)
plt.title('ln(1 - omega/omega_max) vs Time (Left Motor)')

for effort, df in eff_to_df.items():
    # Skip missing or zero-effort runs
    if effort is None or effort == 0:
        continue

    # Ensure we have steady-state value for this effort
    omega_max = steady_by_eff.get(int(effort), None)
    if omega_max is None or omega_max == 0:
        print(f'effort {effort}: no steady omega_max found — skipping ln plot')
        continue

    if '_left_vel' not in df.columns or '_time' not in df.columns:
        print(f'effort {effort}: required columns missing — skipping ln plot')
        continue

    # `_time` in CSV is in milliseconds; convert to seconds
    time = df['_time'] / 1000.0
    omega = (df['_left_vel'] * rad_per_count).astype(float)

    # Compute ratio and guard against values >= 1 or NaNs
    ratio = omega / float(omega_max)
    # Create mask where (1 - ratio) is positive and finite
    valid_mask = np.isfinite(ratio) & ((1.0 - ratio) > 0)
    valid_idx = np.where(valid_mask)[0]
    if valid_idx.size == 0:
        print(f'effort {effort}: no valid points for ln(1 - omega/omega_max) — skipping')
        continue

    # Use only the first N valid points (user requested 50)
    max_points = 10
    sel_idx = valid_idx[:max_points]
    y = np.log(1.0 - ratio.to_numpy()[sel_idx])
    t_sel = time.to_numpy()[sel_idx]
    plt.plot(t_sel, y, label=f'Effort: {effort}%')

    # (Per-run fit removed — only global fit will be computed)

    # accumulate for global fit
    try:
        if 't_sel' in locals() and 'y' in locals() and len(t_sel) > 0:
            if 't_all' not in locals():
                t_all = []
                y_all = []
            t_all.extend(t_sel.tolist())
            y_all.extend(y.tolist())
    except Exception:
        pass

plt.xlabel('Time (s)')
plt.ylabel('ln(1 - omega/omega_max)')
plt.legend()
plt.grid(True)

# Compute and overlay a single global fit across all runs (if we collected points)
if 't_all' in locals() and len(t_all) >= 2:
    try:
        coeffs_g = np.polyfit(np.array(t_all), np.array(y_all), 1)
        m_g, b_g = float(coeffs_g[0]), float(coeffs_g[1])
        tau_g = -1.0 / m_g if m_g != 0 else float('inf')
        print(f'Global fit: ln(1 - ω/ω_max) = {m_g:.6e} * t + {b_g:.6e}  ->  τ = {tau_g:.4f} s')

        # overlay global fit across time span
        ts = np.linspace(min(t_all), max(t_all), 300)
        y_g = m_g * ts + b_g
        plt.plot(ts, y_g, 'k-', linewidth=2.0, label='Global fit')

        # add global fit to fits list for CSV output
        fits.append({'effort': 'global', 'm': m_g, 'b': b_g, 'tau_s': tau_g})
    except Exception as e:
        print(f'Global fit failed: {e}')

# Create plots directory and save figures
plots_dir = os.path.join('runs', 'plots')
os.makedirs(plots_dir, exist_ok=True)

# Save figure 1: Left Motor Velocity vs Time
plt.figure(1)
fname1 = os.path.join(plots_dir, 'left_velocity_vs_time.png')
plt.savefig(fname1, dpi=200, bbox_inches='tight')
print(f'Saved: {fname1}')

# Save figure 2: Left Motor Position vs Time
plt.figure(2)
fname2 = os.path.join(plots_dir, 'left_position_vs_time.png')
plt.savefig(fname2, dpi=200, bbox_inches='tight')
print(f'Saved: {fname2}')

# Save figure 3: Steady-State Left Velocity vs Voltage
plt.figure(3)
fname3 = os.path.join(plots_dir, 'steady_state_left_velocity_vs_voltage.png')
plt.savefig(fname3, dpi=200, bbox_inches='tight')
print(f'Saved: {fname3}')

# Save figure 4: ln(1 - omega/omega_max) vs Time
plt.figure(4)
fname4 = os.path.join(plots_dir, 'ln_1_minus_omega_over_omega_max_vs_time.png')
plt.savefig(fname4, dpi=200, bbox_inches='tight')
print(f'Saved: {fname4}')

# Save fit parameters CSV if fits were computed
fits_csv = os.path.join(plots_dir, 'ln_fit_params.csv')
try:
    if 'fits' in locals() and len(fits) > 0:
        pd.DataFrame(fits).to_csv(fits_csv, index=False)
        print(f'Saved fit parameters: {fits_csv}')
    else:
        print('No fit parameters to save.')
except Exception as e:
    print(f'Failed to save fit parameters CSV: {e}')

# Show all plots
plt.show()

