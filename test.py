# test.py
#
# ==============================================================================
# Script to run on PC to run motor characterization test and perform data
# collection
# ==============================================================================

### TO-DO:
#
#

# ==============================================================================
# IMPORTS
# ==============================================================================

from serial import Serial, SerialException
from time import sleep
import matplotlib.pyplot as plt
import msvcrt
import pandas as pd
import numpy as np
import os
import queue as local_queue
import time
import math

# ==============================================================================
# INITIALIZATION
# ==============================================================================

# ------------------------------------------------------------------------------
# Variables
key = ''                    # Stores value of key pressed as a string
running = False             # Set when motor testing is progress
streaming = False           # Set when data streaming is in progress
runs = {}                   # Dict to contain runs
run_count = 0               # Number of runs
frame_buffer = ""           # Buffer for delimiter-framed lines
control_mode = 0            # 0 = effort mode, 1 = velocity mode, 2 = line follow mode
effort = 0                  # Current effort value
setpoint = 0                # Current velocity setpoint in rad/s
kp = 0.0                    # Current proportional gain
ki = 0.0                    # Current integral gain
k_line = 0.0                # Current line following gain
first = True
done = False
# mode = 1                  # 1, 2, 3 = straight, pivot, arc
queue = local_queue.Queue() # Queue to hold tests
queuing = False             # Set when queuing tests
test_origin = "manual"      # tracks how the current test started: "manual" or "queue"

# --------------------------------------------------------------------------
# Dictionary to map control mode number to string
control_mode_dict = {0: "Effort", 1: "Velocity", 2: "Line Following"}

# --------------------------------------------------------------------------
# User prompt string
user_prompt = '''\r\nCommand keys:
    t      : Select a test to run: Effort, Velocity, Line Following
    u      : Queue tests
    r      : Run test
    k      : Kill (stop) motors
    s      : Stream data
    d      : Save complete data to CSV and optionally create plots
    b      : Check battery voltage (prints to this terminal)
    c      : Calibrate sensors: IR sensors or IMU
    h      : Help / show this menu
    ctrl-c : Interrupt this program
'''

# --------------------------------------------------------------------------
# CONSTANTS FOR UNIT CONVERSIONS
GEAR_RATIO = 3952/33  # Gear ratio of motor to wheel (~120)
CPR_MOTOR = 12 # Counts per rev of the motor shaft (before gearbox)
CPR_WHEEL = GEAR_RATIO*CPR_MOTOR  # Counts per rev of the wheel (~1440)
RAD_PER_COUNT = 2 * (math.pi) / CPR_WHEEL  # Radians per count
WHEEL_RADIUS_MM = 35  # Wheel radius in mm

# ==============================================================================
# HELPER FUNCTIONS
# ==============================================================================

# Map an effort percentage (0-100) to the single-character key
def eff_to_key(eff):
    if eff == 100:
        return 'a'
    # assume efforts are multiples of 10
    digit = int(eff // 10)
    return str(digit)

# ------------------------------------------------------------------------------
# Map a single-character key to an effort percentage (0-100)
def key_to_eff(key):
    if key == 'a':
        return 100
    if key.isdigit():
        digit = int(key)
        return digit * 10
    return None

# ------------------------------------------------------------------------------
# Function to create dictionary for storing data from one run
def create_run(control_val, run_num, size):
    time = np.zeros(size)
    p1 = np.zeros(size)
    p2 = np.zeros(size)
    v1 = np.zeros(size)
    v2 = np.zeros(size)
    ctrl = np.zeros(size)  # Store control value (effort or setpoint)

    df = pd.DataFrame({
        "_time": time,
        "_control": ctrl,
        "_left_pos": p1,
        "_right_pos": p2,
        "_left_vel": v1,
        "_right_vel": v2
    })

    return {"control_val": control_val, "run_num": run_num, "size": size, "motor_data": df}

# ------------------------------------------------------------------------------
# Function to clean DataFrame by removing all-zero rows except leading zeros
def clean_data(df, cols=None, mode='all'):
    """
    Clean DataFrame by removing all zero rows except initial zeros up to the first non-zero data.
    
    Parameters:
      df: pandas.DataFrame
      cols: list of columns to check. If None, defaults to all motor_data columns.
      mode: 'all' -> remove rows where ALL specified cols are zero;
            'any' -> remove rows where ANY specified cols are zero.

    Returns: (cleaned_df, removed_count)
    """
    if cols is None:
        cols = ["_time", "_left_pos", "_right_pos", "_left_vel", "_right_vel"]

    # Ensure columns exist
    cols = [c for c in cols if c in df.columns]
    if not cols:
        return df, 0

    # Create boolean mask for zero-rows according to mode
    if mode == 'any':
        zero_mask = (df[cols] == 0).any(axis=1)
    else:
        zero_mask = (df[cols] == 0).all(axis=1)

    # Find the indices of non-zero rows
    nonzero_idx = (~zero_mask).to_numpy().nonzero()[0]
    
    if len(nonzero_idx) == 0:
        # All rows are zero -> keep first row only
        if len(df) <= 1:
            return df.reset_index(drop=True), 0
        cleaned = df.iloc[:1].reset_index(drop=True)
        removed = len(df) - 1
        return cleaned, removed

    # Find first and last non-zero indices
    first_nonzero = int(nonzero_idx[0])
    last_nonzero = int(nonzero_idx[-1])

    # Keep all rows from start up to first non-zero (preserve leading zeros)
    # plus all non-zero rows up to the last non-zero row
    cleaned = df.iloc[:last_nonzero + 1].copy()
    
    # Remove any zero rows between first and last non-zero (excluding leading zeros)
    if first_nonzero > 0:  # If we have leading zeros
        # Keep all rows up to first_nonzero (leading zeros)
        # Then only keep non-zero rows after that
        mask = pd.Series(True, index=cleaned.index)
        mask[first_nonzero:] = ~zero_mask[first_nonzero:last_nonzero + 1]
        cleaned = cleaned[mask].reset_index(drop=True)
    else:
        # No leading zeros, just keep non-zero rows
        cleaned = cleaned[~zero_mask[:last_nonzero + 1]].reset_index(drop=True)

    removed = len(df) - len(cleaned)
    return cleaned, removed

# ------------------------------------------------------------------------------
# Function to perform START / ACK handshake for data streaming
def start_stream_handshake(ser, timeout=2.0, retries=3):
    """Send START and wait for ACK from MCU. Returns True if ACK received."""
    for attempt in range(retries):
        try:
            ser.write(b'#START\n')
            ser.flush()
        except Exception:
            return False

        start = time.time()
        while time.time() - start < timeout:
            try:
                line = ser.readline().decode(errors='ignore').strip()
            except Exception:
                line = ''
            if not line:
                continue
            if line == '$ACK':
                return True
            # ignore other lines
        # retry
    return False

# ==============================================================================
# MAIN PROGRAM
# ==============================================================================

# Create 'runs' directory if it doesn't exist
try:
    os.makedirs('runs', exist_ok=True)
    # print("Output directory 'runs' is ready")
except Exception as e:
    print(f"Warning: Could not create 'runs' directory: {e}")

# Establish Bluetooth connection
try:
    ser = Serial('COM9', baudrate=115200, timeout=1)
except SerialException:
    print("Unable to connect to port")

print("Begin Program:")
print(user_prompt)

while True:
    try:
        # Check for key pressed
        if msvcrt.kbhit():
            key = msvcrt.getch().decode().lower()

            if key == 't':
                if running:
                    print("Cannot select test while test is running")
                elif streaming:
                    print("Cannot select test while data is streaming")
                else:
                    print("Select test to run:")
                    print("  e: Effort mode")
                    print("  v: Velocity mode")
                    print("  l: Line Following mode")
                    selected = input("Enter choice (e, v, l, or q to quit): ")
                    if selected == 'e':
                        control_mode = 0
                        print("Selected Effort mode")
                        # Accept a single key (0-9 or 'a') or an integer percentage 0-100
                        key_in = input("Enter effort key (0-9 or 'a' for 100%) or percent (0-100): ").strip()
                        
                        # Only use key_to_eff for single characters
                        if len(key_in) == 1:
                            eff_val = key_to_eff(key_in)
                            if eff_val is None:
                                print("Invalid effort entry. Use 0-9, 'a', or a number 0-100.")
                                continue
                        else:
                            # Multi-digit input: parse as integer percent directly
                            try:
                                eff_val = int(key_in)
                                if eff_val < 0 or eff_val > 100:
                                    raise ValueError()
                            except Exception:
                                print("Invalid effort entry. Use 0-9, 'a', or a number 0-100.")
                                continue
                        effort = eff_val
                        line = 'e' + eff_to_key(effort)

                    elif selected == 'v':
                        control_mode = 1
                        print("Selected Velocity mode")
                        gains = input("Enter setpoint (mm/s), Kp, and Ki separated by commas (e.g., 100,1.5,0.1): ")
                        try:
                            sp_str, kp_str, ki_str = gains.split(',')
                            # Convert user's mm/s setpoint to rad/s
                            v_mm_s = float(sp_str) # mm/s
                            omega_rad_s = v_mm_s / WHEEL_RADIUS_MM  # rad/s
                            # Scale setpoint by 100 to send as integer
                            setpoint_scaled = int(omega_rad_s * 100)
                            kp = float(kp_str)
                            ki = float(ki_str)
                            # Scale gains by 100 to send as integers
                            kp_scaled = int(kp * 100)
                            ki_scaled = int(ki * 100)
                            # Send as 4-digit integers
                            line = f'v{setpoint_scaled:04d}{kp_scaled:04d}{ki_scaled:04d}'
                            # store setpoint in mm/s for the data log
                            setpoint = v_mm_s
                        except ValueError:
                            print("Invalid format. Please enter three numbers separated by a comma.")

                    elif selected == 'l':
                        control_mode = 2
                        print("Selected Line Following mode")
                        gains = input("Enter Kp, Ki, K_line, and target separated by commas (e.g., 1.5,0.1,8.0,0.5): ")
                        try:
                            kp_str, ki_str, k_line_str, target_str = gains.split(',')
                            # Convert user's mm/s target to rad/s
                            v_mm_s = float(target_str) # mm/s
                            omega_rad_s = v_mm_s / WHEEL_RADIUS_MM  # rad/s
                            # Scale target by 100 to send as integer
                            target_scaled = int(omega_rad_s * 100)
                            kp = float(kp_str)
                            ki = float(ki_str)
                            k_line = float(k_line_str)
                            # Scale gains by 100 to send as integers
                            kp_scaled = int(kp * 100)
                            ki_scaled = int(ki * 100)
                            k_line_scaled = int(k_line * 100)
                            line = f'l{kp_scaled:04d}{ki_scaled:04d}{k_line_scaled:04d}{target_scaled:04d}'
                        except ValueError:
                            print("Invalid format. Please enter four numbers separated by commas.")
                    
                    elif selected == 'q':
                        print("Test selection cancelled.")
                        continue
                    else:
                        print("Invalid selection. Enter 'e', 'v', 'l', or 'q' to quit.")

                    # Send configuration line to Romi    
                    ser.write(line.encode())
                    print("Command sent to Romi. Hit 'r' to run the test.")

            elif key == 'u':
                if running:
                    print("Cannot add tests to queue while test is running")
                elif streaming:
                    print("Cannot add tests to queue while data is streaming")
                else:
                    print(("Select from the following:"))
                    print("  l: View items in queue")
                    print("  c: Clear queue")
                    print("  a: Add test to queue")
                    selected = input("Enter choice (l, c, a, or q to quit): ")
                    if selected == 'l':
                        if queue.is_empty():
                            print("Queue is empty.")
                            print(user_prompt)
                        else:
                            print("Current queue:")
                            for i in queue.items:
                                # Format for test is (mode, param1, param2, ...)
                                print(i)
                    
                    elif selected == 'c':
                        while not queue.is_empty():
                            queue.dequeue()
                        print("Queue cleared.")
                        print(user_prompt)
                    elif selected == 'a':
                        # Reset the run counter before starting a new queue
                        run_count = 0
                        runs.clear()

                        # For now only allow adding effort tests
                        eff = input("Enter effort test in the following format: start, end, step (e.g., 0,100,10): ")
                        try:
                            start_str, end_str, step_str = eff.split(',')
                            start = int(start_str)
                            end = int(end_str)
                            step = int(step_str)
                            if start < 0 or end > 100 or step <= 0 or start > end:
                                raise ValueError("Invalid range or step")
                            # Add tests to queue
                            for e in range(start, end + 1, step):
                                queue.enqueue( ('effort', e) )
                        except ValueError as e:
                            print(f"Invalid format: {e}")
                        queuing = True if not queue.is_empty() else False
                    elif selected == 'q':
                        print("Queue operation cancelled.")
                        continue
                    
            elif key == 'r':
                if running:
                    print("Test is already running")
                elif streaming:
                    print("Data is streaming.")
                else:
                    print("Starting test...")
                    running = True
                    ser.write(b'r')
            elif key == 'k':
                if running:
                    # Kill motors
                    ser.write(b'k')
                    queuing = False
                    print("End test. Stop motors")
                else:
                    print("Motors are already off")
            elif key == 's':
                if running:
                    print("Test is already running, cannot stream data now.")
                elif streaming:
                    print("Data is already streaming.")
                else:
                    # # Start handshake with MCU
                    # ok = start_stream_handshake(ser)
                    # if not ok:
                    #     # Fallback: try legacy single-char start for compatibility
                    #     try:
                    #         ser.write(b's')
                    #     except Exception:
                    #         pass
                    #     print("Stream handshake failed; sent legacy 's' command (best-effort).")
                    # else:
                    #     print("MCU acknowledged streaming request")

                    ser.write(b's')

                    # Flush any initial bytes
                    # if ser.in_waiting:
                    #     ser.read()

                    print("Data streaming to PC...")
                    streaming = True

            elif key == 'c':
                if running:
                    print("Cannot calibrate while test is running")
                elif streaming:
                    print("Cannot calibrate while data is streaming")
                else:
                    print("Select calibration to perform:")
                    print("  1: IR Sensors")
                    print("  2: IMU")
                    selected = input("Enter choice (1 or 2): ")
                    if selected == '1':
                        # print("IR sensor calibration command sent to Romi.")
                        key = input("Place Romi on white surface and press any key to continue...")
                        ser.write(b'w')
                        print("White calibration done. Now place Romi on black surface.")
                        key = input("Place Romi on black surface and press any key to continue...")
                        ser.write(b'b')
                        print("Black calibration done.")
                    elif selected == '2':
                        # print("IMU calibration command sent to Romi.")
                        ser.write(b'i')
                        key = input("Follow IMU calibration procedure and press any key to continue...")
                        ser.write(b'j')
                        print("IMU calibration command sent.")
                    else:
                        print("Invalid selection. Enter '1' or '2'.")

            elif key == 'b':
                # Request battery voltage (firmware will respond with a number and newline)
                if streaming:
                    print("Battery query deferred (currently streaming)")
                else:
                    ser.write(b'V')
                    # try to read a short reply
                    sleep(0.05)
                    try:
                        line = ser.readline().decode().strip()
                        if line:
                            print(f"Battery voltage: {line} V")
                        else:
                            print("Requested battery voltage; waiting for response...")
                    except Exception:
                        print("Requested battery voltage.")

            elif key == 'd':
                # Save all data to CSV and optionally create plots
                # Ensure base 'runs' and subfolders exist
                try:
                    os.makedirs('runs', exist_ok=True)
                    csv_dir = os.path.join('runs', 'csvs')
                    plots_dir = os.path.join('runs', 'plots')
                    os.makedirs(csv_dir, exist_ok=True)
                    os.makedirs(plots_dir, exist_ok=True)
                except Exception as e:
                    print(f"Warning: could not create output directories: {e}")

                if not runs:
                    print("No runs available to display or save.")
                else:
                    print("Select target:")
                    print("  l : Latest run")
                    print("  a : All runs")
                    print("  c : Cancel")
                    target = input("Enter choice (l/a/c): ").strip().lower()
                    if target == 'c' or target == '':
                        print("Operation cancelled.")
                        continue

                    print("Create plots? (y/n): ")
                    create_plots = input().strip().lower() == 'y'

                    if create_plots:
                        # Prompt user for which data channels to plot
                        print("Select data to plot (comma-separated):")
                        print("  1 : left motor velocity")
                        print("  2 : right motor velocity")
                        print("  3 : left motor position")
                        print("  4 : right motor position")
                        data_sel = input("Enter choices (e.g. 1,2,3): ").strip()
                        if not data_sel:
                            print("No data selections made. Skipping plots.")
                            create_plots = False
                        else:
                            # map selection to dataframe columns and short codes
                            sel_map = {
                                '1': ('_left_vel', 'Left velocity', 'lv'),
                                '2': ('_right_vel', 'Right velocity', 'rv'),
                                '3': ('_left_pos', 'Left position', 'lp'),
                                '4': ('_right_pos', 'Right position', 'rp')
                            }
                            chosen = []
                            for token in [t.strip() for t in data_sel.split(',')]:
                                if token in sel_map and token not in chosen:
                                    chosen.append(token)
                            if not chosen:
                                print("No valid data selections found. Skipping plots.")
                                create_plots = False

                    def normalize_mode(mode_val):
                        try:
                            return mode_val[0].lower()
                        except Exception:
                            return 'e'

                    if target == 'l':
                        run_name = f'run{run_count}'
                        meta = runs.get(run_name)
                        if not meta:
                            print(f"Run {run_name} not found.")
                            continue

                        df = meta["motor_data"]
                        df_clean, removed = clean_data(df, mode='all')
                        if removed:
                            print(f"Removed {removed} all-zero rows from {run_name} before plotting/saving")

                        mode_char = normalize_mode(meta.get('mode', 'E'))
                        control_val = meta.get('control_val', 'unknown')

                        if mode_char == 'v':
                            params = meta.get('params', {}) or {}
                            kp = params.get('kp', 0.0) * 100
                            ki = params.get('ki', 0.0) * 100
                            label = f"SP={control_val} Kp={kp:.2f} Ki={ki:.2f}"
                            base_name = f"{run_name}_V_sp{control_val}_Kp{kp:.2f}_Ki{ki:.2f}"
                        else:
                            label = f"Effort={control_val}"
                            base_name = f"{run_name}_E_eff{control_val}"

                        # Always save complete CSV with all data columns
                        try:
                            csv_name = os.path.join(csv_dir, base_name + ".csv")
                            cols = ["_time", "_left_pos", "_right_pos", "_left_vel", "_right_vel"]
                            cols = [c for c in cols if c in df_clean.columns]
                            df_clean[cols].to_csv(csv_name, index=False)
                            print(f"Saved complete motor data to {csv_name}")
                        except Exception as e:
                            print(f"Failed to save CSV for {run_name}: {e}")

                        if create_plots:
                            # Save separate plot per selected channel for this single run
                            for k in chosen:
                                col, readable, code = sel_map[k]
                                try:
                                    plt.figure()

                                    # Determine y-axis label with units
                                    if "pos" in col:
                                        ylabel_text = readable + " [mm]"
                                    elif "vel" in col:
                                        ylabel_text = readable + " [mm/s]"
                                    else:
                                        ylabel_text = readable

                                    # CONVERT THE RAW CSV DATA (COUNTS)
                                    x_data = df_clean["_time"]
                                    y_data = df_clean[col].copy() # Get raw counts or counts/s

                                    # Convert based on column type
                                    if "pos" in col:
                                        # Convert counts to mm
                                        y_data = y_data * RAD_PER_COUNT * WHEEL_RADIUS_MM
                                    elif "vel" in col:
                                        # Convert counts/s to mm/s
                                        y_data = y_data * RAD_PER_COUNT * WHEEL_RADIUS_MM

                                    plt.plot(x_data, y_data, label=label)
                                    plt.xlabel("Time, [ms]")
                                    plt.ylabel(ylabel_text)
                                    try:
                                        plt.legend()
                                    except Exception:
                                        pass
                                    fig_name = os.path.join(plots_dir, base_name + "_" + code + ".png")
                                    plt.savefig(fig_name)
                                    plt.close()
                                    print(f"Saved figure to {fig_name}")
                                except Exception as e:
                                    print(f"Failed to save figure {code} for {run_name}: {e}")

                        print(user_prompt)

                    elif target == 'a':
                        # Save CSV with all runs and optionally create combined plots
                        for run_name, meta in runs.items():
                            df = meta["motor_data"]
                            df_clean, removed = clean_data(df, mode='all')
                            if removed:
                                print(f"Removed {removed} all-zero rows from {run_name} before saving")

                            mode_char = normalize_mode(meta.get('mode', 'E'))
                            control_val = meta.get('control_val', 'unknown')

                            if mode_char == 'v':
                                kp = meta.get('params', {}).get('kp', 0) if meta.get('params') else 0
                                ki = meta.get('params', {}).get('ki', 0) if meta.get('params') else 0
                                base_name = f"{run_name}_V_sp{control_val}_Kp{kp:.2f}_Ki{ki:.2f}"
                            else:
                                base_name = f"{run_name}_E_eff{control_val}"

                            # Save complete CSV with all data columns
                            try:
                                csv_name = os.path.join(csv_dir, base_name + ".csv")
                                cols = ["_time", "_left_pos", "_right_pos", "_left_vel", "_right_vel"]
                                cols = [c for c in cols if c in df_clean.columns]
                                df_clean[cols].to_csv(csv_name, index=False)
                                print(f"Saved complete motor data to {csv_name}")
                            except Exception as e:
                                print(f"Failed to save CSV for {run_name}: {e}")

                        # Create combined plots if requested
                        if create_plots:
                            for k in chosen:
                                col, readable, code = sel_map[k]
                                plt.figure()
                                
                                # Determine y-axis label with units
                                if "pos" in col:
                                    ylabel_text = readable + " [mm]"
                                elif "vel" in col:
                                    ylabel_text = readable + " [mm/s]"
                                else:
                                    ylabel_text = readable

                                for run_name, meta in runs.items():
                                    df = meta["motor_data"]
                                    df_clean, removed = clean_data(df, mode='all')

                                    mode_char = normalize_mode(meta.get('mode', 'E'))
                                    control_val = meta.get('control_val', 'unknown')

                                    if mode_char == 'v':
                                        kp = meta.get('params', {}).get('kp', 0) if meta.get('params') else 0
                                        ki = meta.get('params', {}).get('ki', 0) if meta.get('params') else 0
                                        label = f"{run_name} (V) sp={control_val} Kp={kp:.2f} Ki={ki:.2f}"
                                    else:
                                        label = f"{run_name} (E) eff={control_val}"

                                    # CONVERT THE RAW CSV DATA (COUNTS)
                                    try:
                                        x_data = df_clean["_time"]
                                        y_data = df_clean[col].copy() # get raw counts or counts/s

                                        if "pos" in col:
                                            # Convert counts to mm
                                            y_data = y_data * RAD_PER_COUNT * WHEEL_RADIUS_MM
                                        elif "vel" in col:
                                            # Convert counts/s to mm/s
                                            y_data = y_data * RAD_PER_COUNT * WHEEL_RADIUS_MM

                                        # Plot the converted data
                                        plt.plot(x_data, y_data, label=label)
                                    except Exception as e:
                                        print(f"Failed to add plot {readable} for {run_name}: {e}")

                                try:
                                    plt.xlabel("Time, [ms]")
                                    plt.ylabel(ylabel_text)
                                    plt.legend()
                                    fig_all = os.path.join(plots_dir, f"all_runs_{code}.png")
                                    plt.savefig(fig_all)
                                    plt.close()
                                    print(f"Saved plot of all runs for {readable} to '{fig_all}'")
                                except Exception as e:
                                    print(f"Failed to save combined plot for {readable}: {e}")

                        print(user_prompt)
                    else:
                        print("Unknown target choice. Press 'd' to try again.")
                        print(user_prompt)
                        pass
            
            elif key == 'h':
                # Print helpful prompt
                print(user_prompt)

            # Clear any remaining input
            while msvcrt.kbhit():
                msvcrt.getch()

            
        # If not running a test and queuing tests, start the next test
        elif not running and not streaming and queuing:
            if not queue.is_empty():
                next_test = queue.dequeue()
                mode = next_test[0]
                if mode == 'effort':
                    effort = next_test[1]
                    line = 'e' + eff_to_key(effort)
                    control_mode = 0
                    print(f"Queued next {mode} test: {effort}%")
                else:
                    print(f"Unknown test mode in queue: {mode}")
                    continue
                # Send command to Romi 
                ser.write(line.encode())
                sleep(0.2)
                # Start test
                ser.write(b'r')
                running = True
        
        # Handle serial input
        else:
            # Check for bytes waiting
            if ser.in_waiting:
                # print("\r\nWe are reading you loud and clear!")
                if running:
                    ch = ser.read().decode()
                    if ch == 'q':
                        print("Testing is done. Press 's' to stream data")
                        # print(user_prompt)
                        sleep(0.1)
                        ser.write(b's')  # Request data streaming
                        streaming = True
                        running = False
                elif streaming:
                    # Create new run for the run
                    if first:
                        if control_mode == 0:
                            mode_name = "Effort"
                            params = None
                        elif control_mode == 1:
                            mode_name = "Velocity"
                            params = {'setpoint': setpoint, 'kp': kp, 'ki': ki}
                        sample_size = 250    # Default sample size

                        # Create new run (run_count already at correct value)
                        run_name = f'run{run_count}'
                        runs[run_name] = create_run(effort, run_count, sample_size)
                        if params:
                            runs[run_name]['params'] = params
                        runs[run_name]['mode'] = mode_name
                        print(f"Run {run_name} created in {mode_name} mode (size={sample_size})")

                        recv_line_num = 0
                        first = False
                        run_count += 1 # increment after creating the run
                    
                    else:
                        # Read all the available bytes in the UART buffer
                        chunk = ser.read(ser.in_waiting or 1).decode()
                        if chunk:
                            frame_buffer += chunk

                        # For complete <S> ... <E> frames, extract lines
                        while "<S>" in frame_buffer and "<E>" in frame_buffer:
                            start = frame_buffer.find("<S>")
                            end = frame_buffer.find("<E>", start)

                            if end == -1: # if there is no <E> found, break
                                break # incomplete frame

                            # Extract the inside contents
                            frame = frame_buffer[start+3 : end]
                            frame_buffer = frame_buffer[end+3:]  # Remove processed frame

                            # END OF STREAM CHECK
                            if frame == "#END":
                                print("Received end of stream marker from Romi. Hit 'd' to print data.")
                                # Acknowledge end of stream
                                try:
                                    ser.write(b'ACK_END\n')
                                except Exception:
                                    pass
                                first = True
                                streaming = False
                                sleep(0.2)
                                continue

                            # For a normal frame, parse the CSV payload
                            try:
                                idx_str, time_s, left_pos_c, right_pos_c, left_vel_c, right_vel_c = frame.split(',')
                                idx = int(idx_str)
                            except ValueError:
                                print(f"[Rejected] Bad frame contents: '{frame}'")
                                continue

                            size = runs.get(run_name, {}).get('size')
                            if idx < 0 or idx >= size:
                                print(f"[Rejected] Index {idx} out of range (size={size})")
                                continue

                            # Store the values
                            runs[run_name]["motor_data"].loc[idx,"_time"] = float(time_s)
                            runs[run_name]["motor_data"].loc[idx, "_left_pos"] = float(left_pos_c)
                            runs[run_name]["motor_data"].loc[idx, "_right_pos"] = float(right_pos_c)
                            runs[run_name]["motor_data"].loc[idx, "_left_vel"] = float(left_vel_c)
                            runs[run_name]["motor_data"].loc[idx, "_right_vel"] = float(right_vel_c)

                else:
                    pass
            else:
                pass

    except KeyboardInterrupt:
        break

if 'ser' in locals() and getattr(ser, 'is_open', False):
    try:
        ser.write(b'k')         # Kill motors
    except Exception:
        pass
    try:
        ser.close()                 # Close port
    except Exception:
        pass
else:
    # If ser isn't available, nothing to close
    pass
print("Program interrupted")