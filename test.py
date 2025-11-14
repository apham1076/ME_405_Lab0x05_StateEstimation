# Script to run on PC to run motor characterization test and perform data collection

# Last modified: 11-13-25 7:30 PM

# # ************* NOTES **************

# ************* TO-DO **************
# Figure out why data is so jaggedy (message Charlie on Piazza)
# Debug data streaming: use START and ACK


from serial import Serial, SerialException
from time import sleep
import matplotlib.pyplot as plt
import msvcrt
import pandas as pd
import numpy as np
import os
import queue as local_queue
import time

key = ''                    # Stores value of key pressed as a string
running = False             # Set when motor testing is progress
streaming = False           # Set when data streaming is in progress
runs = {}                   # Dict to contain runs
run_count = 0               # Number of runs
control_mode = 0            # 0 = effort mode, 1 = velocity mode, 2 = line follow mode
effort = 0                 # Current effort value
setpoint = 0               # Current velocity setpoint in rad/s
kp = 0.0                    # Current proportional gain
ki = 0.0                    # Current integral gain
k_line = 0.0                # Current line following gain
first = True
done = False
# mode = 1                    # 1, 2, 3 = straight, pivot, arc
effort = 0                   # Current effort value
my_queue = local_queue.Queue()           # Queue to hold tests

control_mode_dict = {0: "Effort", 1: "Velocity", 2: "Line Following"}

old_user_prompt = '''\r\nCommand keys:
    m      : Toggle mode: Effort, Velocity, and Line Following
    0-9,a  : Set effort (0-100%) for open-loop control [Effort mode]
    t      : Set setpoint (rad/s) for closed-loop control [Velocity mode]
    p      : Set proportional gain (Kp) for controller [Velocity mode]
    i      : Set integral gain (Ki) for controller [Velocity mode]
    g      : GO (start test)
    k      : Kill (stop) motors
    r      : Run automated sequence (0 to 100% by 10%)
    x      : Plot all runs' left velocity
    s      : Stream data
    n      : Toggle driving mode (straight/pivot/arc)
    d      : Save latest run to CSV and plot PNG
    c      : Run automated closed-loop test
    w      : IR white calibration (prints table from Nucleo USB REPL)
    b      : IR black calibration (prints table from Nucleo USB REPL)
    l      : Set IR line following gains (lf_kp, lf_ki)
    v      : Query battery voltage (prints to this terminal)
    h      : Help / show this menu
    ctrl-c : Interrupt this program\r\n'''

user_prompt = '''\r\nCommand keys:
    t      : Select a test to run: Effort, Velocity, Line Following
    u      : Queue tests
    r      : Run test
    k      : Kill (stop) motors
    s      : Stream data
    d      : Save data from latest run to CSV and plot PNG
    b      : Check battery voltage (prints to this terminal)
    c      : Calibrate sensors: IR sensors or IMU
    h      : Help / show this menu
    ctrl-c : Interrupt this program\r\n
'''

# # ************* HELPER FUNCTIONS **************

# Map an effort percentage (0-100) to the single-character key
def eff_to_key(eff):
    if eff == 100:
        return 'a'
    # assume efforts are multiples of 10
    digit = int(eff // 10)
    return str(digit)

# Map a single-character key to an effort percentage (0-100)
def key_to_eff(key):
    if key == 'a':
        return 100
    if key.isdigit():
        digit = int(key)
        return digit * 10
    return None

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


# # ************* MAIN PROGRAM **************

print("Begin Program:")

# Create 'runs' directory if it doesn't exist
try:
    os.makedirs('runs', exist_ok=True)
    # print("Output directory 'runs' is ready")
except Exception as e:
    print(f"Warning: Could not create 'runs' directory: {e}")

# Establish Bluetooth connection
try:
    ser = Serial('COM8', baudrate=115200, timeout=1)
except SerialException:
    print("Unable to connect to port")

print(user_prompt)

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

while True:
    try:
        # Check for key pressed
        if msvcrt.kbhit():
            try:
                # Prefer getwch which returns a Python string and handles wide chars
                key = msvcrt.getwch()
            except Exception:
                try:
                    key = msvcrt.getch().decode(errors='ignore')
                except Exception:
                    key = ''

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
                        eff_val = key_to_eff(key_in)
                        if eff_val is None:
                            # Try parse as integer percent
                            try:
                                perc = int(key_in)
                                if perc < 0 or perc > 100:
                                    raise ValueError()
                                eff_val = perc
                            except Exception:
                                print("Invalid effort entry. Use 0-9, 'a', or a number 0-100.")
                                continue
                        effort = eff_val
                        line = 'e' + eff_to_key(effort)
                    elif selected == 'v':
                        control_mode = 1
                        print("Selected Velocity mode")
                        gains = input("Enter setpoint, Kp, and Ki separated by a comma (e.g., 40,1.5,0.1): ")
                        try:
                            sp_str, kp_str, ki_str = gains.split(',')
                            setpoint = int(sp_str)
                            kp = float(kp_str)
                            ki = float(ki_str)
                            kp_int = int(kp * 100)
                            ki_int = int(ki * 100)
                            line = f'v{setpoint:04d}{kp_int:04d}{ki_int:04d}'
                        except ValueError:
                            print("Invalid format. Please enter two numbers separated by a comma.")

                    elif selected == 'l':
                        control_mode = 2
                        print("Selected Line Following mode")
                        gains = input("Enter Kp, Ki, K_line, and target separated by commas (e.g., 1.5,0.1,2.0,0.5): ")
                        try:
                            kp_str, ki_str, k_line_str, target_str = gains.split(',')
                            kp = float(kp_str)
                            ki = float(ki_str)
                            k_line = float(k_line_str)
                            target = float(target_str)
                            kp_int = int(kp * 100)
                            ki_int = int(ki * 100)
                            k_line_int = int(k_line * 100)
                            target_int = int(target * 100)
                            line = f'l{kp_int:04d}{ki_int:04d}{k_line_int:04d}{target_int:04d}'
                        except ValueError:
                            print("Invalid format. Please enter four numbers separated by commas.")
                    
                    elif selected == 'q':
                        print("Test selection cancelled.")
                        continue
                    else:
                        print("Invalid selection. Enter 'e', 'v', 'l', or 'q' to quit.")

                    # Send configuration line to Romi    
                    ser.write(line.encode())
                    print("Configuration sent to Romi.")
            
            if key == 'u':
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
                        if my_queue.is_empty():
                            print("Queue is empty.")
                        else:
                            print("Current queue:")
                            for i in my_queue.items:
                                # Format for test is (mode, param1, param2, ...)
                                print(i)
                    
                    elif selected == 'c':
                        while not my_queue.is_empty():
                            my_queue.dequeue()
                        ser.write(b'uc')  # send clear command to Romi
                        print("Queue cleared.")
                    elif selected == 'a':
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
                                my_queue.enqueue( ('effort', e) )
                            ser.write(b'ua')  # send add command to Romi
                            for i in my_queue.items:
                                cmd = 'e' + eff_to_key(i[1])
                                ser.write(cmd.encode())
                                sleep(0.1)
                        except ValueError as e:
                            print(f"Invalid format: {e}")
                    elif selected == 'q':
                        print("Queue operation cancelled.")
                        continue
                    
            if key == 'r':
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
                    print("End test. Stop motors")
                else:
                    print("Motors are already off")
            elif key == 's':
                if running:
                    print("Test is already running, cannot stream data now.")
                elif streaming:
                    print("Data is already streaming.")
                else:
                    # Start handshake with MCU
                    ok = start_stream_handshake(ser)
                    if not ok:
                        # Fallback: try legacy single-char start for compatibility
                        try:
                            ser.write(b's')
                        except Exception:
                            pass
                        print("Stream handshake failed; sent legacy 's' command (best-effort).")
                    else:
                        print("MCU acknowledged streaming request")

                    # Flush any initial bytes
                    if ser.in_waiting:
                        ser.read()

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
                        print("IR sensor calibration command sent to Romi.")
                        key = input("Place Romi on white surface and press any key to continue...")
                        ser.write(b'w')
                        print("White calibration done. Now place Romi on black surface.")
                        key = input("Place Romi on black surface and press any key to continue...")
                        ser.write(b'b')
                        print("Black calibration done.")
                    elif selected == '2':
                        print("IMU calibration command sent to Romi.")
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
                # Unified command: prompt user for single/latest or all runs and whether to save CSVs, plots, or both
                # Ensure base 'runs' and subfolders exist
                try:
                    os.makedirs('runs', exist_ok=True)
                    csv_dir = os.path.join('runs', 'csvs')
                    plots_dir = os.path.join('runs', 'plots')
                    os.makedirs(csv_dir, exist_ok=True)
                    os.makedirs(plots_dir, exist_ok=True)
                except Exception as e:
                    print(f"Warning: could not create runs subdirectories: {e}")

                if not runs:
                    print("No runs available to display or save.")
                else:
                    print("Select target to save/plot:")
                    print("  l : Latest run")
                    print("  a : All runs (combined plot and/or individual CSVs)")
                    print("  c : Cancel")
                    target = input("Enter choice (l/a/c): ").strip().lower()
                    if target == 'c' or target == '':
                        print("Operation cancelled.")
                        continue

                    print("Select output type:")
                    print("  p : plots only")
                    print("  c : csvs only")
                    print("  b : both plots and csvs")
                    print("  n : none (cancel)")
                    out = input("Enter choice (p/c/b/n): ").strip().lower()
                    if out == 'n' or out == '':
                        print("Operation cancelled.")
                        continue

                    save_plots = out in ('p', 'b')
                    save_csvs = out in ('c', 'b')

                    # Prompt user for which data channels to include
                    print("Select data to include (comma-separated):")
                    print("  1 : left motor velocity")
                    print("  2 : right motor velocity")
                    print("  3 : left motor position")
                    print("  4 : right motor position")
                    data_sel = input("Enter choices (e.g. 1,2,3): ").strip()
                    if not data_sel:
                        print("No data selections made. Operation cancelled.")
                        continue
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
                        print("No valid data selections found. Operation cancelled.")
                        continue

                    # Prepare a variable suffix for filenames
                    var_suffix = "_".join([sel_map[k][2] for k in chosen])

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

                        if save_plots:
                            # Save separate plot per selected channel for this single run
                            for k in chosen:
                                col, readable, code = sel_map[k]
                                try:
                                    plt.figure()
                                    plt.plot(df_clean["_time"], df_clean[col], label=label)
                                    plt.xlabel("Time, [ms]")
                                    plt.ylabel(readable)
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

                        if save_csvs:
                            # Save separate CSV per selected channel for this single run
                            for k in chosen:
                                col, readable, code = sel_map[k]
                                try:
                                    csv_name = os.path.join(csv_dir, base_name + "_" + code + ".csv")
                                    cols = ["_time", col]
                                    cols = [c for c in cols if c in df_clean.columns]
                                    df_clean[cols].to_csv(csv_name, index=False)
                                    print(f"Saved motor_data to {csv_name}")
                                except Exception as e:
                                    print(f"Failed to save CSV {code} for {run_name}: {e}")

                        print(user_prompt)

                    elif target == 'a':
                        # Optionally create individual CSVs and a combined plot
                        if save_plots:
                            plt.figure()

                        # For all-runs: for each selected channel, create combined plot across runs
                        for k in chosen:
                            col, readable, code = sel_map[k]
                            if save_plots:
                                plt.figure()
                            # Save per-run CSVs for this channel
                            for run_name, meta in runs.items():
                                df = meta["motor_data"]
                                df_clean, removed = clean_data(df, mode='all')
                                if removed:
                                    print(f"Removed {removed} all-zero rows from {run_name} before plotting/saving")

                                mode_char = normalize_mode(meta.get('mode', 'E'))
                                control_val = meta.get('control_val', 'unknown')

                                if mode_char == 'v':
                                    kp = meta.get('params', {}).get('kp', 0) if meta.get('params') else 0
                                    ki = meta.get('params', {}).get('ki', 0) if meta.get('params') else 0
                                    label = f"{run_name} (V) sp={control_val} Kp={kp:.2f} Ki={ki:.2f}"
                                    base_name = f"{run_name}_V_sp{control_val}_Kp{kp:.2f}_Ki{ki:.2f}"
                                else:
                                    label = f"{run_name} (E) eff={control_val}"
                                    base_name = f"{run_name}_E_eff{control_val}"

                                if save_plots:
                                    try:
                                        plt.plot(df_clean["_time"], df_clean[col], label=label)
                                    except Exception as e:
                                        print(f"Failed to add plot {readable} for {run_name}: {e}")

                                if save_csvs:
                                    try:
                                        csv_name = os.path.join(csv_dir, base_name + "_" + code + ".csv")
                                        cols = ["_time", col]
                                        cols = [c for c in cols if c in df_clean.columns]
                                        df_clean[cols].to_csv(csv_name, index=False)
                                        print(f"Saved motor_data to {csv_name}")
                                    except Exception as e:
                                        print(f"Failed to save CSV {code} for {run_name}: {e}")

                            if save_plots:
                                try:
                                    plt.xlabel("Time, [ms]")
                                    plt.ylabel(readable)
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
            
            elif key == 'h':
                # Print helpful prompt
                print(user_prompt)
            else:
                print(f"Unknown command '{key}'. Press 'h' for help.")

            # Clear any remaining input
            if ser.in_waiting:  
                ser.read()
        
        else:
            # Check for bytes waiting
            if ser.in_waiting:
                # print("\r\nWe are reading you loud and clear!")
                if running:
                    ch = ser.read().decode()
                    if ch == 'q':
                        print("Testing is done. Press 's' to stream data")
                        print(user_prompt)
                        running = False
                elif streaming:
                    if first:
                        mode_name = control_mode_dict[control_mode]
                        sample_size = 250
                        params = None

                        # Create new run
                        run_count += 1
                        run_name = f'run{run_count}'
                        runs[run_name] = create_run(effort, run_count, sample_size)
                        # Add in params if in velocity mode and line-following mode
                        if params:
                            runs[run_name]['params'] = params
                        runs[run_name]['mode'] = mode_name
                        print(f"Run {run_name} created in {mode_name} mode (size={sample_size})")

                        recv_line_num = 0
                        first = False
                    
                    else:
                        line = ser.readline().decode().strip()
                        if not line:
                            continue

                        if line.startswith("#END"):
                            print("Received end of stream marker from Romi. Hit 'd' to print data.")
                            # Acknowledge end of stream
                            try:
                                ser.write(b'ACK_END\n')
                            except Exception:
                                pass
                            first = True
                            streaming = False
                            sleep(0.5)
                            print(user_prompt)
                            continue

                        # Read line by line
                        try:
                            idx_str, time_s, left_pos, right_pos, left_vel, right_vel = line.split(',')
                            idx = int(idx_str)
                        except ValueError:
                            print(f"[Rejected] Received line #{recv_line_num}: could not parse the index. Raw: '{line}'")
                            recv_line_num += 1
                            continue

                        try:
                            # Check index bounds before writing to dataframe
                            size = runs.get(run_name, {}).get('size')
                            if size is None:
                                print(f"[Rejected] Line #{recv_line_num}: unknown run size for {run_name}")
                                recv_line_num += 1
                                continue
                            if idx < 0 or idx >= size:
                                print(f"[Rejected] Line #{recv_line_num}: index {idx} out of range for {run_name} (size={size})")
                                recv_line_num += 1
                                continue

                            runs[run_name]["motor_data"].loc[idx,"_time"] = float(time_s)
                            runs[run_name]["motor_data"].loc[idx, "_left_pos"] = float(left_pos)
                            runs[run_name]["motor_data"].loc[idx, "_right_pos"] = float(right_pos)
                            runs[run_name]["motor_data"].loc[idx, "_left_vel"] = float(left_vel)
                            runs[run_name]["motor_data"].loc[idx, "_right_vel"] = float(right_vel)
                        except ValueError:
                            print(f"[Rejected] Line #{recv_line_num}: numeric conversion failed. Raw: '{line}'")
                            recv_line_num += 1
                            continue
                        
                        recv_line_num += 1

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