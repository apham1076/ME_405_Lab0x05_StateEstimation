# Script to run on PC to run motor characterization test and perform data collection

# Last modified: 10-30-25 11:00am

# # ************* NOTES **************

# ************* TO-DO **************
# Debug driving mode
# Plot data for several runs
# Figure out why data is so jaggedy (message Charlie on Piazza)
# Debug data streaming: use START and ACK


from serial import Serial, SerialException
from time import sleep
import matplotlib.pyplot as plt
import msvcrt
import pandas as pd
import numpy as np
import os

key = ''                    # Stores value of key pressed as a string
running = False             # Set when motor testing is progress
streaming = False           # Set when data streaming is in progress
runs = {}                   # Dict to contain runs
run_count = 0               # Number of runs
override = False            # Set if user wants to overwrite existing run
_continue = False           # Second option, if user doesn't want to overwrite
question = False            # Set when option to override is presented
control_mode = 0            # 0 = effort mode, 1 = velocity mode, 2 = line follow mode
current_setpoint = 0        # Current velocity setpoint in rad/s
first = True
done = False
mode = 1                    # 1, 2, 3 = straight, pivot, arc

user_prompt = '''\r\nCommand keys:
    e      : Toggle mode: Effort, Velocity, and Line Following
    0-9,a  : Set effort (0-100%) for open-loop control [Effort mode]
    t      : Set setpoint (rad/s) for closed-loop control [Velocity mode]
    p      : Set proportional gain (Kp) for controller [Velocity mode]
    i      : Set integral gain (Ki) for controller [Velocity mode]
    g      : GO (start test)
    k      : Kill (stop) motors
    r      : Run automated sequence (0 to 100% by 10%)
    x      : Plot all runs' left velocity
    s      : Stream data
    m      : Toggle driving mode (straight/pivot/arc)
    d      : Save latest run to CSV and plot PNG
    c      : Run automated closed-loop test
    w      : IR white calibration (prints table from Nucleo USB REPL)
    b      : IR black calibration (prints table from Nucleo USB REPL)
    l      : Set IR line following gains (lf_kp, lf_ki)
    v      : Query battery voltage (prints to this terminal)
    h      : Help / show this menu
    ctrl-c : Interrupt this program\r\n'''

# Function to calibrate IR sensors
def calibrate_ir_sensors():
    # Placeholder for IR sensor calibration logic
    pass

# Function to create dictionary for storing data from one run
def create_run(control_val, size):
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

    return {"control_val": control_val, "size": size, "motor_data": df}

# Function to check if run has been created for a particular effort value
def has_eff(runs, eff_value):
    for run in runs.values():
        if run["eff"] == eff_value:
            return True    
    return False


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


# Map an effort percentage (0-100) to the single-character key expected by the Romi
def eff_to_key(eff):
    if eff == 100:
        return 'a'
    # assume efforts are multiples of 10
    digit = int(eff // 10)
    return str(digit)


# Automated run sequence: for each effort in 'efforts' send the effort key and 'g' to start
# then wait for the Romi to send a 'q' (test done), request streaming ('s'), and collect the
# streamed data into the global `runs` dict (same format as interactive streaming).
def auto_run_sequence(efforts):
    global ser, runs, run_count

    try:
        for eff in efforts:
            key = eff_to_key(eff)
            print(f"Starting automated test for {eff}% (key='{key}')")
            # set effort
            # clear any stale input, then write the key. For 100% ('a') give the Romi more time to process.
            try:
                # pyserial method to clear input buffer
                ser.reset_input_buffer()
            except Exception:
                pass
            ser.write(key.encode())
            # start test
            ser.write(b'g')
            print("Test started, waiting for completion signal from Romi...")

            # wait for 'q' from Romi that indicates the test is done
            while True:
                if ser.in_waiting:
                    ch = ser.read().decode(errors='ignore')
                    if ch == 'q':
                        print("Test complete signal received")
                        break
                else:
                    sleep(0.05)

            # request streaming
            ser.write(b's')
            sleep(0.1)

            # Read header line and parse flexibly for effort or velocity mode
            header = ser.readline().decode().strip()
            parts = header.split(',')
            try:
                if parts[0] in ('E', 'e'):
                    # Effort mode header: E,effort,size
                    if len(parts) < 3:
                        raise ValueError("not enough fields for effort header")
                    _mode_str = parts[0]
                    _control_val = float(parts[1])
                    _size = int(parts[2])
                    params = None
                elif parts[0] in ('V', 'v'):
                    # Velocity mode header: V,setpoint,kp,ki,size
                    if len(parts) < 5:
                        raise ValueError("not enough fields for velocity header")
                    _mode_str = parts[0]
                    _control_val = float(parts[1])
                    kp = float(parts[2])
                    ki = float(parts[3])
                    _size = int(parts[4])
                    params = {'kp': kp, 'ki': ki}
                elif parts[0] in ('L', 'l'):
                    # Line follow mode header: L,target,kp,ki,k_line,size
                    if len(parts) < 6:
                        raise ValueError("not enough fields for line follow header")
                    _mode_str = parts[0]
                    _control_val = float(parts[1])
                    kp = float(parts[2])
                    ki = float(parts[3])
                    k_line = float(parts[4])
                    _size = int(parts[5])
                    params = {'kp': kp, 'ki': ki, 'k_line': k_line}
                else:
                    raise ValueError("unknown mode in header")
            except Exception as e:
                print(f"Failed to parse header '{header}': {e}")
                continue

            run_count += 1
            run_name = f'run{run_count}'
            runs[run_name] = create_run(_control_val, _size)
            if params:
                runs[run_name]['params'] = params
            runs[run_name]['mode'] = _mode_str
            # Provide informative print depending on mode
            if _mode_str in ('E', 'e'):
                print(f"Created {run_name} (E) eff={_control_val}, size={_size}")
            elif _mode_str in ('V', 'v'):
                print(f"Created {run_name} (V) sp={_control_val} Kp={kp:.2f} Ki={ki:.2f} size={_size}")
            elif _mode_str in ('L', 'l'):
                print(f"Created {run_name} (L) target={_control_val} Kp={kp:.2f} Ki={ki:.2f} K_line={k_line:.2f} size={_size}")


            last_idx = -1
            recv_line_num = 0 # counts how many lines were received, regardless of idx
            while True:
                line = ser.readline().decode().strip()
                if not line:
                    continue

                # Check for end of stream marker
                if line.startswith("#END"):
                    print("Received end of stream marker from Romi.")
                    break

                try:
                    # Try to split and parse fields
                    idx_str, time_s, left_pos, right_pos, left_vel, right_vel = line.split(',')
                    idx = int(idx_str)
                except ValueError:
                    print(f"[Rejected] Received line #{recv_line_num}: could not parse the index. Raw: '{line}'")
                    recv_line_num += 1
                    continue

                # Try to convert valid numeric data
                try:
                    runs[run_name]["motor_data"].loc[idx, "_time"] = float(time_s)
                    runs[run_name]["motor_data"].loc[idx, "_left_pos"] = float(left_pos)
                    runs[run_name]["motor_data"].loc[idx, "_right_pos"] = float(right_pos)
                    runs[run_name]["motor_data"].loc[idx, "_left_vel"] = float(left_vel)
                    runs[run_name]["motor_data"].loc[idx, "_right_vel"] = float(right_vel)
                except ValueError:
                    print(f"[Rejected] Line #{recv_line_num}: numeric conversion failed. Raw: '{line}'")
                    recv_line_num += 1
                    continue

                last_idx = idx
                recv_line_num += 1

                # Stop if we've received all expected samples
                if last_idx >= (_size - 1):
                    print(f"All { _size } samples received for {run_name}.")
                    break

            if run_count == 10:
                print(f"Automated test is finished ({run_count} runs completed). Hit 'x' to plot data.")


    except KeyboardInterrupt:
        print("\nAutomated sequence interrupted by user (Ctrl-C). Sending kill and closing serial port.")
        try:
            if ser and hasattr(ser, 'is_open') and ser.is_open:
                try:
                    ser.write(b'k')
                except Exception:
                    pass
                try:
                    ser.close()
                except Exception as e:
                    print(f"Error closing serial port: {e}")
                else:
                    print("Serial port closed.")
        finally:
            return

# Function to run an automated closed-loop test
def run_closed_loop_test():
    global ser, runs, run_count, current_setpoint, control_mode, streaming, running

    if control_mode == 0:
        ser.write(b'e')
        sleep(0.1)
    elif control_mode == 2:
        ser.write(b'e')
        sleep(0.1)
        ser.write(b'e')
        sleep(0.1)
    
    print("Switching to velocity mode for closed-loop test...")
    control_mode = 1


    try:

        # Get test parameters from user
        kp = float(input("Enter proportional gain (Kp): "))
        ki = float(input("Enter integral gain (Ki): "))
        setpoint = int(input("Enter velocity setpoint (rad/s): "))

        # Send Kp
        kp_int = int(kp * 100)
        cmd = f"p{abs(kp_int):04d}"
        ser.write(cmd.encode())
        sleep(0.1)

        # Send Ki
        ki_int = int(ki * 100)
        cmd = f"i{abs(ki_int):04d}"
        ser.write(cmd.encode())
        sleep(0.1)

        # Send setpoint
        cmd = 'y' if setpoint >= 0 else 'z'
        cmd += f"{abs(setpoint):04d}"
        ser.write(cmd.encode())
        current_setpoint = setpoint
        sleep(0.1)

        print(f"\nTest parameters set:")
        print(f"Kp: {kp}")
        print(f"Ki: {ki}")
        print(f"Setpoint: {setpoint} rad/s")
        input("Press Enter to start the test...")

        # Start test
        print("Starting test...")
        ser.write(b'g')
        running = True

        # Wait for test completion
        while running:
            if ser.in_waiting:
                ch = ser.read().decode()
                if ch == 'q':
                    print("Test complete. Starting data stream...")
                    running = False
            sleep(0.05)

        # Stream data
        ser.write(b's')
        streaming = True

    except ValueError:
        print("Invalid input. Test aborted.")
        return
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
        if ser and ser.is_open:
            ser.write(b'k')  # Kill motors
        return

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

# ser = Serial('COM8', baudrate=115200, timeout=1)

print("Begin Program:")
print(user_prompt)

while True:
    try:
        # Check for key pressed
        if msvcrt.kbhit():
            key = msvcrt.getch().decode()
            # print(f"{key} was pressed")
            if key.isdigit() or key == 'a':
                if running:
                    print("Cannot set effort at this time. Try again when test is finished")
                elif streaming:
                    print("Cannot set effort at this time. Data streaming in progress")
                else:
                    ser.write(key.encode())
                    if key == 'a':
                        print(f"Effort value set to 100%. Enter 'g' to begin test.")
                    else:
                        print(f"Effort value set to {key}0%. Enter 'g' to begin test.")
            elif key == 'g':
                if running:
                    print("Test is already running")
                elif streaming:
                    print("Data is streaming.")
                else:
                    print("Starting test...")
                    running = True
                    ser.write(b'g')
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
                    # Tell Romi to stream data to PC
                    ser.write(b's')

                    # Flush serial port
                    if ser.in_waiting:
                        ser.read()

                    print("Data streaming to PC...")
                    streaming = True
            
            elif key == 'm':
                if not running and not streaming:
                    print("Toggle mode")
                    ser.write(b'm')
                    if mode == 1:
                        print("Romi will drive in a straight line.")
                        mode = 2
                    elif mode == 2:
                        print("Romi will pivot in place.")
                        mode = 3
                    else:
                        mode = 1
                        print("Romi will follow an arc.")

            elif key == 'e':
                if running:
                    print("Cannot change control mode while test is running")
                elif streaming:
                    print("Cannot change control mode while streaming")
                else:
                    ser.write(b'e')
                    control_mode = (control_mode + 1) % 3
                    mode_str = "effort" if control_mode == 0 else "velocity" if control_mode == 1 else "line following"
                    print(f"Switched to {mode_str} control mode")

            elif key == 't':
                if running:
                    print("Cannot set velocity setpoint while test is running")
                elif streaming:
                    print("Cannot set velocity setpoint while streaming")
                elif control_mode != 1:
                    print("Must be in velocity mode to set setpoint")
                else:
                    try:
                        # Get setpoint from user
                        setpoint = input("Enter velocity setpoint (rad/s): ")
                        setpoint = int(setpoint)
                        # Format: 'yXXXX' for positive, 'zXXXX' for negative
                        cmd = 'y' if setpoint >= 0 else 'z'  # 'y' for positive, 'z' for negative
                        cmd += f"{abs(setpoint):04d}"  # Always send magnitude as 4 digits
                        ser.write(cmd.encode())
                        current_setpoint = setpoint
                        sign_str = "+" if setpoint >= 0 else "-"
                        print(f"Velocity setpoint set to {sign_str}{abs(setpoint)} rad/s")
                    except ValueError:
                        print("Invalid input. Please enter an integer.")

            elif key == 'i':
                if running:
                    print("Cannot set Ki while test is running")
                elif streaming:
                    print("Cannot set Ki while streaming")
                else:
                    try:
                        # Get Ki from user
                        ki = input("Enter integral gain (Ki): ")
                        ki = float(ki)
                        # Send Ki to Romi - format: 'iXXXX' where XXXX is Ki*100
                        ki_int = int(ki * 100)  # Scale up by 100 to send as integer
                        cmd = f"i{abs(ki_int):04d}"
                        ser.write(cmd.encode())
                        print(f"Integral gain set to {ki}")
                    except ValueError:
                        print("Invalid input. Please enter a number.")

            elif key == 'p':
                if running:
                    print("Cannot set Kp while test is running")
                elif streaming:
                    print("Cannot set Kp while streaming")
                else:
                    try:
                        # Get Kp from user
                        kp = input("Enter proportional gain (Kp): ")
                        kp = float(kp)
                        # Send Kp to Romi - format: 'pXXXX' where XXXX is Kp*100
                        kp_int = int(kp * 100)  # Scale up by 100 to send as integer
                        cmd = f"p{abs(kp_int):04d}"
                        ser.write(cmd.encode())
                        print(f"Proportional gain set to {kp}")
                    except ValueError:
                        print("Invalid input. Please enter a number.")
            
            elif key == 'w':
                # IR white calibration (Nucleo will print the table to USB REPL;
                # some prints may also echo here if your firmware routes them)
                if running or streaming:
                    print("Cannot calibrate while running/streaming")
                else:
                    ser.write(b'w')
                    print("Sent IR WHITE calibration command. Check USB PuTTY for the calibration table.")

            elif key == 'b':
                # IR black calibration
                if running or streaming:
                    print("Cannot calibrate while running/streaming")
                else:
                    ser.write(b'b')
                    print("Sent IR BLACK calibration command. Check USB PuTTY for the calibration table.")

            elif key == 'l':
                # Set gains for line-following
                if running:
                    print("Cannot set line-following gains while test is running")
                elif streaming:
                    print("Cannot set line-following gains while streaming")
                else:
                    try:
                        # Get line-following gains from user
                        kp = input("Enter closed-loop proportional gain (Kp): ")
                        kp = float(kp)
                        ki = input("Enter closed-loop integral gain (Ki): ")
                        ki = float(ki)
                        k_line = input("Enter line-following proportional gain (K_line): ")
                        k_line = float(k_line)
                        v_target = input("Enter line-following target: ")
                        v_target = float(v_target)
                        # Send line-following gains to Romi - format: 'lppppiiiilllltttt' where pppp is Kp*100, iiii is Ki*100, llll is K_line*100, and tttt is target
                        kp_int = int(kp * 100)
                        ki_int = int(ki * 100)
                        k_line_int = int(k_line * 100)
                        v_target_int = int(v_target * 100)
                        cmd = f"l{kp_int:04d}{ki_int:04d}{k_line_int:04d}{v_target_int:04d}"
                        ser.write(cmd.encode())
                        print(f"Line-following gains set to Kp={kp}, Ki={ki}, K_line={k_line}, Setpoint={v_target}")
                    except ValueError:
                        print("Invalid input. Please enter numbers for gains.")

            elif key == 'v':
                # Request battery voltage (firmware will respond with a number and newline)
                if streaming:
                    print("Battery query deferred (currently streaming)")
                else:
                    ser.write(b'v')
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
                # Plot and save the latest run's motor_data to CSV
                if run_count == 0:
                    print("No runs available to display or save.")
                else:
                    run_name = f'run{run_count}'
                    df = runs[run_name]["motor_data"]
                    # Clean data: remove zeros except leading zeros
                    df_clean, removed = clean_data(df, mode='all')
                    if removed:
                        print(f"Removed {removed} all-zero rows from {run_name} before plotting/saving")
                    x = df_clean["_time"]
                    y = df_clean["_left_vel"]
                    plt.plot(x, y)
                    plt.xlabel("Time, [ms]")
                    plt.ylabel("Left velocity, [rad/s]")

                    # Determine whether this run was in effort or velocity mode and build filenames accordingly
                    run_mode = runs[run_name].get('mode', 'E')  # 'E' or 'V'
                    control_val = runs[run_name].get('control_val', 'unknown')

                    if run_mode in ('V', 'v'):
                        params = runs[run_name].get('params', {})
                        kp = params.get('kp', 0.0)*100
                        ki = params.get('ki', 0.0)*100
                        label = f"SP={control_val} Kp={kp:.2f} Ki={ki:.2f}"
                        fig_name = f"runs/{run_name}_V_sp{control_val}_Kp{kp:.2f}_Ki{ki:.2f}_left_vel.png"
                        csv_name = f"runs/{run_name}_V_sp{control_val}_Kp{kp:.2f}_Ki{ki:.2f}.csv"
                    else:
                        # Effort mode
                        label = f"Effort={control_val}"
                        fig_name = f"runs/{run_name}_E_eff{control_val}_left_vel.png"
                        csv_name = f"runs/{run_name}_E_eff{control_val}.csv"

                    # Add legend with the control parameters
                    try:
                        plt.legend([label])
                    except Exception:
                        pass

                    # Save figure and CSV
                    plt.savefig(fig_name)
                    plt.close()

                    try:
                        df_clean.to_csv(csv_name, index=False)
                        print(f"Saved motor_data to {csv_name}")
                    except Exception as e:
                        print(f"Failed to save CSV {csv_name}: {e}")

                    print(f"Saved figure to {fig_name}")
                    print(user_prompt)

            elif key == 'r':
                # Run automated sequence for efforts 0..100 step 10
                print("Starting automated batch: efforts 0 to 100 by 10")
                efforts = list(range(0, 91, 10))
                auto_run_sequence(efforts)

            elif key == 'x':
                # Plot all runs at once
                if not runs:
                    print("No runs available to plot.")
                else:
                    plt.figure()
                    for run_name, meta in runs.items():
                        df = meta["motor_data"]
                        # Clean data: remove zeros except leading zeros
                        df_clean, removed = clean_data(df, mode='all')
                        if removed:
                            print(f"Removed {removed} all-zero rows from {run_name} before plotting")
                            
                        mode = meta.get('mode', 'E')  # Default to effort mode for backward compatibility
                        control_val = meta.get('control_val', 'unknown')
                        
                        if mode == 'V':  # Velocity mode
                            kp = meta.get('params', {}).get('kp', 0)
                            ki = meta.get('params', {}).get('ki', 0)
                            label = f"{run_name} (V) sp={control_val} Kp={kp:.2f} Ki={ki:.2f}"
                        else:  # Effort mode
                            label = f"{run_name} (E) eff={control_val}"
                            
                        plt.plot(df_clean["_time"], df_clean["_left_vel"], label=label)

                    plt.xlabel("Time, [ms]")
                    plt.ylabel("Left velocity, [rad/s]")
                    plt.legend()
                    # instead of plt.show()
                    plt.savefig(f"runs/all_runs_velocity_response.png")
                    plt.close()

                    print("Saved plot of all runs to 'runs/all_runs_velocity_response.png'")
                    print(user_prompt)
            
            elif key == 'c':
                if running:
                    print("Cannot start automated test while test is running")
                elif streaming:
                    print("Cannot start automated test while streaming")
                else:
                    run_closed_loop_test()
            
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
                    # print("ve r streeming")
                    if first:
                        # Read and parse header line (mode, control, params, size)
                        header_line = ser.readline().decode().strip()
                        if not header_line:
                            continue
                        parts = header_line.split(',')
                        hdr_mode = parts[0].upper() if parts else 'E'  # 'E' for effort, 'V' for velocity, 'L' for line follow

                        try:
                            if hdr_mode == 'E':
                                control_val = float(parts[1])  # effort value
                                size = int(parts[2])
                                params = None
                            elif hdr_mode == 'V':  # Velocity mode
                                control_val = float(parts[1])  # setpoint value
                                kp = float(parts[2])
                                ki = float(parts[3])
                                size = int(parts[4])
                                params = {'kp': kp, 'ki': ki}
                            elif hdr_mode == 'L':  # Line follow mode
                                control_val = float(parts[1])  # target value
                                kp = float(parts[2])
                                ki = float(parts[3])
                                k_line = float(parts[4])
                                size = int(parts[5])
                                params = {'kp': kp, 'ki': ki, 'k_line': k_line}
                            else:
                                print(f"Unknown header mode: '{header_line}'")
                                continue
                        except Exception as e:
                            print(f"Failed to parse header '{header_line}': {e}")
                            continue

                        # Create new run
                        run_count += 1
                        run_name = f'run{run_count}'
                        runs[run_name] = create_run(control_val, size)
                        if params:
                            runs[run_name]['params'] = params
                        runs[run_name]['mode'] = hdr_mode
                        print(f"Run {run_name} created in {hdr_mode} mode (size={size})")

                        recv_line_num = 0
                        first = False
                    
                    else:
                        line = ser.readline().decode().strip()
                        if not line:
                            continue

                        if line.startswith("#END"):
                            print("Received end of stream marker from Romi. Hit 'd' to print data.")
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

if ser.is_open:
    ser.write(b'k')         # Kill motors
ser.close()                 # Close port
print("Program interrupted")