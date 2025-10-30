# Script to run on PC to run motor characterization test and perform data collection

# Last modified: 10-25-25 7:30pm

# # ************* NOTES **************

# ************* TO-DO **************
# Debug driving mode
# Plot data for several runs
# Figure out why data is so jaggedy (message Charlie on Piazza)
# Debug data streaming: use START and ACK


from serial import Serial, SerialException
from time import sleep, time
import matplotlib.pyplot as plt
import msvcrt
import pandas as pd


# Automated velocity-sweep test
def auto_velocity_sweep():
    """Sweep Kp over a range for a single setpoint and fixed Ki.
    Prompts: setpoint (int ticks/s), kp_min, kp_max, kp_step (floats), ki (float).
    Runs tests, collects streamed data into `runs`, saves per-run CSVs and
    creates an overlay plot saved to the runs folder.
    """
    global ser, runs, run_count, control_mode

    try:
        setpoint = int(input("Enter velocity setpoint (ticks/s): "))
        kp_min = float(input("Enter Kp minimum: "))
        kp_max = float(input("Enter Kp maximum: "))
        kp_step = float(input("Enter Kp step: "))
        ki = float(input("Enter Ki (fixed for sweep): "))
    except ValueError:
        print("Invalid input. Aborting sweep.")
        return

    if kp_step == 0:
        print("kp_step cannot be zero")
        return

    # Build kp list
    kp_values = []
    v = kp_min
    if kp_step > 0:
        while v <= kp_max + 1e-12:
            kp_values.append(round(v, 6))
            v += kp_step
    else:
        while v >= kp_max - 1e-12:
            kp_values.append(round(v, 6))
            v += kp_step

    if not kp_values:
        print("No Kp values generated for given range/step")
        return

    print(f"Running sweep with {len(kp_values)} Kp values: {kp_values}")

    # Ensure MCU in velocity mode
    if control_mode:
        ser.write(b'e')
        control_mode = False
        sleep(0.15)

    sweep_run_names = []
    ts = int(time())

    try:
        for kp in kp_values:
            print(f"Starting test for Kp={kp}, Ki={ki}, SP={setpoint}")

            try:
                ser.reset_input_buffer()
            except Exception:
                pass

            # Send Kp and Ki
            kp_int = int(abs(kp) * 100)
            ser.write(f"p{kp_int:04d}".encode())
            sleep(0.08)
            ki_int = int(abs(ki) * 100)
            ser.write(f"i{ki_int:04d}".encode())
            sleep(0.08)

            # Send setpoint
            sp_cmd = 'y' if setpoint >= 0 else 'z'
            sp_cmd += f"{abs(setpoint):04d}"
            ser.write(sp_cmd.encode())
            sleep(0.08)

            # Start
            ser.write(b'g')

            # Wait for 'q'
            while True:
                if ser.in_waiting:
                    ch = ser.read().decode(errors='ignore')
                    if ch == 'q':
                        print("Test complete signal received")
                        break
                else:
                    sleep(0.05)

            # Stream
            ser.write(b's')
            sleep(0.12)

            header = ser.readline().decode().strip()
            parts = header.split(',')
            try:
                if parts[0] in ('V', 'v'):
                    if len(parts) < 5:
                        raise ValueError("not enough fields for velocity header")
                    _mode = parts[0]
                    _control_val = float(parts[1])
                    kp_hdr = float(parts[2])
                    ki_hdr = float(parts[3])
                    _size = int(parts[4])
                    params = {'kp': kp_hdr, 'ki': ki_hdr}
                elif parts[0] in ('E', 'e'):
                    if len(parts) < 3:
                        raise ValueError("not enough fields for effort header")
                    _mode = parts[0]
                    _control_val = float(parts[1])
                    _size = int(parts[2])
                    params = None
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
            runs[run_name]['mode'] = _mode
            print(f"Created {run_name} (mode={_mode}, size={_size})")

            line_num = 0
            while line_num <= _size - 1:
                line = ser.readline().decode().strip()
                if not line:
                    continue
                try:
                    time_s, left_pos, right_pos, left_vel, right_vel = line.split(',')
                    runs[run_name]['motor_data'].loc[line_num, '_time'] = float(time_s)
                    runs[run_name]['motor_data'].loc[line_num, '_left_pos'] = float(left_pos)
                    runs[run_name]['motor_data'].loc[line_num, '_right_pos'] = float(right_pos)
                    runs[run_name]['motor_data'].loc[line_num, '_left_vel'] = float(left_vel)
                    runs[run_name]['motor_data'].loc[line_num, '_right_vel'] = float(right_vel)
                    line_num += 1
                except ValueError:
                    print(f"Line {line_num} rejected. Contents: {line}")
                    line_num += 1

            print(f"Finished streaming for {run_name}")
            sweep_run_names.append(run_name)

        # Overlay plot
        if sweep_run_names:
            plt.figure()
            for rn in sweep_run_names:
                meta = runs[rn]
                df = meta['motor_data']
                df_clean, removed = clean_data(df, mode='all')
                if removed:
                    print(f"Removed {removed} all-zero rows from {rn} before plotting")
                mode_char = meta.get('mode', 'E')
                control_val = meta.get('control_val', 'unknown')
                if mode_char in ('V', 'v'):
                    kp_v = meta.get('params', {}).get('kp', 0)
                    ki_v = meta.get('params', {}).get('ki', 0)
                    label = f"Kp={kp_v:.2f} Ki={ki_v:.2f} SP={control_val}"
                else:
                    label = f"Eff={control_val}"
                plt.plot(df_clean['_time'], df_clean['_left_vel'], label=label)

            plt.xlabel('Time, [ms]')
            plt.ylabel('Left velocity, [rad/s]')
            plt.legend()
            overlay_name = f"runs/velocity_sweep_{ts}_overlay.png"
            plt.savefig(overlay_name)
            plt.close()
            print(f"Saved overlay plot to {overlay_name}")

    except KeyboardInterrupt:
        print("\nSweep interrupted by user")
        return


# Function to run an automated closed-loop test
def run_closed_loop_test():
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
                print(f"Created {run_name} (eff={_control_val}, size={_size})")
            else:
                print(f"Created {run_name} (V) sp={_control_val} Kp={kp:.2f} Ki={ki:.2f} size={_size}")

            line_num = 0
            while line_num <= _size - 10:
                line = ser.readline().decode().strip()
                if not line:
                    continue
                try:
                    time_s, left_pos, right_pos, left_vel, right_vel = line.split(',')
                    runs[run_name]["motor_data"].loc[line_num, "_time"] = float(time_s)
                    runs[run_name]["motor_data"].loc[line_num, "_left_pos"] = float(left_pos)
                    runs[run_name]["motor_data"].loc[line_num, "_right_pos"] = float(right_pos)
                    runs[run_name]["motor_data"].loc[line_num, "_left_vel"] = float(left_vel)
                    runs[run_name]["motor_data"].loc[line_num, "_right_vel"] = float(right_vel)
                    line_num += 1
                except ValueError:
                    print(f"Line {line_num} rejected. Contents: {line}")
                    line_num += 1

            print(f"Finished streaming for {run_name}")
            
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
    # Placeholder for future auto_velocity_sweep function
    pass

    global ser, runs, run_count, current_setpoint, control_mode, streaming, running

    if control_mode:
        print("Switching to velocity mode for closed-loop test...")
        ser.write(b'e')
        control_mode = False
        sleep(0.1)

    try:
        # Ensure we are in velocity mode
        # if not control_mode:
        #     print("Switching to velocity mode...")
        #     ser.write(b'e')
        #     control_mode = False
        #     sleep(0.1)

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

                # elif not question:
                #     # Check if run has already been made
                #     if has_eff(runs, eff):
                #             print(f"A run has already been created for an effort value of {eff}")
                #             print("Do you want to override it? Override: 'o', Do nothing: 'p'")
                #             question = True
                #     else:
                #         # Create the dict, no run with this effort has been made yet
                #         run_count += 1
                #         run_name = f'run{run_count}'
                #         runs[run_name] = create_run(eff, size)
            
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
                    control_mode = not control_mode
                    mode_str = "effort" if control_mode else "velocity"
                    print(f"Switched to {mode_str} control mode")

            elif key == 't':
                if running:
                    print("Cannot set velocity setpoint while test is running")
                elif streaming:
                    print("Cannot set velocity setpoint while streaming")
                elif control_mode:
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
                        kp = params.get('kp', 0.0)
                        ki = params.get('ki', 0.0)
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
                        # Read and parse header line
                        header = ser.readline().decode().strip().split(",")
                        mode = header[0]  # 'E' for effort, 'V' for velocity
                        
                        if mode == 'E':
                            control_val = float(header[1])  # effort value
                            size = int(header[2])
                            params = None
                        else:  # Velocity mode
                            control_val = float(header[1])  # setpoint value
                            kp = float(header[2])
                            ki = float(header[3])
                            size = int(header[4])
                            params = {'kp': kp, 'ki': ki}
                        
                        # Create new run
                        run_count += 1
                        run_name = f'run{run_count}'
                        runs[run_name] = create_run(control_val, size)
                        if params:
                            runs[run_name]['params'] = params
                        runs[run_name]['mode'] = mode
                        print(f"Run {run_name} created in {mode} mode")
                        # print(size)
                        line_num = 0
                        first = False
                    # elif done:
                    #     first = True
                    #     streaming = False
                    #     done = False
                    else:
                        if line_num <= size - 4:
                            # Read line by line
                            try:
                                spam = ser.readline().decode().strip().split(",")
                                time, left_pos, right_pos, left_vel, right_vel = spam

                                runs[run_name]["motor_data"].loc[line_num,"_time"] = float(time)
                                # runs[run_name]["motor_data"].loc[line_num,"_control"] = float(control)
                                runs[run_name]["motor_data"].loc[line_num, "_left_pos"] = float(left_pos)
                                runs[run_name]["motor_data"].loc[line_num, "_right_pos"] = float(right_pos)
                                runs[run_name]["motor_data"].loc[line_num, "_left_vel"] = float(left_vel)
                                runs[run_name]["motor_data"].loc[line_num, "_right_vel"] = float(right_vel)
                                line_num += 1
                                # print(line_num)
                            except ValueError:
                                print(f"Line {line_num} rejected. Contents: {spam}")
                                line_num +=1

                        else:
                            print("Data streaming complete. Hit 'd' to print data.")
                            first = True
                            streaming = False
                            sleep(0.5)
                            print(user_prompt)
                else:
                    pass

                    # if not question:
                    #     # Ask user if want to override existing run
                    #     if has_eff(runs, eff):
                    #         print(f"A run has already been created for an effort value of {eff}")
                    #         print("Do you want to override it? Override: 'o', Do nothing: 'p'")
                    #         question = True
                    #     else:
                    #         # Create the dict, no run with this effort has been made yet
                    #         run_count += 1
                    #         run_name = f'run{run_count}'
                    #         runs[run_name] = create_run(eff, size)
                    # else:
                    #     # Create dict object to store data for run
                    #     # First, check if run with this effort has already been created
                    #     if override:
                    #         # Delete run and create new dictionary, do not need to change run_count
                    #         print(f"Run {run_count} was overwritten.") 
                    #         run_name = f'run{run_count}'
                    #         del runs[run_name]
                    #     elif _continue:
                    #         # Do nothing, prompt user with help menu again
                    #         print(user_prompt)

                    #     streaming = False
                    #     override = False
                    #     _continue = False
                    #     question = False
                    #     pass

            else:
                pass

    except KeyboardInterrupt:
        break

if ser.is_open:
    ser.write(b'k')         # Kill motors
ser.close()                 # Close port
print("Program interrupted")