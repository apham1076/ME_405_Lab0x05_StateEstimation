import pandas as pd
import matplotlib.pyplot as plt
import os

# === Folder location ===
folder = r"C:\Users\mikey\Documents\CAL POLY\CLASSES\ME 405\Lab\Lab 0x05"

# ==========================================================================================
# === Helper function to generate the 3-subplot figure for any CSV ===
# ==========================================================================================
def make_plots(filename, label_prefix, dynamic_yaw=False):
    filepath = os.path.join(folder, filename)
    df = pd.read_csv(filepath)

    # === Extract columns ===
    t_meas = df["time (ms)"]
    s_meas = df["s (mm)"]
    yaw_meas = df["yaw (deg)"]
    velL_meas = df["vel_L (mm/s)"]

    t_obsv = df["obsv_time (ms)"]
    s_obsv = df["obsv_s (mm)"]
    yaw_obsv = df["obsv_yaw (deg)"]
    velL_obsv = df["obsv_vel_L (mm/s)"]

    # === Limit to 0–2000 ms ===
    mask_meas = t_meas <= 2000
    mask_obsv = t_obsv <= 2000

    # Measured windowed data
    t_meas_w = t_meas[mask_meas]
    s_meas_w = s_meas[mask_meas]
    yaw_meas_w = yaw_meas[mask_meas]
    velL_meas_w = velL_meas[mask_meas]

    # Observer windowed data
    t_obsv_w = t_obsv[mask_obsv]
    s_obsv_w = s_obsv[mask_obsv]
    yaw_obsv_w = yaw_obsv[mask_obsv]
    velL_obsv_w = velL_obsv[mask_obsv]

    # ======================================================================================
    # === Create figure with 3 stacked subplots ===
    # ======================================================================================
    fig, axes = plt.subplots(3, 1, figsize=(9, 12))

    # ------------------ Arc Length ------------------
    axes[0].plot(t_meas_w, s_meas_w, 'r-', linewidth=2, label="Measured Arc Length")
    axes[0].plot(t_obsv_w, s_obsv_w, 'k--', linewidth=2, label="Estimated Arc Length")
    axes[0].set_ylabel("Arc Length (mm)", fontsize=12)
    axes[0].set_title(f"{label_prefix}: Measured vs Estimated Arc Length", fontsize=14)
    axes[0].legend(loc="lower right")
    axes[0].grid(True, linestyle="--", alpha=0.4)

    # ------------------ Yaw ------------------
    axes[1].plot(t_meas_w, yaw_meas_w, 'r-', linewidth=2, label="Measured Yaw")
    axes[1].plot(t_obsv_w, yaw_obsv_w, 'k--', linewidth=2, label="Estimated Yaw")
    axes[1].set_ylabel("Yaw (deg)", fontsize=12)
    axes[1].set_title(f"{label_prefix}: Measured vs Estimated Yaw", fontsize=14)

    if dynamic_yaw:
        # dynamic bounds with padding
        ymin = min(yaw_meas_w.min(), yaw_obsv_w.min())
        ymax = max(yaw_meas_w.max(), yaw_obsv_w.max())
        padding = 0.1 * (ymax - ymin if ymax != ymin else 1)
        axes[1].set_ylim([ymin - padding, ymax + padding])
    else:
        axes[1].set_ylim([-20, 20])  # static bounds for straight test

    axes[1].legend(loc="lower right")
    axes[1].grid(True, linestyle="--", alpha=0.4)

    # ------------------ Left Wheel Velocity ------------------
    axes[2].plot(t_meas_w, velL_meas_w, 'r-', linewidth=2, label="Measured Left Velocity")
    axes[2].plot(t_obsv_w, velL_obsv_w, 'k--', linewidth=2, label="Estimated Left Velocity")
    axes[2].set_xlabel("Time (ms)", fontsize=12)
    axes[2].set_ylabel("Velocity (mm/s)", fontsize=12)
    axes[2].set_title(f"{label_prefix}: Measured vs Estimated Left Wheel Velocity", fontsize=14)
    axes[2].legend(loc="lower right")
    axes[2].grid(True, linestyle="--", alpha=0.4)

    plt.tight_layout()

    # === Save SVG ===
    svg_name = os.path.splitext(filename)[0] + "_combined_plots.svg"
    save_path = os.path.join(folder, svg_name)
    plt.savefig(save_path, format="svg")
    print(f"Saved: {save_path}")

    plt.show()

# ==========================================================================================
# === Generate plots for all three test runs ===
# ==========================================================================================

# 1) Straight test (fixed yaw bounds)
make_plots("run1_E_50_STR - Copy.csv", label_prefix="Straight Test", dynamic_yaw=False)

# 2) Pirouette test (dynamic yaw bounds)
make_plots("run2_E_50_PV - Copy.csv", label_prefix="Pirouette Test", dynamic_yaw=True)

# 3) Arc test (dynamic yaw bounds recommended)
make_plots("run3_E_50_ARC - Copy.csv", label_prefix="Arc Test", dynamic_yaw=True)