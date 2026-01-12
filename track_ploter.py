import pandas as pd
import matplotlib.pyplot as plt
import sys

# -----------------------------
# Usage
# -----------------------------
# python plot_slice_paths.py path/to/slice.csv
#
# Supports:
# 1) Single participant (long format)
# 2) All participants (wide format)
# -----------------------------

if len(sys.argv) != 2:
    print("Usage: python plot_slice_paths.py <slice_csv>")
    sys.exit(1)

csv_file = sys.argv[1]
df = pd.read_csv(csv_file)

fig, ax = plt.subplots()

# -----------------------------
# Detect format
# -----------------------------
is_wide = all(col in df.columns for col in ["x1", "y1", "x5", "y5"])

# -----------------------------
# Plot robot path
# -----------------------------
ax.plot(
    df["robot_x"],
    df["robot_y"],
    "r-",
    linewidth=2,
    label="Robot"
)

# Mark robot start/end
ax.plot(df["robot_x"].iloc[0], df["robot_y"].iloc[0], "ro", label="Robot start")
ax.plot(df["robot_x"].iloc[-1], df["robot_y"].iloc[-1], "rx", label="Robot end")

# -----------------------------
# Plot participants
# -----------------------------
if is_wide:
    # -------- WIDE FORMAT --------
    for i in range(1, 6):
        ax.plot(
            df[f"x{i}"],
            df[f"y{i}"],
            linewidth=2,
            label=f"Participant {i}"
        )

        ax.plot(df[f"x{i}"].iloc[0], df[f"y{i}"].iloc[0], "o")
        ax.plot(df[f"x{i}"].iloc[-1], df[f"y{i}"].iloc[-1], "x")

else:
    # -------- LONG FORMAT --------
    for col in sorted(df["column"].unique()):
        sub = df[df["column"] == col]

        ax.plot(
            sub["x"],
            sub["y"],
            linewidth=2,
            label=f"Participant {col}"
        )

        ax.plot(sub["x"].iloc[0], sub["y"].iloc[0], "o")
        ax.plot(sub["x"].iloc[-1], sub["y"].iloc[-1], "x")

# -----------------------------
# Plot styling
# -----------------------------
ax.set_aspect("equal")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title("Robot and Participant Trajectories")
ax.legend()
ax.grid(True)

plt.show()
