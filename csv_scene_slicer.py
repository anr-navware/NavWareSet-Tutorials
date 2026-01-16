import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import json
import matplotlib as mpl
import os


# -----------------------------
# Disable Matplotlib keybindings
# -----------------------------
mpl.rcParams['keymap.save'] = []
mpl.rcParams['keymap.fullscreen'] = []
mpl.rcParams['keymap.pan'] = []
mpl.rcParams['keymap.zoom'] = []
mpl.rcParams['keymap.quit'] = []

scene = 52

# -----------------------------
# Load CSV
# -----------------------------
if scene < 10:
    df = pd.read_csv(f"all_poses/0{scene}_poses/{scene}_robot_and_participants.csv")
    OUTPUT_DIR = f"all_poses/0{scene}_poses/"   
else:
    df = pd.read_csv(f"all_poses/{scene}_poses/{scene}_robot_and_participants.csv")
    OUTPUT_DIR = f"all_poses/{scene}_poses/"

robot_x = df["robot_x"].values
robot_y = df["robot_y"].values
robot_yaw = df["robot_yaw_rad"].values
timestamps = df["timestamp"].values

object_coords = []
for i in range(1, 6):
    object_coords.append((df[f"x{i}"].values, df[f"y{i}"].values))

# -----------------------------
# Occupancy
# -----------------------------
if scene < 10:
    with open(f"all_poses/0{scene}_poses/{scene}_occupancy_xy_points.json") as f:
        occupancy_data = json.load(f)
else:
    with open(f"all_poses/{scene}_poses/{scene}_occupancy_xy_points.json") as f:
        occupancy_data = json.load(f)

occupancy_x = [p["x"] for p in occupancy_data]
occupancy_y = [p["y"] for p in occupancy_data]

# -----------------------------
# Plot
# -----------------------------
fig, ax = plt.subplots()

robot_triangle = plt.Polygon([[0, 0], [0, 0], [0, 0]], closed=True, color="red")
ax.add_patch(robot_triangle)

objects_plot = [ax.plot([], [], "bo", markersize=8)[0] for _ in range(5)]
object_labels = [ax.text(0, 0, str(i + 1), color="blue", fontsize=10) for i in range(5)]

ax.plot(occupancy_x, occupancy_y, "k.", markersize=1)

ax.set_xlim(0, 12)
ax.set_ylim(-7, 3)
ax.set_aspect("equal")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")

# -----------------------------
# Geometry
# -----------------------------
def get_triangle_coords(x, y, yaw, base=0.3, height=0.5):
    forward = np.array([np.cos(yaw), np.sin(yaw)])
    left = np.array([-np.sin(yaw), np.cos(yaw)])

    tip = np.array([x, y]) + height * forward
    base_center = np.array([x, y]) - 0.2 * height * forward

    return np.array([
        tip,
        base_center + (base / 2) * left,
        base_center - (base / 2) * left
    ])

# -----------------------------
# Frame & slice state
# -----------------------------
frame_idx = 0
n_frames = len(df)

slice_start_idx = None
selected_cols = None

LABEL_OFFSET = np.array([0.1, 0.1])

def draw_frame(i):
    robot_triangle.set_xy(
        get_triangle_coords(robot_x[i], robot_y[i], robot_yaw[i])
    )

    for j, (ox, oy) in enumerate(object_coords):
        objects_plot[j].set_data([ox[i]], [oy[i]])
        object_labels[j].set_position((ox[i] + LABEL_OFFSET[0], oy[i] + LABEL_OFFSET[1]))

    title = f"Time: {timestamps[i]:.3e}"
    if slice_start_idx is not None:
        title += " | SLICE START"
    if selected_cols is not None:
        title += " | ALL COLS" if len(selected_cols) == 5 else f" | COL {selected_cols[0]}"
    ax.set_title(title)

    fig.canvas.draw_idle()

# -----------------------------
# Slice saving
# -----------------------------
def save_slice(start_i, end_i, cols):
    if len(cols) == 5:
        # ---------- WIDE FORMAT (ALL) ----------
        rows = []
        for k in range(start_i, end_i + 1):
            row = {
                "timestamp": timestamps[k],
                "robot_x": robot_x[k],
                "robot_y": robot_y[k],
                "robot_yaw_rad": robot_yaw[k],
            }
            for c in range(1, 6):
                row[f"x{c}"] = object_coords[c - 1][0][k]
                row[f"y{c}"] = object_coords[c - 1][1][k]
            rows.append(row)

        out = pd.DataFrame(rows)

        fname = (
            f"track_scene{scene}_all_"
            f"from_{int(timestamps[start_i])}_to_{int(timestamps[end_i])}.csv"
        )

    else:
        # ---------- LONG FORMAT (SINGLE) ----------
        col = cols[0]
        rows = []
        ox, oy = object_coords[col - 1]

        for k in range(start_i, end_i + 1):
            rows.append({
                "timestamp": timestamps[k],
                "x": ox[k],
                "y": oy[k],
                "column": col,
                "robot_x": robot_x[k],
                "robot_y": robot_y[k],
                "robot_yaw_rad": robot_yaw[k],
            })

        out = pd.DataFrame(rows)

        fname = (
            f"track_scene{scene}_col{col}_"
            f"from_{int(timestamps[start_i])}_to_{int(timestamps[end_i])}.csv"
        )

    out_path = os.path.join(OUTPUT_DIR, fname)
    out.to_csv(out_path, index=False)
    print(f"Saved slice → {out_path}")

# -----------------------------
# Keyboard handler
# -----------------------------
def on_key(event):
    global frame_idx, slice_start_idx, selected_cols

    if event.key == "right":
        frame_idx = min(frame_idx + 1, n_frames - 1)

    elif event.key == "left":
        frame_idx = max(frame_idx - 1, 0)

    elif event.key == "w":
        slice_start_idx = frame_idx
        selected_cols = None
        print(f"Slice start @ {timestamps[frame_idx]}")

    elif event.key in ["1", "2", "3", "4", "5"] and slice_start_idx is not None:
        selected_cols = [int(event.key)]
        print(f"Selected column {selected_cols[0]}")

    elif event.key == "a" and slice_start_idx is not None:
        selected_cols = [1, 2, 3, 4, 5]
        print("Selected ALL columns")

    elif event.key == " " and slice_start_idx is not None and selected_cols is not None:
        if frame_idx > slice_start_idx:
            save_slice(slice_start_idx, frame_idx, selected_cols)
        else:
            print("Invalid slice")

        slice_start_idx = None
        selected_cols = None

    draw_frame(frame_idx)

fig.canvas.mpl_connect("key_press_event", on_key)

draw_frame(frame_idx)
plt.show()
