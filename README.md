
# NavWareSet Tutorials

📚 **NavWareSet Tutorials** is a hands-on workspace showing how to open, visualize, and work with data from the [NavWareSet dataset](https://anr-navware.github.io/navwareset/).

This repository demonstrates:
- How to plot robot & participant trajectories from CSV.
- How to visualize annotated 3D point clouds and occupancy maps.
- How to replay LIDAR data from `.bag` files with proper static transforms.
- How to inspect LIDAR data using RViz.

---

## 📂 **Repository Structure**

```
.
├── 13_annotated/            # Example annotation files
├── 13_poses/                # Example pose CSVs
├── 13_grs.bag               # Example ROS bag (GRS sensors)
├── 13_robot.bag             # Example ROS bag (robot sensors)
├── csv_scene_viewer.py      # Python: 2D pose & occupancy plot
├── pcd_annotated_scene_viewer.py  # Python: 3D PCD + annotations
├── launch_lidar_data.launch # ROS launch file for bag playback & RViz
├── my_rviz.rviz             # Example RViz config
├── requirements.txt         # Python dependencies
├── README.md                # 📖 You are here!
```
---
**Disclaimer**: The ROS bags in this repo are 1 second slices of the originals. The originals are dozens of GB in size and because of that can not be uploaded to GitHub.

## ⚙️ **1️⃣ Install Python Dependencies**

Create a virtual environment (recommended):
```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

---

## ▶️ **2️⃣ Run the Python Scripts**

### ✅ **A. `csv_scene_viewer.py` — Plot 2D Poses & Occupancy**

**What it does:**  
Plots the robot’s trajectory, participant positions, and the occupancy map over time using Matplotlib.

**How to run:**
```bash
source .venv/bin/activate
python csv_scene_viewer.py
```

This will open an animated Matplotlib window showing robot movement and static occupancy points.

---

### ✅ **B. `pcd_annotated_scene_viewer.py` — View 3D Point Clouds + Annotations**

**What it does:**  
Visualizes time-synchronized 3D point clouds, annotated bounding boxes, robot frames, and occupancy grids using Open3D.

**How to run:**
```bash
source .venv/bin/activate
python pcd_annotated_scene_viewer.py
```

**Controls:**
- `→` / `←` : Step forward/backward through frames.
- `W/A/S/D`, `Q/E`, `↑/↓` : Adjust occupancy grid position and yaw.
- `P` : Save current point cloud.
- `O` : Save occupancy grid XY points.
- `Space` : Exit viewer.

---

## 📡 **3️⃣ Open the ROS Bags**

### ✅ **A. `13_grs.bag` — Play LIDAR Data with Transforms**

**How to run:**
```bash
roslaunch launch_lidar_data.launch
```

**What this does:**
- Plays the `13_grs.bag` at 1x speed.
- Publishes required static transforms (`map → rslidar → camera_color → camera_color_optical_frame`).
- Opens RViz with `my_rviz.rviz` configuration.

**Tip:**  
If you want to use your own `.bag` or `.rviz` config, edit `launch_lidar_data.launch` or pass them as arguments.

---

### ✅ **B. `13_robot.bag` — Play Robot Poses (Manually)**

**How to run:**

This bag can be played separately:
```bash
rosbag play --clock 13_robot.bag
```

Then open RViz manually:
```bash
rviz -d my_rviz.rviz
```

**Note:** Make sure `use_sim_time` is true if playing bags with timestamps.

---

## 📎 **4️⃣ Dataset Source**

To download or explore more scenes, visit:  
👉 [NavWareSet Official Website](https://anr-navware.github.io/navwareset/)

---

## ✅ **5️⃣ Best Practices**

- Always activate your Python virtual environment:
  ```bash
  source .venv/bin/activate
  ```
- Keep ROS bags **out of Git** if they are large — share them via dataset download or separate link.
- Use relative paths in your `.launch` files for portability.

---

## 📖 **6️⃣ Citation**

If you use NavWareSet in your research, please cite the authors as indicated on the official website.

---

## 🚀 Happy exploring!
