import open3d as o3d
import glob
import numpy as np
import json
import os
import pandas as pd
import bisect
import logging

# ------------------------
# Logger setup
# ------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
logger = logging.getLogger(__name__)

# ------------------------
# Configuration
# ------------------------
scene_number = 13

hsr_scene_numbers = [1, 2, 3, 4, 5, 6, 8, 9, 10, 12, 13, 27, 28, 29, 30, 31, 32, 34, 35, 36, 38, 39]

occupancy_grid_offset = {'x': 0, 'y': 0, 'z': -2.20, 'yaw': np.radians(0)}

TIME_OFFSET_NS = 0

if scene_number in hsr_scene_numbers or scene_number in [46, 52]:
    TIME_OFFSET_NS = 15_500_000_000

image_pose = occupancy_grid_offset.copy()

# ------------------------
# Functions
# ------------------------
def load_occupancy_from_json(json_path):
    with open(json_path, 'r') as f:
        points_xy = json.load(f)
    points = np.array([[pt["x"], pt["y"], 0.0] for pt in points_xy])
    colors = np.zeros((points.shape[0], 3))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

def vis_dict(directory):
    logger.info(f"Visualizing point clouds and annotations in {directory}")
    pcds = sorted(glob.glob(f'{directory}/*.pcd'))
    anns = sorted(glob.glob(f'{directory.replace("pointcloud", "ann")}/*.json'))
    if not pcds:
        logger.warning("No .pcd files found in the directory.")
        return

    logger.info(f"Found {len(pcds)} .pcd files with annotations")

    if scene_number < 10:
        pose_df = pd.read_csv(f'0{scene_number}_annotated/{scene_number}_robot_pose.csv')
    else:
        pose_df = pd.read_csv(f'{scene_number}_annotated/{scene_number}_robot_pose.csv')
    pose_times = pose_df['timestamp_ns'].values

    if scene_number < 10:
        with open(f'0{scene_number}_annotated/{scene_number}_grs_to_bot_offset.json', 'r') as f:
            offset_data = json.load(f)
    else:
        with open(f'{scene_number}_annotated/{scene_number}_grs_to_bot_offset.json', 'r') as f:
            offset_data = json.load(f)

    POSE_OFFSET = np.array([
        offset_data.get('x', 0.0),
        offset_data.get('y', 0.0),
        offset_data.get('z', 0.0)
    ])
    YAW_OFFSET_RAD = np.radians(offset_data.get('yaw_deg', 0.0))

    if scene_number < 10:
        occupancy_json_path = f"0{scene_number}_annotated/{scene_number}_occupancy_xy_points.json"
    else:
        occupancy_json_path = f"{scene_number}_annotated/{scene_number}_occupancy_xy_points.json"

    raw_occupancy_map = load_occupancy_from_json(occupancy_json_path)

    def extract_timestamp_from_filename(filename):
        ts_str = os.path.basename(filename).rsplit('.', 2)
        if len(ts_str) >= 2:
            full_ts = '.'.join(ts_str[:2])
            return int(float(full_ts) * 1e9)
        return None

    def find_closest_pose_idx(ts):
        i = bisect.bisect_left(pose_times, ts)
        if i > 0 and (i == len(pose_times) or abs(ts - pose_times[i - 1]) < abs(ts - pose_times[i])):
            return i - 1
        return i

    def create_robot_frame(x, y, yaw, offset=np.zeros(3), yaw_offset=0.0, size=0.3):
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
        T = np.eye(4)
        T[:2, :2] = [[np.cos(yaw), -np.sin(yaw)],
                     [np.sin(yaw),  np.cos(yaw)]]
        T[:3, 3] = np.array([x, y, 0.0]) + offset

        R = np.eye(4)
        R[:2, :2] = [[np.cos(yaw_offset), -np.sin(yaw_offset)],
                     [np.sin(yaw_offset),  np.cos(yaw_offset)]]

        final_transform = R @ T
        frame.transform(final_transform)
        return frame

    def set_initial_camera(view_ctrl):
        eye = np.array([5.0, 0.0, 5.0])
        lookat = np.array([5.0, 0.0, 10.0])
        up = np.array([0.0, 1.0, 0.0])
        front = (lookat - eye)
        front /= np.linalg.norm(front)
        view_ctrl.set_lookat(lookat)
        view_ctrl.set_front(front)
        view_ctrl.set_up(up)
        view_ctrl.set_zoom(0.5)

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    set_initial_camera(vis.get_view_control())
    idx = 0
    occupancy_map = None
    current_pcd = None

    def load_pcd(filepath):
        pcd = o3d.io.read_point_cloud(filepath)
        points = np.asarray(pcd.points)
        if points.size == 0:
            logger.warning(f"Empty point cloud in {filepath}, adding a dummy point.")
            points = np.array([[1.0, 0.0, 1.0]])
        pcd.points = o3d.utility.Vector3dVector(points)
        return pcd

    def load_annotations(json_file):
        if not os.path.exists(json_file):
            return []
        with open(json_file, 'r') as f:
            ann_data = json.load(f)
        cuboids = []
        for figure in ann_data.get("figures", []):
            if figure.get("geometryType") == "cuboid_3d":
                bbox = figure["geometry"]
                cuboids.append(draw_bbox(bbox))
        return cuboids

    def draw_bbox(bbox):
        center = np.array([bbox["position"]["x"], bbox["position"]["y"], bbox["position"]["z"]])
        size = np.array([bbox["dimensions"]["x"], bbox["dimensions"]["y"], bbox["dimensions"]["z"]])
        rotation = np.array([bbox["rotation"]["x"], bbox["rotation"]["y"], bbox["rotation"]["z"]])
        bbox_o3d = o3d.geometry.OrientedBoundingBox(center,
                                                    o3d.geometry.get_rotation_matrix_from_xyz(rotation),
                                                    size)
        bbox_o3d.color = (1, 0, 0)
        return bbox_o3d

    def update_view(vis, new_idx):
        nonlocal idx, occupancy_map, current_pcd
        if 0 <= new_idx < len(pcds):
            idx = new_idx
            pcd_file = pcds[idx]
            ts = extract_timestamp_from_filename(pcd_file)
            if ts is None:
                logger.warning(f"Could not extract timestamp from {pcd_file}")
                return

            adjusted_ts = ts + TIME_OFFSET_NS
            pose_idx = find_closest_pose_idx(adjusted_ts)
            pose_row = pose_df.iloc[pose_idx]

            ctr = vis.get_view_control()
            cam_params = ctr.convert_to_pinhole_camera_parameters()
            vis.clear_geometries()

            current_pcd = load_pcd(pcd_file)
            bbox_objs = load_annotations(anns[idx]) if anns else []
            robot_frame = create_robot_frame(
                pose_row['x'],
                pose_row['y'],
                pose_row['yaw_rad'],
                offset=POSE_OFFSET,
                yaw_offset=YAW_OFFSET_RAD
            )

            occupancy_map = o3d.geometry.PointCloud(raw_occupancy_map)
            transform = np.eye(4)
            cos_yaw = np.cos(image_pose['yaw'])
            sin_yaw = np.sin(image_pose['yaw'])
            transform[:2, :2] = [[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]]
            transform[:3, 3] = [image_pose['x'], image_pose['y'], image_pose['z']]
            occupancy_map.transform(transform)

            vis.add_geometry(current_pcd)
            vis.add_geometry(occupancy_map)
            for bbox in bbox_objs:
                vis.add_geometry(bbox)
            vis.add_geometry(robot_frame)
            vis.update_renderer()
            ctr.convert_from_pinhole_camera_parameters(cam_params)

    def right_click(vis): update_view(vis, idx + 1)
    def left_click(vis): update_view(vis, idx - 1)
    def exit_key(vis): logger.info("Exiting visualization."); vis.destroy_window()

    def adjust_image_pose(dx=0.0, dy=0.0, dz=0.0, dyaw=0.0):
        image_pose['x'] += dx
        image_pose['y'] += dy
        image_pose['z'] += dz
        image_pose['yaw'] += dyaw
        logger.info(f"Image pose updated: x={image_pose['x']:.2f}, y={image_pose['y']:.2f}, z={image_pose['z']:.2f}, yaw={np.degrees(image_pose['yaw']):.1f}°")
        update_view(vis, idx)

    def save_occupancy_xy():
        if occupancy_map is None:
            logger.warning("No occupancy map loaded.")
            return
        points = np.asarray(occupancy_map.points)
        colors = np.asarray(occupancy_map.colors)
        black_mask = np.all(colors == 0, axis=1)
        black_points = points[black_mask]
        xy = black_points[:, :2]
        data = [{"x": float(x), "y": float(y)} for x, y in xy]
        with open("occupancy_xy_points.json", "w") as f:
            json.dump(data, f, indent=2)
        logger.info(f"Saved {xy.shape[0]} black occupancy grid x,y points to occupancy_xy_points.json")

    def save_current_pcd():
        if current_pcd is None:
            logger.warning("No point cloud loaded.")
            return
        filename = f"saved_pointcloud_{idx:04d}.pcd"
        o3d.io.write_point_cloud(filename, current_pcd)
        logger.info(f"Saved current point cloud to {filename}")

    vis.register_key_callback(262, right_click)
    vis.register_key_callback(263, left_click)
    vis.register_key_callback(32, exit_key)

    vis.register_key_callback(ord('A'), lambda vis: adjust_image_pose(dx=-0.1))
    vis.register_key_callback(ord('D'), lambda vis: adjust_image_pose(dx=+0.1))
    vis.register_key_callback(ord('W'), lambda vis: adjust_image_pose(dy=+0.1))
    vis.register_key_callback(ord('S'), lambda vis: adjust_image_pose(dy=-0.1))
    vis.register_key_callback(ord('Q'), lambda vis: adjust_image_pose(dyaw=+np.radians(1)))
    vis.register_key_callback(ord('E'), lambda vis: adjust_image_pose(dyaw=-np.radians(1)))
    vis.register_key_callback(265, lambda vis: adjust_image_pose(dz=+0.1))
    vis.register_key_callback(264, lambda vis: adjust_image_pose(dz=-0.1))

    vis.register_key_callback(ord('P'), lambda vis: save_current_pcd())
    vis.register_key_callback(ord('O'), lambda vis: save_occupancy_xy())

    update_view(vis, idx)
    vis.run()

# ------------------------
# Main entry point
# ------------------------
if __name__ == '__main__':
    if scene_number < 10:
        vis_dict(f'0{scene_number}_annotated/scene_{scene_number}/pointcloud')
    else:
        vis_dict(f'{scene_number}_annotated/scene_{scene_number}/pointcloud')
