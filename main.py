import os
import sys
import yaml
import cv2
import numpy as np
import rerun as rr
import open3d as o3d
from pathlib import Path
from natsort import natsorted
from tqdm import tqdm
import json
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R
import tkinter as tk
from tkinter import filedialog
import argparse

class FishEyeToPointCloud:
    def __init__(self, calibration, pcd_path, undistorted_size=(760, 1008)):
        self.calibration = calibration
        self._load_point_cloud(pcd_path)
        self._init_undistort_maps()
        self.undistorted_size = undistorted_size  # (width, height)
        self.scale_x = undistorted_size[0] / calibration["cameras"][0]["width"]
        self.scale_y = undistorted_size[1] / calibration["cameras"][0]["height"]

    def _load_point_cloud(self, path):
        pcd = o3d.io.read_point_cloud(path)
        self.points = np.asarray(pcd.points)
        self.kd_tree = KDTree(self.points) if len(self.points) > 0 else None
        print(f"Loaded {len(self.points)} point cloud points")

    def _init_undistort_maps(self):
        self.maps = {}
        for cam in self.calibration["cameras"]:
            K = np.array([
                [cam["intrinsic"]["fl_x"], 0, cam["intrinsic"]["cx"]],
                [0, cam["intrinsic"]["fl_y"], cam["intrinsic"]["cy"]],
                [0, 0, 1]
            ])
            D = np.array([
                cam["distortion"]["params"]["k1"],
                cam["distortion"]["params"]["k2"],
                cam["distortion"]["params"]["k3"],
                cam["distortion"]["params"]["k4"]
            ])
            mapx, mapy = cv2.fisheye.initUndistortRectifyMap(
                K, D, np.eye(3), K,
                (cam["width"], cam["height"]),
                cv2.CV_32FC1
            )
            self.maps[cam["name"]] = (mapx, mapy)

    def _undist_to_orig(self, x_undist, y_undist, camera_name):
        x_scaled = x_undist / self.scale_x
        y_scaled = y_undist / self.scale_y
        mapx, mapy = self.maps[camera_name]
        return mapx[int(y_scaled), int(x_scaled)], mapy[int(y_scaled), int(x_scaled)]

    def _fisheye_backproject(self, x_orig, y_orig, camera_name):
        cam = next(c for c in self.calibration["cameras"] if c["name"] == camera_name)
        fx, fy = cam["intrinsic"]["fl_x"], cam["intrinsic"]["fl_y"]
        cx, cy = cam["intrinsic"]["cx"], cam["intrinsic"]["cy"]
        x_norm = (x_orig - cx) / fx
        y_norm = (y_orig - cy) / fy
        r = np.sqrt(x_norm**2 + y_norm**2)
        if r < 1e-6:
            return np.array([0, 0, 1.0])
        theta = r
        k1, k2, k3, k4 = [cam["distortion"]["params"][f"k{i}"] for i in range(1,5)]
        for _ in range(5):
            theta2 = theta*theta
            theta4 = theta2*theta2
            theta6 = theta4*theta2
            theta8 = theta4*theta4
            f = theta * (1 + k1*theta2 + k2*theta4 + k3*theta6 + k4*theta8) - r
            df = 1 + 3*k1*theta2 + 5*k2*theta4 + 7*k3*theta6 + 9*k4*theta8
            theta -= f / df
        s = np.sin(theta)
        ray = np.array([
            s * x_norm / r,
            s * y_norm / r,
            np.cos(theta)
        ])
        return ray / np.linalg.norm(ray)

    def _find_intersection(self, ray_origin, ray_dir, max_dist=20.0, steps=1000, hit_thresh=0.1):
        if self.kd_tree is None:
            return max_dist
        t_values = np.linspace(0, max_dist, steps)
        query_points = ray_origin + t_values[:, None] * ray_dir
        distances, _ = self.kd_tree.query(query_points, workers=-1)
        hit_indices = np.where(distances < hit_thresh)[0]
        if len(hit_indices) > 0:
            return t_values[hit_indices[0]]
        else:
            return max_dist

    def image_to_pointcloud(self, x_undist, y_undist, camera_name, frame_data):
        try:
            x_orig, y_orig = self._undist_to_orig(x_undist, y_undist,
                                                  "left" if camera_name == "camera0" else "right")
        except KeyError:
            raise ValueError(f"Camera {camera_name} not found in calibration")
        ray_cam = self._fisheye_backproject(x_orig, y_orig,
                                            "left" if camera_name == "camera0" else "right")
        cam_data = frame_data[camera_name]
        cam_pos = np.array(cam_data["position"])
        rot_matrix = R.from_quat(cam_data["rotation"]).as_matrix()
        ray_world = rot_matrix @ ray_cam
        depth = self._find_intersection(cam_pos, ray_world)
        return cam_pos + ray_world * depth

def load_calibration(calib_path):
    with open(calib_path) as f:
        return json.load(f)

def load_pointcloud(pcd_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else None
    return points, colors

def is_frame_complete(frame_dir, frame_id):
    required_files = [
        f"{frame_dir}/extrinsics0/frame_{frame_id:04d}.yaml",
        f"{frame_dir}/extrinsics1/frame_{frame_id:04d}.yaml",
        f"{frame_dir}/images/camera0/frame_{frame_id:04d}.png",
        f"{frame_dir}/images/camera1/frame_{frame_id:04d}.png"
    ]
    return all(os.path.exists(f) for f in required_files)

def load_single_camera_data(frame_dir, camera_id, frame_id):
    try:
        with open(f"{frame_dir}/extrinsics{camera_id}/frame_{frame_id:04d}.yaml") as f:
            ext = yaml.safe_load(f)
            if f"camera{camera_id}" not in ext:
                raise ValueError(f"Missing camera{camera_id} field in YAML")
        img_path = f"{frame_dir}/images/camera{camera_id}/frame_{frame_id:04d}.png"
        img = cv2.imread(img_path)
        if img is None:
            raise ValueError(f"Failed to load image: {img_path}")
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return {
            "position": ext[f"camera{camera_id}"]["position"],
            "rotation": ext[f"camera{camera_id}"]["rotation"],
            "image": img_rgb
        }
    except Exception as e:
        print(f"Camera {camera_id} frame {frame_id} failed to load: {str(e)}")
        return None

def load_frame_data(frame_dir, frame_id):
    frame_data = {"camera0": None, "camera1": None}
    for cam_id in [0, 1]:
        cam_data = load_single_camera_data(frame_dir, cam_id, frame_id)
        if cam_data:
            frame_data[f"camera{cam_id}"] = cam_data
    return frame_data if frame_data["camera0"] or frame_data["camera1"] else None

clicked_points = []
clicked_points_2d = []

def visualize_interactive(converter, frame_data, camera_name):
    global clicked_points, clicked_points_2d
    clicked_points_2d = []
    img = frame_data[camera_name]["image"]
    img_disp = cv2.cvtColor(img.copy(), cv2.COLOR_RGB2BGR)

    def redraw_marks():
        nonlocal img_disp
        img_disp = cv2.cvtColor(img.copy(), cv2.COLOR_RGB2BGR)
        for x, y in clicked_points_2d:
            cv2.circle(img_disp, (int(x), int(y)), 10, (0, 0, 255), 2)
        cv2.imshow("Image", img_disp)

    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            try:
                point_3d = converter.image_to_pointcloud(x, y, camera_name, frame_data)
                print(f"2D: ({x}, {y}) → 3D: {point_3d}")
                clicked_points.append(point_3d)
                clicked_points_2d.append((x, y))
                rr.log(
                    "world/clicked_points",
                    rr.Points3D(clicked_points, colors=[255, 0, 0], radii=0.1),
                    static=True
                )
                redraw_marks()
            except Exception as e:
                print(f"Error: {e}")

    cv2.namedWindow("Image")
    cv2.setMouseCallback("Image", mouse_callback)
    redraw_marks()

    while True:
        key = cv2.waitKey(0)
        if key == ord('q') or key == 27:
            break
        elif key == ord('s'):
            np.save("clicked_points.npy", np.array(clicked_points))
            print(f"Saved {len(clicked_points)} points to clicked_points.npy")
        elif key == ord('i'):
            save_image_with_dialog(img_disp)
        elif key == ord('z'):
            if clicked_points:
                clicked_points.pop()
                clicked_points_2d.pop()
                rr.log(
                    "world/clicked_points",
                    rr.Points3D(clicked_points, colors=[255, 0, 0], radii=0.1),
                    static=True
                )
                redraw_marks()
            else:
                print("No points to undo")
    cv2.destroyAllWindows()

def save_image_with_dialog(img):
    root = tk.Tk()
    root.withdraw()
    file_path = filedialog.asksaveasfilename(
        defaultextension=".png",
        filetypes=[("PNG files", "*.png"), ("JPEG files", "*.jpg;*.jpeg"), ("All files", "*.*")]
    )
    if file_path:
        cv2.imwrite(file_path, img)
        print(f"Saved image to {file_path}")
    root.destroy()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pointcloud', type=str, help='Path to a single point cloud file')
    parser.add_argument('--all-pointclouds', action='store_true', help='Show all point clouds')
    args = parser.parse_args()

    global clicked_points, clicked_points_2d
    rr.init("Camera Trajectory Viewer", spawn=True)

    frame_dir = "/home/user/metacam-edu-reader/output"
    pcd_dir = "/home/user/metacam-edu-reader/output/pointcloud"
    calib_path = "/home/user/metacam-edu-reader/output/calibration.json"

    if args.pointcloud:
        pcd_files = [Path(args.pointcloud)]
    elif args.all_pointclouds:
        pcd_files = natsorted(Path(pcd_dir).glob("*.pcd"))
    else:
        # Default: maybe first point cloud or prompt user
        pcd_files = natsorted(Path(pcd_dir).glob("*.pcd"))
        if not pcd_files:
            print("No .pcd files found!")
            return
        pcd_files = [pcd_files[0]]

    calibration = load_calibration(calib_path)

    for i, pcd_path in enumerate(pcd_files):
        print(f"Processing point cloud: {pcd_path}")
        points, colors = load_pointcloud(str(pcd_path))
        rr.log(f"world/points_{i}", rr.Points3D(points, colors=colors), static=True)
        # ... rest of your frame loading and visualization code ...
        # Optionally, break here if you only want to process the first one interactively

    if os.path.exists("clicked_points.npy"):
        clicked_points = np.load("clicked_points.npy").tolist()
    else:
        clicked_points = []
    clicked_points_2d = []

    rr.log(
        "world/clicked_points",
        rr.Points3D(clicked_points, colors=[255, 0, 0], radii=0.1),
        static=True
    )

    ext_files = natsorted(Path(f"{frame_dir}/extrinsics0").glob("frame_*.yaml"))
    all_frame_ids = [int(f.stem.split("_")[1]) for f in ext_files]
    print(f"Found {len(all_frame_ids)} frames")
    index_to_frame_id = {i: fid for i, fid in enumerate(all_frame_ids)}
    valid_frames = []
    cam0_positions, cam1_positions = [], []

    for frame_id in all_frame_ids:
        if not is_frame_complete(frame_dir, frame_id):
            print(f"Skipping incomplete frame: {frame_id}")
            continue
        print(f"Loading frame {frame_id}")
        frame_data = load_frame_data(frame_dir, frame_id)
        if not frame_data:
            continue
        valid_frames.append(frame_data)
        if frame_data["camera0"]:
            cam0_positions.append(frame_data["camera0"]["position"])
        if frame_data["camera1"]:
            cam1_positions.append(frame_data["camera1"]["position"])

    if len(cam0_positions) > 1:
        rr.log("world/cameras/camera0/trajectory",
              rr.LineStrips3D([np.array(cam0_positions)], colors=[255, 0, 0]),
              static=True)
    if len(cam1_positions) > 1:
        rr.log("world/cameras/camera1/trajectory",
              rr.LineStrips3D([np.array(cam1_positions)], colors=[0, 0, 255]),
              static=True)

    WIDTH, HEIGHT = 1920, 1080
    focal_length = 1000
    principal_point = [WIDTH / 2, HEIGHT / 2]

    for idx, frame_data in enumerate(valid_frames):
        rr.set_time(timeline="frame", sequence=idx)
        img0 = frame_data["camera0"]["image"] if frame_data["camera0"] else None
        img1 = frame_data["camera1"]["image"] if frame_data["camera1"] else None
        if frame_data["camera0"]:
            pos = frame_data["camera0"]["position"]
            rot = frame_data["camera0"]["rotation"]
            rr.log(
                "world/camera0",
                rr.Transform3D(
                    translation=pos,
                    rotation=rr.Quaternion(xyzw=rot)
                )
            )
            rr.log(
                "world/camera0/frustum",
                rr.Pinhole(
                    focal_length=focal_length,
                    principal_point=principal_point,
                    width=WIDTH,
                    height=HEIGHT,
                    camera_xyz=rr.ViewCoordinates.RDF
                )
            )
            rr.log(
                f"world/cameras/camera0/image",
                rr.Image(img0),
            )
            rr.log(
                "world/camera0/image_in_space",
                rr.Image(img0).compress(jpeg_quality=75)
            )
        if frame_data["camera1"]:
            pos = frame_data["camera1"]["position"]
            rot = frame_data["camera1"]["rotation"]
            rr.log(
                "world/camera1",
                rr.Transform3D(
                    translation=pos,
                    rotation=rr.Quaternion(xyzw=rot)
                )
            )
            rr.log(
                "world/camera1/frustum",
                rr.Pinhole(
                    focal_length=focal_length,
                    principal_point=principal_point,
                    width=WIDTH,
                    height=HEIGHT,
                    camera_xyz=rr.ViewCoordinates.RDF
                )
            )
            rr.log(
                f"world/cameras/camera1/image",
                rr.Image(img1),
            )
            rr.log(
                "world/camera1/image_in_space",
                rr.Image(img1).compress(jpeg_quality=75)
            )
        if img0 is not None and img1 is not None:
            if img0.shape[0] == img1.shape[0]:
                combined = np.hstack([img0, img1])
            else:
                img1_resized = cv2.resize(img1, (img1.shape[1], img0.shape[0]))
                combined = np.hstack([img0, img1_resized])
            rr.log("world/cameras/combined_image", rr.Image(combined))

    while True:
        try:
            user_input = input("Enter frame index and camera (e.g. '0 left'), or 'q' to quit: ")
            if user_input.lower() == 'q':
                break
            idx_str, camera = user_input.split()
            idx = int(idx_str)
            if idx not in index_to_frame_id:
                print(f"Index {idx} out of range! Valid: 0 ~ {len(all_frame_ids)-1}")
                continue
            frame_id = index_to_frame_id[idx]
            camera_name = "camera0" if camera.lower() == "left" else "camera1"
            frame_data = load_frame_data(frame_dir, frame_id)
            if not frame_data or not frame_data[camera_name]:
                print(f"No data for {camera_name} in frame {frame_id}")
                continue
            visualize_interactive(converter, frame_data, camera_name)
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main()