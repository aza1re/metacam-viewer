import os
import yaml
import cv2
import numpy as np
import rerun as rr
import open3d as o3d
from pathlib import Path
from natsort import natsorted
from tqdm import tqdm  # Add tqdm for progress bar

def load_pointcloud(pcd_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else None
    return points, colors

def is_frame_complete(frame_dir, frame_id):
    """Check if frame data is complete"""
    required_files = [
        f"{frame_dir}/extrinsics0/frame_{frame_id:04d}.yaml",
        f"{frame_dir}/extrinsics1/frame_{frame_id:04d}.yaml",
        f"{frame_dir}/images/camera0/frame_{frame_id:04d}.png",
        f"{frame_dir}/images/camera1/frame_{frame_id:04d}.png"
    ]
    return all(os.path.exists(f) for f in required_files)

def load_single_camera_data(frame_dir, camera_id, frame_id):
    """Safely load single camera data"""
    try:
        # Load extrinsics
        with open(f"{frame_dir}/extrinsics{camera_id}/frame_{frame_id:04d}.yaml") as f:
            ext = yaml.safe_load(f)
            if f"camera{camera_id}" not in ext:
                raise ValueError(f"Missing camera{camera_id} field in YAML")
        # Load image
        img_path = f"{frame_dir}/images/camera{camera_id}/frame_{frame_id:04d}.png"
        img = cv2.imread(img_path)
        if img is None:
            raise ValueError(f"Failed to load image: {img_path}")
        return {
            "position": ext[f"camera{camera_id}"]["position"],
            "rotation": ext[f"camera{camera_id}"]["rotation"],
            "image": img
        }
    except Exception as e:
        print(f"Camera {camera_id} frame {frame_id} failed to load: {str(e)}")
        return None

def load_frame_data(frame_dir, frame_id):
    """Load complete frame data, automatically skip incomplete camera data"""
    frame_data = {"camera0": None, "camera1": None}
    for cam_id in [0, 1]:
        cam_data = load_single_camera_data(frame_dir, cam_id, frame_id)
        if cam_data:
            frame_data[f"camera{cam_id}"] = cam_data
    # If both cameras are missing, return None
    return frame_data if frame_data["camera0"] or frame_data["camera1"] else None

def main():
    rr.init("metacam-viewer")  # Start a recording session (required for rerun >=0.23)
    rr.spawn()
    # Set paths
    frame_dir = "output"
    pcd_path = r"/home/user/metacam-edu-reader/3Dviewer/cloud_group_0.pcd"
    # Load point cloud
    points, colors = load_pointcloud(pcd_path)
    rr.log("world/points", rr.Points3D(points, colors=colors), static=True)
    # Get all possible frame IDs
    ext_files = natsorted(Path(f"{frame_dir}/extrinsics0").glob("frame_*.yaml"))
    all_frame_ids = [int(f.stem.split("_")[1]) for f in ext_files]
    print(f"Found {len(all_frame_ids)} frames")
    # Collect valid data
    valid_frames = []
    cam0_positions, cam1_positions = [], []
    for frame_id in tqdm(all_frame_ids, desc="Loading frames"):
        if not is_frame_complete(frame_dir, frame_id):
            continue
        frame_data = load_frame_data(frame_dir, frame_id)
        if not frame_data:
            continue
        valid_frames.append(frame_data)
        # Collect trajectory points (only record cameras with data)
        if frame_data["camera0"]:
            cam0_positions.append(frame_data["camera0"]["position"])
        if frame_data["camera1"]:
            cam1_positions.append(frame_data["camera1"]["position"])
    # Draw trajectory lines (only if at least 2 points)
    if len(cam0_positions) > 1:
        rr.log("world/cameras/camera0/trajectory", 
              rr.LineStrips3D([np.array(cam0_positions)], colors=[255, 0, 0]),
              static=True)
    if len(cam1_positions) > 1:
        rr.log("world/cameras/camera1/trajectory",
              rr.LineStrips3D([np.array(cam1_positions)], colors=[0, 0, 255]),
              static=True)
    # Camera intrinsics (example values)
    WIDTH, HEIGHT = 1920, 1080
    focal_length = 1000
    principal_point = [WIDTH / 2, HEIGHT / 2]

    # Load valid frames on timeline
    for idx, frame_data in enumerate(valid_frames):
        rr.set_time(timeline="frame", sequence=idx)
        img0 = frame_data["camera0"]["image"] if frame_data["camera0"] else None
        img1 = frame_data["camera1"]["image"] if frame_data["camera1"] else None

        # Camera 0 frustum (if exists)
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

        # Camera 1 frustum (if exists)
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

        # Log combined image if both exist
        if img0 is not None and img1 is not None:
            if img0.shape[0] == img1.shape[0]:
                combined = np.hstack([img0, img1])
            else:
                import cv2
                img1_resized = cv2.resize(img1, (img1.shape[1], img0.shape[0]))
                combined = np.hstack([img0, img1_resized])
            rr.log("world/cameras/combined_image", rr.Image(combined))

if __name__ == "__main__":
    main()