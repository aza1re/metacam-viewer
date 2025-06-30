import os
import yaml
import cv2
import numpy as np
import rerun as rr
import open3d as o3d
from pathlib import Path
from natsort import natsorted

def load_pointcloud(pcd_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else None
    return points, colors

def is_frame_complete(frame_dir, frame_id):
    """检查帧数据是否完整"""
    required_files = [
        f"{frame_dir}/extrinsics0/frame_{frame_id:04d}.yaml",
        f"{frame_dir}/extrinsics1/frame_{frame_id:04d}.yaml",
        f"{frame_dir}/images/camera0/frame_{frame_id:04d}.png",
        f"{frame_dir}/images/camera1/frame_{frame_id:04d}.png"
    ]
    return all(os.path.exists(f) for f in required_files)

def load_single_camera_data(frame_dir, camera_id, frame_id):
    """安全加载单个相机数据"""
    try:
        # 加载外参
        with open(f"{frame_dir}/extrinsics{camera_id}/frame_{frame_id:04d}.yaml") as f:
            ext = yaml.safe_load(f)
            if f"camera{camera_id}" not in ext:
                raise ValueError(f"YAML中缺少camera{camera_id}字段")
        
        # 加载图像
        img_path = f"{frame_dir}/images/camera{camera_id}/frame_{frame_id:04d}.png"
        img = cv2.imread(img_path)
        if img is None:
            raise ValueError(f"图像加载失败: {img_path}")
            
        return {
            "position": ext[f"camera{camera_id}"]["position"],
            "rotation": ext[f"camera{camera_id}"]["rotation"],
            "image": img
        }
    except Exception as e:
        print(f"相机{camera_id}帧{frame_id}加载失败: {str(e)}")
        return None

def load_frame_data(frame_dir, frame_id):
    """加载完整帧数据，自动跳过不完整的相机数据"""
    frame_data = {"camera0": None, "camera1": None}
    
    for cam_id in [0, 1]:
        cam_data = load_single_camera_data(frame_dir, cam_id, frame_id)
        if cam_data:
            frame_data[f"camera{cam_id}"] = cam_data
    
    # 如果两个相机都缺失，返回None
    return frame_data if frame_data["camera0"] or frame_data["camera1"] else None

def main():
    rr.init("Camera Trajectory Viewer", spawn=True)
    
    # 配置路径
    frame_dir = "output"
    pcd_path = r"C:\Users\64625\RM\projects\3Dviewer\cloud_group_0.pcd"
    
    # 加载点云
    points, colors = load_pointcloud(pcd_path)
    rr.log("world/points", rr.Points3D(points, colors=colors), static=True)
    print("here")
    
    # 获取所有可能的帧ID
    ext_files = natsorted(Path(f"{frame_dir}/extrinsics0").glob("frame_*.yaml"))
    all_frame_ids = [int(f.stem.split("_")[1]) for f in ext_files]
    print(f"找到 {len(all_frame_ids)} 帧数据")
    # 收集有效数据
    valid_frames = []
    cam0_positions, cam1_positions = [], []
    
    for frame_id in all_frame_ids:
        if not is_frame_complete(frame_dir, frame_id):
            print(f"跳过不完整帧: {frame_id}")
            continue
        print(f"加载帧 {frame_id}")
        frame_data = load_frame_data(frame_dir, frame_id)
        if not frame_data:
            continue
            
        valid_frames.append(frame_data)
        # 收集轨迹点（仅记录有数据的相机）
        if frame_data["camera0"]:
            cam0_positions.append(frame_data["camera0"]["position"])
        if frame_data["camera1"]:
            cam1_positions.append(frame_data["camera1"]["position"])
    
    # 绘制轨迹线（只有至少2个点时才绘制）
    if len(cam0_positions) > 1:
        rr.log("world/cameras/camera0/trajectory", 
              rr.LineStrips3D([np.array(cam0_positions)], colors=[255, 0, 0]),
              static=True)
    if len(cam1_positions) > 1:
        rr.log("world/cameras/camera1/trajectory",
              rr.LineStrips3D([np.array(cam1_positions)], colors=[0, 0, 255]),
              static=True)
        
    WIDTH, HEIGHT = 1920, 1080  # 假设的图像尺寸
    focal_length = 1000  # 假设的焦距(像素)
    principal_point = [WIDTH/2, HEIGHT/2]
    
    # 按时间轴加载有效帧
    for idx, frame_data in enumerate(valid_frames):
        rr.set_time(timeline="frame", sequence=idx)
        
        
        # 相机0数据（如果存在）
        if frame_data["camera0"]:
            pos = frame_data["camera0"]["position"]
            rot = frame_data["camera0"]["rotation"]
            img = frame_data["camera0"]["image"]
            
            # 记录相机3D姿态和图像
            rr.log(
                "world/camera0",
                rr.Transform3D(
                    translation=pos,
                    rotation=rr.Quaternion(xyzw=rot)
            ))
            rr.log(
                f"world/cameras/camera0/image",
                rr.Image(frame_data["camera0"]["image"]),
            )
            
            # 添加相机视锥体
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
            
            # 将图像投影到3D空间
            rr.log(
                "world/camera0/image_in_space",
                rr.Image(img).compress(jpeg_quality=75)
            )

        
        # 相机1数据（如果存在）
        if frame_data["camera1"]:
            pos = frame_data["camera1"]["position"]
            rot = frame_data["camera1"]["rotation"]
            img = frame_data["camera1"]["image"]
            
            # 记录相机3D姿态和图像
            rr.log(
                "world/camera1",
                rr.Transform3D(
                    translation=pos,
                    rotation=rr.Quaternion(xyzw=rot)
            ))
            rr.log(
                f"world/cameras/camera1/image",
                rr.Image(frame_data["camera1"]["image"]),
            )
            
            # 添加相机视锥体
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
                "world/camera1/image_in_space",
                rr.Image(img).compress(jpeg_quality=75)
            )

if __name__ == "__main__":
    main()