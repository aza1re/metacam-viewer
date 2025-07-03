import os
import yaml
import cv2
import numpy as np
import rerun as rr
import open3d as o3d
from pathlib import Path
from natsort import natsorted
import threading
from rerun.blueprint import BlueprintPanel
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R
import time
import json
import tkinter as tk
from tkinter import filedialog

class FishEyeToPointCloud:
    def __init__(self, calibration, pcd_path, undistorted_size=(760, 1008)):
        """
        :param calibration: 加载的标定JSON数据
        :param pcd_path: 点云文件路径
        """
        self.calibration = calibration
        self._load_point_cloud(pcd_path)
        self._init_undistort_maps()
        self.undistorted_size = undistorted_size  # (width, height)
        self.scale_x = undistorted_size[0] / calibration["cameras"][0]["width"]
        self.scale_y = undistorted_size[1] / calibration["cameras"][0]["height"]

    def _load_point_cloud(self, path):
        """加载点云并构建KDTree"""
        pcd = o3d.io.read_point_cloud(path)
        self.points = np.asarray(pcd.points)
        self.kd_tree = KDTree(self.points) if len(self.points) > 0 else None
        print(f"Loaded {len(self.points)} point cloud points")

    def _init_undistort_maps(self):
        """初始化鱼眼去畸变映射表"""
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
            # 注意：这里使用原始内参作为新内参，保持坐标系一致
            mapx, mapy = cv2.fisheye.initUndistortRectifyMap(
                K, D, np.eye(3), K, 
                (cam["width"], cam["height"]), 
                cv2.CV_32FC1
            )
            self.maps[cam["name"]] = (mapx, mapy)

    def _undist_to_orig(self, x_undist, y_undist, camera_name):
        # 先将去畸变坐标映射到原始图像坐标系
        x_scaled = x_undist / self.scale_x
        y_scaled = y_undist / self.scale_y
        # 再查映射表获取原始鱼眼坐标
        mapx, mapy = self.maps[camera_name]
        return mapx[int(y_scaled), int(x_scaled)], mapy[int(y_scaled), int(x_scaled)]

    def _fisheye_backproject(self, x_orig, y_orig, camera_name):
        """鱼眼相机反投影核心算法"""
        cam = next(c for c in self.calibration["cameras"] if c["name"] == camera_name)
        fx, fy = cam["intrinsic"]["fl_x"], cam["intrinsic"]["fl_y"]
        cx, cy = cam["intrinsic"]["cx"], cam["intrinsic"]["cy"]
        
        # 归一化坐标
        x_norm = (x_orig - cx) / fx
        y_norm = (y_orig - cy) / fy
        r = np.sqrt(x_norm**2 + y_norm**2)
        
        if r < 1e-6:  # 光心附近
            return np.array([0, 0, 1.0])
        
        # 迭代求解theta（入射角）
        theta = r  # 初始值
        k1, k2, k3, k4 = [cam["distortion"]["params"][f"k{i}"] for i in range(1,5)]
        
        for _ in range(5):  # 5次牛顿迭代
            theta2 = theta*theta
            theta4 = theta2*theta2
            theta6 = theta4*theta2
            theta8 = theta4*theta4
            f = theta * (1 + k1*theta2 + k2*theta4 + k3*theta6 + k4*theta8) - r
            df = 1 + 3*k1*theta2 + 5*k2*theta4 + 7*k3*theta6 + 9*k4*theta8
            theta -= f / df
        
        # 计算3D射线方向
        s = np.sin(theta)
        ray = np.array([
            s * x_norm / r,
            s * y_norm / r,
            np.cos(theta)
        ])
        return ray / np.linalg.norm(ray)

    def _find_intersection(self, ray_origin, ray_dir, max_dist=20.0, steps=1000, hit_thresh=0.1):
        """沿射线步进，遇到距离点云小于阈值的第一个点就返回"""
        if self.kd_tree is None:
            return max_dist

        t_values = np.linspace(0, max_dist, steps)
        query_points = ray_origin + t_values[:, None] * ray_dir
        distances, _ = self.kd_tree.query(query_points, workers=-1)

        hit_indices = np.where(distances < hit_thresh)[0]
        if len(hit_indices) > 0:
            # 命中第一个点
            return t_values[hit_indices[0]]
        else:
            # 没有命中，返回最大距离
            return max_dist

    def image_to_pointcloud(self, x_undist, y_undist, camera_name, frame_data):
        """
        完整转换流程：
        1. 去畸变坐标 → 原始鱼眼坐标
        2. 鱼眼反投影 → 3D射线
        3. 转换到世界坐标系
        4. 与点云求交
        """
        # 1. 获取原始鱼眼坐标
        try:
            x_orig, y_orig = self._undist_to_orig(x_undist, y_undist, 
                                                "left" if camera_name == "camera0" else "right")
        except KeyError:
            raise ValueError(f"Camera {camera_name} not found in calibration")
        
        # 2. 鱼眼反投影
        ray_cam = self._fisheye_backproject(x_orig, y_orig, 
                                          "left" if camera_name == "camera0" else "right")
        
        # 3. 获取相机位姿
        cam_data = frame_data[camera_name]
        cam_pos = np.array(cam_data["position"])
        rot_matrix = R.from_quat(cam_data["rotation"]).as_matrix()
        
        # 4. 转换到世界坐标系
        ray_world = rot_matrix @ ray_cam
        
        # 5. 寻找最近交点
        depth = self._find_intersection(cam_pos, ray_world)
        return cam_pos + ray_world * depth

def load_calibration(calib_path):
    """加载标定文件"""
    with open(calib_path) as f:
        return json.load(f)
    
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
            
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
        return {
            "position": ext[f"camera{camera_id}"]["position"],
            "rotation": ext[f"camera{camera_id}"]["rotation"],
            "image": img_rgb
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
            print(f"已保存 {len(clicked_points)} 个点到 clicked_points.npy")
        elif key == ord('i'):
            save_image_with_dialog(img_disp)
        elif key == ord('z'):
            # 撤回
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
                print("没有可撤回的点")

    cv2.destroyAllWindows()

def save_image_with_dialog(img):
    root = tk.Tk()
    root.withdraw()  # 不显示主窗口
    file_path = filedialog.asksaveasfilename(
        defaultextension=".png",
        filetypes=[("PNG files", "*.png"), ("JPEG files", "*.jpg;*.jpeg"), ("All files", "*.*")]
    )
    if file_path:
        # 注意：OpenCV保存需要BGR格式
        cv2.imwrite(file_path, img)
        print(f"已保存图片到 {file_path}")
    root.destroy()

def main():
    global clicked_points, clicked_points_2d
    rr.init("Camera Trajectory Viewer", spawn=True)
    # 配置路径
    frame_dir = "/home/user/metacam-edu-reader/output"
    pcd_path = "home/user/metacam-edu-reader/output/pointcloud/cloud_group_0.pcd"
    calib_path = "/home/user/metacam-edu-reader/output/calibration.json"
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
    # 加载
    calibration = load_calibration(calib_path)
    points, colors = load_pointcloud(pcd_path)
    converter = FishEyeToPointCloud(calibration, pcd_path)
    rr.log("world/points", rr.Points3D(points, colors=colors), static=True)
    print("here")
    
    # 获取所有可能的帧ID
    ext_files = natsorted(Path(f"{frame_dir}/extrinsics0").glob("frame_*.yaml"))
    all_frame_ids = [int(f.stem.split("_")[1]) for f in ext_files]
    print(f"找到 {len(all_frame_ids)} 帧数据")
    # 顺序号（0,1,2...）到原始帧号的映射
    index_to_frame_id = {i: fid for i, fid in enumerate(all_frame_ids)}
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
    
    # 绘制轨迹线
    if len(cam0_positions) > 1:
        rr.log("world/cameras/camera0/trajectory", 
              rr.LineStrips3D([np.array(cam0_positions)], colors=[255, 0, 0]),
              static=True)
    if len(cam1_positions) > 1:
        rr.log("world/cameras/camera1/trajectory",
              rr.LineStrips3D([np.array(cam1_positions)], colors=[0, 0, 255]),
              static=True)
        
    WIDTH, HEIGHT = 192, 108
    focal_length = 100
    principal_point = [WIDTH/2, HEIGHT/2]

    all_positions = []
    frame_indices = []
    for idx, frame in enumerate(valid_frames):
        if frame["camera0"]:
            all_positions.append(frame["camera0"]["position"])
            frame_indices.append(idx)
        if frame["camera1"]:
            all_positions.append(frame["camera1"]["position"])
            frame_indices.append(idx)
    # threading.Thread(
    #     target=sync_camera_loop,
    #     args=(all_camera_positions,),
    #     daemon=True
    # ).start()

    # kdtree = KDTree(all_positions)

    # blueprint = rr.blueprint(
    #     rr.SpaceView(
    #         name="3D View",
    #         space="world",
    #         contents=["world/**"],
    #     ),
    #     auto_space_views=False
    # )
    # rr.send_blueprint(blueprint)

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
                    camera_xyz=rr.ViewCoordinates.RDF,
                    image_plane_distance=0.2
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
                    camera_xyz=rr.ViewCoordinates.RDF,
                    image_plane_distance=0.2
                )
            )
            
            rr.log(
                "world/camera1/image_in_space",
                rr.Image(img).compress(jpeg_quality=75)
            )

    while True:
        try:
            # 用户输入
            user_input = input("Enter frame index and camera (e.g. '0 left'), or 'q' to quit: ")
            if user_input.lower() == 'q':
                break

            idx_str, camera = user_input.split()
            idx = int(idx_str)
            if idx not in index_to_frame_id:
                print(f"索引 {idx} 超出范围！可用范围: 0 ~ {len(all_frame_ids)-1}")
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