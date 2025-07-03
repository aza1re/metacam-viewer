import sys
import os
import open3d as o3d
from pathlib import Path

def convert_pcd_to_ply(pcd_path, ply_path):
    pcd = o3d.io.read_point_cloud(str(pcd_path))
    if not pcd.has_points():
        print(f"Warning: {pcd_path} has no points, skipping.")
        return
    o3d.io.write_point_cloud(str(ply_path), pcd)
    print(f"Converted: {pcd_path} -> {ply_path}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 pcd_to_ply.py <pcd file or folder>")
        sys.exit(1)
    input_path = Path(sys.argv[1])
    if input_path.is_file() and input_path.suffix.lower() == ".pcd":
        ply_path = input_path.with_suffix(".ply")
        convert_pcd_to_ply(input_path, ply_path)
    elif input_path.is_dir():
        for pcd_file in input_path.glob("*.pcd"):
            ply_file = pcd_file.with_suffix(".ply")
            convert_pcd_to_ply(pcd_file, ply_file)
    else:
        print("Input must be a .pcd file or a folder containing .pcd files.")

if __name__ == "__main__":
    main()