from pathlib import Path
import sys
import open3d as o3d
import tqdm

from processing import create_icp_pcd
from reader import LidarTopic, MetaCamEduReader, MetaCamEduSyncedMsgs


def pcd_viewer(pcd: o3d.geometry.PointCloud):
    """Visualize a PointCloud object using Open3D.

    Args:
        pcd (o3d.geometry.PointCloud): The PointCloud object to visualize.
    """

    def print_extrinsic(vis):
        extrinsic = (
            vis.get_view_control().convert_to_pinhole_camera_parameters().extrinsic
        )
        print("Extrinsic matrix:")
        print("[")
        for row in extrinsic:
            print(f"   [{row[0]:.8f}, {row[1]:.8f}, {row[2]:.8f}, {row[3]:.8f}],")
        print("]")
        return False

    key_to_callback = {
        ord("E"): print_extrinsic,
    }

    o3d.visualization.draw_geometries_with_key_callbacks([pcd], key_to_callback)


def main():
    if len(sys.argv) < 2:
        print("Usage: python viewer_main.py <path_to_data_folder> [--preview]")
        return

    path = Path(sys.argv[1])
    preview = "--preview" in sys.argv

    if preview:
        pcd = o3d.io.read_point_cloud(path / "Preview.pcd")
    elif (path / "output.pcd").exists():
        pcd = o3d.io.read_point_cloud(path / "output.pcd")
    else:
        with MetaCamEduReader(path) as reader:
            msgs_list = [msg for _, msg in reader.topic_msgs(LidarTopic)]
            pcds = [
                MetaCamEduSyncedMsgs.rosbag_to_o3d_pcd(msgs)
                for msgs in tqdm.tqdm(msgs_list, "Loading point clouds", len(msgs_list))
            ]
            pcd = create_icp_pcd(pcds)
            o3d.io.write_point_cloud(path / "output.pcd", pcd)

    pcd_viewer(pcd)


if __name__ == "__main__":
    main()
