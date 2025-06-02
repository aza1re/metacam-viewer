import multiprocessing
from pathlib import Path
import sys
from typing import List, Tuple

import cv2
import numpy as np
import open3d as o3d
import tqdm

from reader import MetaCamEduReader, MetaCamEduSyncedMsgs


# Hardcoded extrinsics to view the point cloud from behind and below the origin
# (because the lidar is mostly pointing 45 degrees downwards).
CAM_EXTRINSIC = np.array(
    [
        [0.04517296, -0.99879606, 0.01912683, -0.00992348],
        [0.71837172, 0.01917336, -0.69539517, 0.23546977],
        [0.69419123, 0.04515323, 0.71837296, 3.17508674],
        [0.00000000, 0.00000000, 0.00000000, 1.00000000],
    ]
)


DISTANCE_TO_COLORS = [
    (0.0, np.array([255.0, 255.0, 255.0])),  # white
    (1.0, np.array([255.0, 0.0, 0.0])),  # red
    (2.0, np.array([255.0, 255.0, 0.0])),  # yellow
    (3.0, np.array([0.0, 255.0, 0.0])),  # green
    (4.0, np.array([0.0, 255.0, 255.0])),  # cyan
    (5.0, np.array([0.0, 0.0, 255.0])),  # blue
    (6.0, np.array([0.0, 0.0, 0.0])),  # black
]


def color_pcd(points: np.ndarray) -> np.ndarray:
    """Color the point cloud based on distance from the Lidar.

    Args:
        points (np.ndarray): The point cloud points.

    Returns:
        np.ndarray: The colors
    """
    colors = np.ones((points.shape[0], 3), dtype=np.float32)
    for i, point in enumerate(points):
        distance = np.linalg.norm(np.asarray(point))
        for (da, ca), (db, cb) in zip(DISTANCE_TO_COLORS, DISTANCE_TO_COLORS[1:]):
            if da <= distance < db:
                t = (distance - da) / (db - da)
                color = (1 - t) * ca + t * cb
                colors[i, :] = color / 255.0
                break
        else:
            colors[i, :] = np.array([0.0, 0.0, 0.0])

    return colors


def create_pcd_frame(
    pcd: o3d.geometry.PointCloud,
    size: Tuple[int, int],
) -> np.ndarray:
    """Create a frame from a PointCloud object.

    Args:
        pcd (o3d.geometry.PointCloud): The PointCloud object to convert.
        size (Tuple[int, int]): The size of the output image (width, height).

    Returns:
        np.ndarray: A 2D numpy array representing the point cloud.
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=size[0], height=size[1], visible=False, left=0, top=0)
    vis.get_render_option().show_coordinate_frame = True
    vis.get_render_option().background_color = np.array([0.5, 0.5, 0.5])
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5))
    vis.add_geometry(pcd)

    ctr = vis.get_view_control().convert_to_pinhole_camera_parameters()
    ctr.extrinsic = CAM_EXTRINSIC
    vis.get_view_control().convert_from_pinhole_camera_parameters(ctr)

    vis.poll_events()
    vis.update_renderer()

    img = vis.capture_screen_float_buffer(do_render=True)
    vis.destroy_window()

    img = (np.asarray(img) * 255.0).astype(np.uint8)
    return img


def create_frame(
    synced_msgs: MetaCamEduSyncedMsgs,
    pcd: o3d.geometry.PointCloud | None = None,
) -> np.ndarray:
    """Create a combined frame from synced messages.

    Args:
        synced_msgs (MetaCamEduSyncedMsgs): Synced messages containing camera data.
        pcd (o3d.geometry.PointCloud, optional): Point cloud data to render. Defaults to None.

    Returns:
        np.ndarray: Combined image frame with rendered point cloud, left camera image,
            and right camera image.
    """
    left_img = cv2.imdecode(
        np.frombuffer(synced_msgs.camera_left.data, np.uint8), cv2.IMREAD_COLOR
    )
    right_img = cv2.imdecode(
        np.frombuffer(synced_msgs.camera_right.data, np.uint8), cv2.IMREAD_COLOR
    )
    pcd_img = create_pcd_frame(
        pcd or synced_msgs.pcd(),
        size=(left_img.shape[1] + right_img.shape[1], left_img.shape[0]),
    )
    combined_img = np.hstack((left_img, right_img, pcd_img))
    return combined_img


def main():
    if len(sys.argv) < 2:
        print(
            "Usage: python video_main.py <path_to_data_folder> [--first-frame] [--no-color]"
        )
        return

    path = Path(sys.argv[1])
    first_frame = "--first-frame" in sys.argv
    no_color = "--no-color" in sys.argv

    with MetaCamEduReader(path) as reader:
        # Frame
        framerate = 30.0

        msgs_list: List[MetaCamEduSyncedMsgs]
        if first_frame:
            msgs_list = [next(reader.synced_msg_with_framerate(framerate))[1]]
        else:
            msgs_list = [msg for _, msg in reader.synced_msg_with_framerate(framerate)]

        pcds = [
            msgs.pcd()
            for msgs in tqdm.tqdm(msgs_list, "Loading point clouds", len(msgs_list))
        ]

        if not no_color:
            with multiprocessing.Pool() as p:
                colors = list(
                    tqdm.tqdm(
                        p.imap(color_pcd, (np.asarray(pcd.points) for pcd in pcds)),
                        "Coloring point clouds",
                        len(msgs_list),
                    )
                )

            for pcd, color in zip(pcds, colors):
                pcd.colors = o3d.utility.Vector3dVector(color)

        msgs = msgs_list[0]
        pcd = pcds[0]

        frame = create_frame(msgs, pcd)
        size = frame.shape[:2][::-1]

        cv2.imwrite(path / "output.jpg", frame)
        print("First frame saved as output.jpg")

        if first_frame:
            return

        # Video
        #
        # For some reason, the video output is quite large
        # and seems to have some wrong configuration, making
        # it for example can't be sent through WhatsApp.
        #
        # So it's recommended to do an ffmpeg without any changes:
        # `ffmpeg -i output.mp4 transcoded.mp4`
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        video = cv2.VideoWriter(
            path / "output.mp4",
            fourcc,
            framerate,
            size,
        )

        for msgs, pcd in tqdm.tqdm(
            zip(msgs_list, pcds),
            "Processing frames",
            len(msgs_list),
        ):
            frame = create_frame(msgs, pcd)
            video.write(frame)

        video.release()
        print("Video saved as output.mp4")


if __name__ == "__main__":
    main()
