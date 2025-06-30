import abc
import dataclasses
import json
from pathlib import Path
import struct
from typing import Generic, Iterable, List, Tuple, Type, TypeVar

import numpy as np
import open3d as o3d
from rosbags.highlevel import AnyReader
from rosbags.typesys.stores.ros1_noetic import (
    sensor_msgs__msg__Imu as Imu,
    sensor_msgs__msg__CompressedImage as CompressedImage,
    sensor_msgs__msg__PointCloud2 as PointCloud2,
)
from tabulate import tabulate

from calib import Calibration


T = TypeVar("T")


class Topic(Generic[T], abc.ABC):
    """Generic abstract class for ROS topics."""

    @classmethod
    @abc.abstractmethod
    def name(cls) -> str:
        pass


class ImuTopic(Topic[Imu]):
    @classmethod
    def name(cls) -> str:
        return "/livox/imu"


class CameraLeftTopic(Topic[CompressedImage]):
    @classmethod
    def name(cls) -> str:
        return "/camera/left/jpeg"


class CameraRightTopic(Topic[CompressedImage]):
    @classmethod
    def name(cls) -> str:
        return "/camera/right/jpeg"


class LidarTopic(Topic[PointCloud2]):
    @classmethod
    def name(cls) -> str:
        return "/livox/lidar"


@dataclasses.dataclass
class MetaCamEduLidarField:
    """Dataclass to represent a field in the Lidar message."""

    x: float
    y: float
    z: float
    intensity: float
    tag: int
    line: int
    timestamp: float

    @classmethod
    def unpack(cls, data: bytes) -> "MetaCamEduLidarField":
        """Unpack a byte array into a MetaCamEduLidarField object.

        The unpacking is according to the following structure
        which is found from `MetaCamEduSyncedMsgs.lidar_field_table`:
        | name      |   offset | datatype    |   count |
        |-----------|----------|-------------|---------|
        | x         |        0 | FLOAT32 (7) |       1 |
        | y         |        4 | FLOAT32 (7) |       1 |
        | z         |        8 | FLOAT32 (7) |       1 |
        | intensity |       12 | FLOAT32 (7) |       1 |
        | tag       |       16 | UINT8 (2)   |       1 |
        | line      |       17 | UINT8 (2)   |       1 |
        | timestamp |       18 | FLOAT64 (8) |       1 |

        Args:
            data (bytes): The byte array containing the field data.

        Returns:
            MetaCamEduLidarField: An instance of MetaCamEduLidarField.
        """
        x, y, z, intensity, tag, line, timestamp = struct.unpack("<ffffBBd", data)
        return cls(
            x=x,
            y=y,
            z=z,
            intensity=intensity,
            tag=tag,
            line=line,
            timestamp=timestamp,
        )

    @classmethod
    def unpack_array(cls, data: bytes) -> list["MetaCamEduLidarField"]:
        """Unpack a byte array into a list of MetaCamEduLidarField objects.

        Args:
            data (bytes): The byte array containing the field data.

        Returns:
            list[MetaCamEduLidarField]: A list of MetaCamEduLidarField objects.
        """
        stride = struct.calcsize("<ffffBBd")
        return [cls.unpack(data[i : i + stride]) for i in range(0, len(data), stride)]


@dataclasses.dataclass
class MetaCamEduSyncedMsgs:
    """Dataclass to hold synced messages from MetaCam EDU.

    Except IMU, other messages may not be available at the same timestamp
    as the IMU message. As a result, they may be `None`, and if available,
    they will have an offset in the form of an integer representing the
    difference in timestamps from the IMU message.
    """

    imu: Imu

    camera_left: CompressedImage | None = None
    camera_left_offset: int | None = None

    camera_right: CompressedImage | None = None
    camera_right_offset: int | None = None

    lidar: PointCloud2 | None = None
    lidar_offset: int | None = None

    def pcd(self) -> o3d.geometry.PointCloud | None:
        """Convert the lidar message to a Open3D PointCloud object.

        Returns:
            o3d.geometry.PointCloud | None: The PointCloud object if lidar data is
                available, otherwise None.
        """
        if self.lidar is None:
            return None
        return self.rosbag_to_o3d_pcd(self.lidar)

    @staticmethod
    def rosbag_to_o3d_pcd(rosbag: PointCloud2) -> o3d.geometry.PointCloud:
        """Convert a ROS PointCloud2 message to an Open3D PointCloud object.

        Args:
            rosbag (PointCloud2): The ROS PointCloud2 message.

        Returns:
            o3d.geometry.PointCloud: The Open3D PointCloud object.
        """
        fields = MetaCamEduLidarField.unpack_array(rosbag.data)

        points = np.array([[field.x, field.y, field.z] for field in fields])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        if rosbag.fields:
            intensity = np.array([field.intensity for field in fields])
            pcd.colors = o3d.utility.Vector3dVector(
                np.tile(intensity[:, np.newaxis], (1, 3)) / 255.0
            )

        return pcd

    def lidar_field_table(self) -> str | None:
        """Get a formatted table of lidar fields.

        Returns:
            str: A string representation of the lidar fields.
        """
        if self.lidar is None:
            return None
        return self.pcd_field_table(self.lidar)

    @staticmethod
    def pcd_field_table(pcd: PointCloud2) -> str:
        """Get a formatted table of fields in the PointCloud2 message.

        Returns:
            str: A string representation of the PointCloud2 fields.
        """
        datatype_map = {
            1: "INT8",
            2: "UINT8",
            3: "INT16",
            4: "UINT16",
            5: "INT32",
            6: "UINT32",
            7: "FLOAT32",
            8: "FLOAT64",
        }
        field_table = [
            {
                "name": field.name,
                "offset": field.offset,
                "datatype": f"{datatype_map[field.datatype]} ({field.datatype})",
                "count": field.count,
            }
            for field in pcd.fields
        ]
        return tabulate(field_table, headers="keys", tablefmt="grid")


@dataclasses.dataclass
class MetaCamEduReader:
    """Class for managing MetaCam EDU data."""

    path: Path
    rosbag_reader: AnyReader
    calib: Calibration
    preview_pcd: o3d.geometry.PointCloud

    def __init__(self, path: Path | str):
        """Initialize the MetaCamEdu with the path to the ROS bag file.

        Args:
            path (Path | str): The path to the output of MetaCam EDU.
        """
        self.path = Path(path)
        # Collect all .bag files in the data directory, sorted for order
        bag_files = sorted((self.path / "data").glob("*.bag"))
        if not bag_files:
            raise FileNotFoundError(f"No .bag files found in {self.path / 'data'}")
        self.rosbag_reader = AnyReader(bag_files)
        with open(self.path / "info" / "calibration.json", "r") as f:
            self.calib = Calibration.from_dict(json.load(f))
        self.preview_pcd = o3d.io.read_point_cloud(self.path / "Preview.pcd")

    def __enter__(self):
        """Enter the runtime context related to this object."""
        self.rosbag_reader.__enter__()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Exit the runtime context related to this object."""
        self.rosbag_reader.__exit__(exc_type, exc_value, traceback)

    def topic_table(self) -> str:
        """Get a formatted table of topics in the ROS bag.

        Returns:
            str: A string representation of the topic table.
        """
        connection_table = [
            {"topic": c.topic, "msgtype": c.msgtype, "msgcount": c.msgcount}
            for c in self.rosbag_reader.connections
        ]
        return tabulate(connection_table, headers="keys", tablefmt="grid")

    def topic_msgs(self, topic: Type[Topic[T]]) -> Iterable[Tuple[int, T]]:
        """Iterate over messages of a specific topic.

        Args:
            topic (Type[Topic[T]]): The topic class to filter messages by.

        Yields:
            Tuple[int, T]: A tuple containing the timestamp and the message.
        """
        return (
            (
                timestamp,
                self.rosbag_reader.typestore.deserialize_ros1(
                    rawdata, connection.msgtype
                ),
            )
            for connection, timestamp, rawdata in self.rosbag_reader.messages()
            if connection.topic == topic.name()
        )

    def synced_msgs(self) -> Iterable[Tuple[int, MetaCamEduSyncedMsgs]]:
        """Iterate over synced messages from the MetaCam EDU data.

        Since IMU yields higher frequency than camera and lidar, this method
        synchronizes the messages based on the IMU timestamps.
        If a message from a non-IMU topic has a timestamp t_msg such that
        t_IMU1 <= t_msg < t_IMU2 (where t_IMU1 is current IMU timestamp and
        t_IMU2 is next IMU timestamp), then msg is stored with IMU1.

        Yields:
            Tuple[int, MetaCamEduSyncedMsgs]: A tuple containing the IMU timestamp and a
            MetaCamEduSyncedMsgs object containing the synced messages.
        """
        imu_topic_iter = self.topic_msgs(ImuTopic)
        cam_l_topic_iter = self.topic_msgs(CameraLeftTopic)
        cam_r_topic_iter = self.topic_msgs(CameraRightTopic)
        lidar_topic_iter = self.topic_msgs(LidarTopic)

        try:
            imu_t, imu_msg = next(imu_topic_iter)
        except StopIteration:
            return

        # Cam left
        try:
            cam_l_t, cam_l_msg = next(cam_l_topic_iter)
        except StopIteration:
            cam_l_t, cam_l_msg = float("inf"), None

        # Cam right
        try:
            cam_r_t, cam_r_msg = next(cam_r_topic_iter)
        except StopIteration:
            cam_r_t, cam_r_msg = float("inf"), None

        # Lidar
        try:
            lidar_t, lidar_msg = next(lidar_topic_iter)
        except StopIteration:
            lidar_t, lidar_msg = float("inf"), None

        while True:
            try:
                next_imu_t, next_imu_msg = next(imu_topic_iter)
            except StopIteration:
                next_imu_t, next_imu_msg = float("inf"), None

            synced_msgs = MetaCamEduSyncedMsgs(imu=imu_msg)

            # Cam left
            while cam_l_t < imu_t:
                try:
                    cam_l_t, cam_l_msg = next(cam_l_topic_iter)
                except StopIteration:
                    cam_l_t, cam_l_msg = float("inf"), None

            if cam_l_t < next_imu_t:
                synced_msgs.camera_left = cam_l_msg
                synced_msgs.camera_left_offset = cam_l_t - imu_t

            # Cam right
            while cam_r_t < imu_t:
                try:
                    cam_r_t, cam_r_msg = next(cam_r_topic_iter)
                except StopIteration:
                    cam_r_t, cam_r_msg = float("inf"), None

            if cam_r_t < next_imu_t:
                synced_msgs.camera_right = cam_r_msg
                synced_msgs.camera_right_offset = cam_r_t - imu_t

            # Lidar
            while lidar_t < imu_t:
                try:
                    lidar_t, lidar_msg = next(lidar_topic_iter)
                except StopIteration:
                    lidar_t, lidar_msg = float("inf"), None

            if lidar_t < next_imu_t:
                synced_msgs.lidar = lidar_msg
                synced_msgs.lidar_offset = lidar_t - imu_t

            yield imu_t, synced_msgs

            if next_imu_t == float("inf"):
                break

            imu_t, imu_msg = next_imu_t, next_imu_msg

    def synced_msg_with_framerate(
        self, framerate: float
    ) -> Iterable[Tuple[int, MetaCamEduSyncedMsgs]]:
        """Iterate over synced messages with a specified framerate.

        This method guarantees all fields in the returned
        MetaCamEduSyncedMsgs object are filled, even if some messages
        are not available at the current IMU timestamp, it will fill
        them with the previous message.

        Args:
            framerate (float): The desired framerate in fps for the synced messages.

        Yields:
            Tuple[int, MetaCamEduSyncedMsgs]: A tuple containing the IMU timestamp and a
            MetaCamEduSyncedMsgs object containing the synced messages.
        """
        imu_topic_iter = self.topic_msgs(ImuTopic)
        cam_l_topic_iter = self.topic_msgs(CameraLeftTopic)
        cam_r_topic_iter = self.topic_msgs(CameraRightTopic)
        lidar_topic_iter = self.topic_msgs(LidarTopic)

        try:
            curr_t, imu_msg = next(imu_topic_iter)
            _, cam_l_msg = next(cam_l_topic_iter)
            _, cam_r_msg = next(cam_r_topic_iter)
            _, lidar_msg = next(lidar_topic_iter)
        except StopIteration:
            return

        synced_msgs = MetaCamEduSyncedMsgs(
            imu=imu_msg,
            camera_left=cam_l_msg,
            camera_right=cam_r_msg,
            lidar=lidar_msg,
        )

        for next_t, next_synced_msgs in self.synced_msgs():
            if (next_t - curr_t) / 1e9 >= 1 / framerate:
                yield curr_t, synced_msgs
                curr_t = next_t

            synced_msgs = MetaCamEduSyncedMsgs(
                imu=next_synced_msgs.imu,
                camera_left=next_synced_msgs.camera_left or synced_msgs.camera_left,
                camera_right=next_synced_msgs.camera_right or synced_msgs.camera_right,
                lidar=next_synced_msgs.lidar or synced_msgs.lidar,
            )
