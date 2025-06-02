import dataclasses
from typing import List, Dict, Any


@dataclasses.dataclass
class Transform:
    position: List[float]
    rotation: List[List[float]] | None = None


@dataclasses.dataclass
class CameraIntrinsic:
    fl_x: float
    fl_y: float
    cx: float
    cy: float


@dataclasses.dataclass
class DistortionParams:
    k1: float
    k2: float
    k3: float
    k4: float


@dataclasses.dataclass
class CameraDistortion:
    camera_model: str
    params: DistortionParams

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CameraDistortion":
        return cls(
            camera_model=data["camera_model"], params=DistortionParams(**data["params"])
        )


@dataclasses.dataclass
class CameraCalibration:
    name: str
    type: str
    width: int
    height: int
    intrinsic: CameraIntrinsic
    distortion: CameraDistortion
    transform_from_lidar: Transform

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CameraCalibration":
        return cls(
            name=data["name"],
            type=data["type"],
            width=data["width"],
            height=data["height"],
            intrinsic=CameraIntrinsic(**data["intrinsic"]),
            distortion=CameraDistortion.from_dict(data["distortion"]),
            transform_from_lidar=Transform(**data["transform_from_lidar"]),
        )


@dataclasses.dataclass
class ImuCalibration:
    transform_from_lidar: Transform

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ImuCalibration":
        return cls(transform_from_lidar=Transform(**data["transform_from_lidar"]))


@dataclasses.dataclass
class RtkCalibration:
    transform_from_lidar: Transform

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "RtkCalibration":
        return cls(transform_from_lidar=Transform(**data["transform_from_lidar"]))


@dataclasses.dataclass
class Calibration:
    calibration_time: str
    version: str
    cameras: List[CameraCalibration]
    imu: ImuCalibration
    rtk: RtkCalibration

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Calibration":
        return cls(
            calibration_time=data["calibration_time"],
            version=data["version"],
            cameras=[
                CameraCalibration.from_dict(cam_data) for cam_data in data["cameras"]
            ],
            imu=ImuCalibration.from_dict(data["imu"]),
            rtk=RtkCalibration.from_dict(data["rtk"]),
        )
