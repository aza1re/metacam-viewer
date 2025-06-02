from typing import Iterable
import numpy as np
import open3d as o3d
import tqdm


def icp_pcd(
    pcds: Iterable[o3d.geometry.PointCloud],
    # the following hardcoded parameters are just guesses
    max_correspondence_distance: float = 0.01,
    estimation_method: o3d.pipelines.registration.TransformationEstimation = o3d.pipelines.registration.TransformationEstimationPointToPoint(),
) -> np.ndarray:
    """Perform ICP registration on the point clouds.

    This method cumulatively performs ICP registration on point clouds
    over time.

    Args:
        pcds (Iterable[o3d.geometry.PointCloud]): An iterable of point clouds to register
            in chronological order.
        max_correspondence_distance (float): The maximum distance between correspondences
            for the ICP algorithm.
        estimation_method (o3d.pipelines.registration.TransformationEstimation): The method
            used to estimate the transformation between point clouds.

    Returns:
        np.ndarray: The transformations to each point cloud with a shape (n, 4, 4),
        where n is the number of point clouds.
    """
    transforms = np.zeros((len(pcds), 4, 4), dtype=np.float32)
    transforms[0] = np.eye(4, dtype=np.float32)
    for i, (prev_pcd, curr_pcd) in tqdm.tqdm(
        enumerate(zip(pcds, pcds[1:])), "Registering point clouds", len(pcds) - 1
    ):
        reg = o3d.pipelines.registration.registration_icp(
            source=curr_pcd,
            target=prev_pcd,
            max_correspondence_distance=max_correspondence_distance,
            estimation_method=estimation_method,
        )
        transforms[i + 1] = reg.transformation @ transforms[i]
    return transforms


def create_icp_pcd(pcds: Iterable[o3d.geometry.PointCloud]) -> o3d.geometry.PointCloud:
    """Create a single point cloud from multiple point clouds using ICP registration.

    Args:
        pcds (Iterable[o3d.geometry.PointCloud]): An iterable of point clouds to register
            in chronological order.

    Returns:
        o3d.geometry.PointCloud: The registered point cloud.
    """
    transforms = icp_pcd(pcds)
    points = []
    colors = []
    for pcd, transform in zip(pcds, transforms):
        pcd_transformed = pcd.transform(transform)
        points.append(np.asarray(pcd_transformed.points))
        colors.append(np.asarray(pcd_transformed.colors))
    points = np.concatenate(points, axis=0)
    colors = np.concatenate(colors, axis=0)

    combined_pcd = o3d.geometry.PointCloud()
    combined_pcd.points = o3d.utility.Vector3dVector(points)
    combined_pcd.colors = o3d.utility.Vector3dVector(colors)
    return combined_pcd
