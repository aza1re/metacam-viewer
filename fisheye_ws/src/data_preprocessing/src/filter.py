import open3d as o3d
pcd = o3d.io.read_point_cloud("/home/frankye/Projects/fisheye_ws/output/poster/cloud_group_1.pcd")
pcd.remove_non_finite_points()
o3d.io.write_point_cloud("/home/frankye/Projects/fisheye_ws/output/poster/cloud_group_1.ply", pcd)