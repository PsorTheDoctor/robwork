#!/usr/bin/env python3
import open3d as o3d
import numpy as np

from point_estimation import *
import helpers
import settings

scene_id = settings.indices[0]
noise_level = settings.noise_levels[0]


def spatial_filter(input_pcd, z_min):
    filtered_pcd = input_pcd.crop(o3d.geometry.AxisAlignedBoundingBox(
        min_bound=(-float('inf'), -float('inf'), z_min),
        max_bound=(float('inf'), float('inf'), float('inf'))
    ))
    return filtered_pcd


def main():
    scene_pointcloud_file_name = settings.input_folder + 'scene_' + str(scene_id) + '.pcd'
    scene_pointcloud = o3d.io.read_point_cloud(scene_pointcloud_file_name)
    scene_pointcloud_noisy = helpers.add_noise(scene_pointcloud, 0, noise_level)

    # Filter shadow for global alignment
    scene_pointcloud_noisy = spatial_filter(scene_pointcloud_noisy, z_min=-5.0)
    
    object_mesh = o3d.io.read_triangle_mesh(settings.input_folder + "obj_000009_mm.stl")
    object_pointcloud = object_mesh.sample_points_poisson_disk(10000)

    o3d.visualization.draw_geometries([object_pointcloud, scene_pointcloud_noisy], window_name='Pre alignment')

    estimated_pose = global_pose_estimation(scene_pointcloud_noisy, object_pointcloud)

    # Filter table and shadow for local alignment
    scene_pointcloud_noisy = spatial_filter(scene_pointcloud_noisy, z_min=-0.0)

    o3d.visualization.draw_geometries([object_pointcloud, scene_pointcloud_noisy], window_name='Pre alignment')

    estimated_pose = local_pose_estimation(scene_pointcloud_noisy, object_pointcloud, init_pose=estimated_pose)

    ground_truth = np.loadtxt(settings.input_folder + "gt_" + str(scene_id) + ".txt")
    print("Ground truth:", ground_truth)
    print("Error:", helpers.computeError(ground_truth, estimated_pose))
 
    object_pointcloud.colors = o3d.utility.Vector3dVector(np.zeros_like(object_pointcloud.points) + [255, 0, 0])
    o3d.visualization.draw_geometries(
        [object_pointcloud.transform(estimated_pose), scene_pointcloud_noisy], window_name='Final alignment'
    )


if __name__ == "__main__":
    main()
