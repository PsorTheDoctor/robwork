#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import time

from point_estimation import *
import helpers
import settings

indices = settings.indices
noise_levels = settings.noise_levels


def outlier_removal(input_pcd):
    pcd, idx = input_pcd.remove_statistical_outlier(nb_neighbors=200, std_ratio=0.01)
    return pcd


def main():
    i = 0
    for index in indices:
        for iteration in range(0, 5):
            for noise_sigma in noise_levels:
                if index != indices[0] and noise_sigma != 0.0:
                    continue

                scene_pointcloud_file_name = settings.input_folder + 'scene_' + str(index) + '.pcd'
                scene_pointcloud = o3d.io.read_point_cloud(scene_pointcloud_file_name)
                
                scene_pointcloud_noisy = helpers.add_noise(scene_pointcloud, 0, noise_sigma)
                
                object_mesh = o3d.io.read_triangle_mesh(settings.input_folder + "obj_000009_mm.stl")
                object_pointcloud = object_mesh.sample_points_poisson_disk(10000)

                # o3d.visualization.draw_geometries([object_pointcloud, scene_pointcloud_noisy], window_name='Pre alignment')



                start = time.time()
                estimated_pose = global_pose_estimation(scene_pointcloud_noisy, object_pointcloud)
                end = time.time()
                i += 1
                elapsed_time = end - start
                remained_time = elapsed_time * (len(indices) * 5 * len(noise_levels) - i)
                minutes, seconds = divmod(remained_time, 60)
                print(f'Remained time: {round(minutes)}min {round(seconds)}sec')

                np.savetxt(settings.results_folder + 'estimate_' + str(index) + '_' + str(noise_sigma) + '_' + str(iteration) + '.txt',
                        estimated_pose)        

                # print("Final pose")
                # print (estimated_pose)

                ground_truth = np.loadtxt(settings.input_folder + 'gt_' + str(index) + '.txt')

                # print("Ground truth")
                # print(ground_truth)

                error_angle, error_pos = helpers.computeError(ground_truth, estimated_pose)
            
                print(index, iteration, noise_sigma, error_angle, error_pos)

                f = open(settings.results_folder + 'time_' + str(index) + '_' + str(noise_sigma) + '_' + str(iteration) + '.txt', "w")
                f.write(str(elapsed_time) + '\n'
                        )
                f.close()

                # object_pointcloud.colors = o3d.utility.Vector3dVector(np.zeros_like(object_pointcloud.points) + [255,0,0])
                # o3d.visualization.draw_geometries([
                #     object_pointcloud.transform(estimated_pose), scene_pointcloud_noisy], window_name='Final alignment'
                # )


if __name__ == "__main__":
    main()
