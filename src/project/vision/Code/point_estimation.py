import open3d as o3d
import numpy as np
from tqdm import tqdm
import random


def global_pose_estimation(scene, obj):
    tree = o3d.geometry.KDTreeSearchParamKNN(knn=10)
    obj.estimate_normals(tree)
    scene.estimate_normals(tree)

    tree2 = o3d.geometry.KDTreeSearchParamRadius(0.05)
    obj_features = o3d.pipelines.registration.compute_fpfh_feature(obj, tree2)
    scene_features = o3d.pipelines.registration.compute_fpfh_feature(obj, tree2)

    obj_features = np.asarray(obj_features.data).T
    scene_features = np.asarray(scene_features.data).T

    corr = o3d.utility.Vector2iVector()
    for j in tqdm(range(obj_features.shape[0]), desc='Correspondences'):
        fobj = obj_features[j]
        dist = np.sum((fobj - scene_features) ** 2, axis=-1)
        kmin = np.argmin(dist)
        corr.append((j, kmin))

    tree3 = o3d.geometry.KDTreeFlann(scene)
    iters = 100
    thresh_squared = 0.01 ** 2
    random.seed(123456789)
    inliers_best = 0
    for _ in tqdm(range(iters), desc='RANSAC'):
        corr_ = o3d.utility.Vector2iVector(random.choices(corr, k=3))
        est = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        T = est.compute_transformation(obj, scene, corr_)

        obj_aligned = o3d.geometry.PointCloud(obj)
        obj_aligned.transform(T)

        inliers = 0
        for j in range(len(obj_aligned.points)):
            k, idx, dist = tree3.search_knn_vector_3d(obj_aligned.points[j], 1)
            if dist[0] < thresh_squared:
                inliers += 1

        if inliers > inliers_best:
            print(f'Got a new model with {inliers}/{len(obj_aligned.points)} inliers!')
            inliers_best = inliers
            pose = T

    print('Global pose estimation:', pose)
    return pose


def local_pose_estimation(scene, obj, init_pose=None):
    tree = o3d.geometry.KDTreeFlann(scene)
    iters = 50
    thresh_squared = 0.01 ** 2
    pose = init_pose
    obj_aligned = o3d.geometry.PointCloud(obj)
    for _ in tqdm(range(iters), desc='ICP'):
        corr = o3d.utility.Vector2iVector()
        for j in range(len(obj_aligned.points)):
            k, idx, dist = tree.search_knn_vector_3d(obj_aligned.points[j], 1)
            if dist[0] < thresh_squared:
                corr.append((j, idx[0]))

        est = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        T = est.compute_transformation(obj_aligned, scene, corr)
        obj_aligned.transform(T)
        pose = T if pose is None else T @ pose

    print('Local pose estimation:', pose)
    return pose
