""" Demo to show prediction results.
    Author: chenxi-wang
"""

import os
import sys

import numpy as np
import open3d as o3d
import scipy.io as scio
import torch
from graspnetAPI import GraspGroup
from PIL import Image

from .collision_detector import ModelFreeCollisionDetector
from .data_utils import CameraInfo, create_point_cloud_from_depth_image
from .graspnet import GraspNet, pred_decode


def get_net(checkpoint_path, num_view=300):
    """Init the model and load checkpoint."""
    net = GraspNet(input_feature_dim=0, num_view=num_view, num_angle=12, num_depth=4,
                   cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01, 0.02, 0.03, 0.04],
                   is_training=False)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net.to(device)
    checkpoint = torch.load(checkpoint_path, map_location=device)
    net.load_state_dict(checkpoint['model_state_dict'])
    start_epoch = checkpoint['epoch']
    print("-> loaded checkpoint %s (epoch: %d)" % (checkpoint_path, start_epoch))
    net.eval()
    return net


def get_and_process_data(data_dir, num_point=20000):
    """Load data and generate point cloud."""
    color = np.array(Image.open(os.path.join(data_dir, 'color.png')), dtype=np.float32) / 255.0
    depth = np.array(Image.open(os.path.join(data_dir, 'depth.png')))
    workspace_mask = np.array(Image.open(os.path.join(data_dir, 'workspace_mask.png')))
    meta = scio.loadmat(os.path.join(data_dir, 'meta.mat'))
    intrinsic = meta['intrinsic_matrix']
    factor_depth = meta['factor_depth']

    camera = CameraInfo(1280.0, 720.0, intrinsic[0][0], intrinsic[1][1],
                        intrinsic[0][2], intrinsic[1][2], factor_depth)
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    mask = (workspace_mask & (depth > 0))
    cloud_masked = cloud[mask]
    color_masked = color[mask]

    if len(cloud_masked) >= num_point:
        idxs = np.random.choice(len(cloud_masked), num_point, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(len(cloud_masked), num_point - len(cloud_masked), replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    cloud_o3d = o3d.geometry.PointCloud()
    cloud_o3d.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    cloud_o3d.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))

    end_points = dict()
    cloud_sampled_t = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    cloud_sampled_t = cloud_sampled_t.to(device)
    end_points['point_clouds'] = cloud_sampled_t
    end_points['cloud_colors'] = color_sampled

    return end_points, cloud_o3d


def get_grasps(net, end_points):
    """Forward pass and decode predictions."""
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)
    gg_array = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(gg_array)
    return gg


def collision_detection(gg, cloud, voxel_size=0.01, collision_thresh=0.01):
    """Filter out colliding grasps."""
    mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=voxel_size)
    collision_mask = mfcdetector.detect(gg, approach_dist=0.05,
                                        collision_thresh=collision_thresh)
    gg = gg[~collision_mask]
    return gg


def vis_grasps(gg, cloud):
    """Visualize top 50 grasps."""
    gg.nms()
    gg.sort_by_score()
    gg = gg[:50]
    grippers = gg.to_open3d_geometry_list()
    o3d.visualization.draw_geometries([cloud, *grippers])


def demo(checkpoint_path, data_dir, num_point=20000, num_view=300,
         collision_thresh=0.01, voxel_size=0.01):
    """Run full grasp detection demo."""
    net = get_net(checkpoint_path, num_view)
    end_points, cloud = get_and_process_data(data_dir, num_point)
    gg = get_grasps(net, end_points)
    if collision_thresh > 0:
        gg = collision_detection(gg, np.array(cloud.points), voxel_size, collision_thresh)
    vis_grasps(gg, cloud)


def main():
    """Entry point for command-line usage."""
    import argparse

    parser = argparse.ArgumentParser(description='GraspNet Demo')
    parser.add_argument('--checkpoint_path', required=True, help='Model checkpoint path')
    parser.add_argument('--data_dir', required=True, help='Data directory')
    parser.add_argument('--num_point', type=int, default=20000,
                        help='Point Number [default: 20000]')
    parser.add_argument('--num_view', type=int, default=300,
                        help='View Number [default: 300]')
    parser.add_argument('--collision_thresh', type=float, default=0.01,
                        help='Collision Threshold [default: 0.01]')
    parser.add_argument('--voxel_size', type=float, default=0.01,
                        help='Voxel Size [default: 0.01]')
    args, _ = parser.parse_known_args()

    demo(
        checkpoint_path=args.checkpoint_path,
        data_dir=args.data_dir,
        num_point=args.num_point,
        num_view=args.num_view,
        collision_thresh=args.collision_thresh,
        voxel_size=args.voxel_size,
    )


if __name__ == '__main__':
    main()