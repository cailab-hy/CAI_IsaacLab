import numpy as np
from pysdf import SDF
import torch
import trimesh
from urdfpy import URDF
import warp as wp

import os
import sys

base_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '.'))
sys.path.append(base_dir)

from soft_dtw_cuda import SoftDTW

def get_closest_state_idx(ref_traj, curr_ee_pos):
    """Find the index of the closest state in reference trajectory."""

    # ref_traj.shape = (num_trajs, traj_len, 3)
    num_trajs, traj_len = ref_traj.shape[0], ref_traj.shape[1]
    num_envs = curr_ee_pos.shape[0]

    # dist_from_all_state.shape = (num_envs, num_trajs, traj_len, 1)
    dist_from_all_state = torch.cdist(ref_traj.unsqueeze(0), curr_ee_pos.reshape(-1, 1, 1, 3), p=2)
    
    # dist_from_all_state_flatten.shape = (num_envs, num_trajs * traj_len)
    dist_from_all_state_flatten = dist_from_all_state.reshape(num_envs, -1)

    # min_dist_per_env.shape = (num_envs)
    min_dist_per_env = torch.amin(dist_from_all_state_flatten, dim=-1)

    # min_dist_idx.shape = (num_envs)
    min_dist_idx = torch.argmin(dist_from_all_state_flatten, dim=-1)

    # min_dist_traj_idx.shape = (num_envs)
    # min_dist_step_idx.shape = (num_envs)
    min_dist_traj_idx = min_dist_idx // traj_len
    min_dist_step_idx = min_dist_idx % traj_len

    return min_dist_traj_idx, min_dist_step_idx, min_dist_per_env

def get_reward_mask(ref_traj, curr_ee_pos, tolerance):
    
    _, min_dist_step_idx, _ = get_closest_state_idx(ref_traj, curr_ee_pos)
    selected_steps = torch.index_select(ref_traj, dim=1, index=min_dist_step_idx) # selected_steps.shape = (num_trajs, num_envs, 3)
    
    x_min = torch.amin(selected_steps[:,:,0], dim=0)-tolerance
    x_max = torch.amax(selected_steps[:,:,0], dim=0)+tolerance
    y_min = torch.amin(selected_steps[:,:,1], dim=0)-tolerance
    y_max = torch.amax(selected_steps[:,:,1], dim=0)+tolerance

    x_in_range = torch.logical_and(torch.lt(curr_ee_pos[:, 0], x_max), torch.gt(curr_ee_pos[:, 0], x_min))
    y_in_range = torch.logical_and(torch.lt(curr_ee_pos[:, 1], y_max), torch.gt(curr_ee_pos[:, 1], y_min))
    pos_in_range = torch.logical_and(x_in_range, y_in_range).int()

    return pos_in_range

"""
Soft Dynamic Time Warping
"""

def get_imitation_reward_from_dtw(ref_traj, curr_ee_pos, prev_ee_traj, criterion, device):
    """Get imitation reward based on dynamic time warping."""

    soft_dtw = torch.zeros((curr_ee_pos.shape[0]), device=device)
    prev_ee_pos = prev_ee_traj[:, 0, :].squeeze() # select the first ee pos in robot traj
    min_dist_traj_idx, min_dist_step_idx, min_dist_per_env = get_closest_state_idx(ref_traj, prev_ee_pos)

    for i in range(curr_ee_pos.shape[0]):
        traj_idx = min_dist_traj_idx[i]
        step_idx = min_dist_step_idx[i]
        curr_ee_pos_i = curr_ee_pos[i].reshape(1, 3)
        prev_ee_pos_i = prev_ee_pos[i].reshape(1, 3)

        # NOTE: in reference trajectories, larger index -> closer to goal
        traj = ref_traj[traj_idx, step_idx:, :].reshape((1, -1, 3))
        
        _, curr_step_idx, _ = get_closest_state_idx(traj, curr_ee_pos_i)

        if curr_step_idx==0:
            selected_pos = ref_traj[traj_idx, step_idx, :].reshape((1, 1, 3))
            selected_traj = torch.cat([selected_pos, selected_pos], dim=1)
        else:
            selected_traj = ref_traj[traj_idx, step_idx:(curr_step_idx+step_idx), :].reshape((1, -1, 3))
        # eef_traj = torch.cat([prev_ee_pos_i, curr_ee_pos_i], dim=0).reshape((1, -1, 3))
        eef_traj = torch.cat((prev_ee_traj[i, 1:, :], curr_ee_pos_i)).reshape((1, -1, 3))
        soft_dtw[i] = criterion(eef_traj, selected_traj)

    w_task_progress = 1-(min_dist_step_idx / ref_traj.shape[1])

    # imitation_rwd = torch.exp(-soft_dtw)
    imitation_rwd = 1-torch.tanh(soft_dtw)

    return imitation_rwd * w_task_progress

"""
Sampling-Based Curriculum (SBC)
"""

def get_curriculum_reward_scale(cfg_task, curr_max_disp):
    """Compute reward scale for SBC."""

    # Compute difference between max downward displacement at beginning of training (easiest condition)
    # and current max downward displacement (based on current curriculum stage)
    # NOTE: This number increases as curriculum gets harder
    curr_stage_diff = cfg_task.rl.curriculum_height_bound[1] - curr_max_disp

    # Compute difference between max downward displacement at beginning of training (easiest condition)
    # and min downward displacement (hardest condition)
    final_stage_diff = (
        cfg_task.rl.curriculum_height_bound[1] - cfg_task.rl.curriculum_height_bound[0]
    )

    # Compute reward scale
    reward_scale = curr_stage_diff / final_stage_diff + 1.0

    return reward_scale


def get_new_max_disp(curr_success, cfg_task, curriculum_height_bound, curriculum_height_step, curr_max_disp):
    """Update max downward displacement of plug at beginning of episode, based on success rate."""

    if curr_success > cfg_task.rl.curriculum_success_thresh:
        # If success rate is above threshold, increase min downward displacement until max value
        new_max_disp = torch.where(
                                    curr_max_disp + curriculum_height_step[:, 0] < curriculum_height_bound[:, 1],
                                    curr_max_disp + curriculum_height_step[:, 0],
                                    curriculum_height_bound[:, 1],
                                    )
    elif curr_success < cfg_task.rl.curriculum_failure_thresh:
        # If success rate is below threshold, decrease min downward displacement until min value
        new_max_disp = torch.where(
                                    curr_max_disp + curriculum_height_step[:, 1] > curriculum_height_bound[:, 0],
                                    curr_max_disp + curriculum_height_step[:, 1],
                                    curriculum_height_bound[:, 0],
                                    )
    else:
        # Maintain current max downward displacement
        new_max_disp = curr_max_disp

    return new_max_disp


"""
Bonus and Success Checking
"""

def check_plug_close_to_socket(
    keypoints_plug, keypoints_socket, dist_threshold, progress_buf
):
    """Check if plug is close to socket."""

    # Compute keypoint distance between plug and socket
    keypoint_dist = torch.norm(keypoints_socket - keypoints_plug, p=2, dim=-1)

    # Check if keypoint distance is below threshold
    is_plug_close_to_socket = torch.where(
        torch.mean(keypoint_dist, dim=-1) < dist_threshold,
        torch.ones_like(progress_buf),
        torch.zeros_like(progress_buf),
    )

    return is_plug_close_to_socket


def check_plug_inserted_in_socket(
    plug_pos, socket_pos, curriculum_bound, keypoints_plug, keypoints_socket, cfg_task, progress_buf
):
    """Check if plug is inserted in socket."""

    plug_engage_height = curriculum_bound[:, 1] - cfg_task.rl.curriculum_freespace_range

    # Check if plug is within threshold distance of assembled state
    is_plug_below_insertion_height = (
        plug_pos[:, 2] < socket_pos[:, 2] + plug_engage_height
    )
    is_plug_above_table_height = (
        plug_pos[:, 2] > socket_pos[:, 2]
    )

    is_plug_height_success = torch.logical_and(
        is_plug_below_insertion_height, is_plug_above_table_height
    )

    # Check if plug is close to socket
    # NOTE: This check addresses edge case where plug is within threshold distance of
    # assembled state, but plug is outside socket
    is_plug_close_to_socket = check_plug_close_to_socket(
        keypoints_plug=keypoints_plug,
        keypoints_socket=keypoints_socket,
        dist_threshold=cfg_task.rl.close_error_thresh,
        progress_buf=progress_buf,
    )

    # Combine both checks
    is_plug_inserted_in_socket = torch.logical_and(
        is_plug_height_success, is_plug_close_to_socket
    )

    return is_plug_inserted_in_socket

def get_curriculum_reward_scale(cfg_task, curr_max_disp, curriculum_height_bound, curriculum_height_step):
    """Compute reward scale for SBC."""

    # Compute difference between max downward displacement at beginning of training (easiest condition)
    # and current max downward displacement (based on current curriculum stage)
    # NOTE: This number increases as curriculum gets harder
    curr_stage_diff = curr_max_disp - curriculum_height_bound[:, 0]

    # Compute difference between max downward displacement at beginning of training (easiest condition)
    # and min downward displacement (hardest condition)
    final_stage_diff = (
        curriculum_height_bound[:, 1] - curriculum_height_bound[:, 0]
    )

    # Compute reward scale
    reward_scale = curr_stage_diff / final_stage_diff + 1.0

    return reward_scale.mean()


def load_asset_convex_hull_in_warp(urdf_path, device):
    """Create mesh object in Warp."""

    urdf = URDF.load(urdf_path)
    mesh = urdf.links[0].collision_mesh
    convex_hull = trimesh.convex.convex_hull(mesh)

    wp_mesh = wp.Mesh(
        points=wp.array(convex_hull.vertices, dtype=wp.vec3, device=device),
        indices=wp.array(convex_hull.faces.flatten(), dtype=wp.int32, device=device),
    )

    return wp_mesh

def load_asset_convex_hulls_in_warp(plug_files, socket_files, device):
    """Create mesh objects in Warp for all environments."""

    # Load and store plug meshes
    plug_meshes = [
        load_asset_convex_hull_in_warp(
            urdf_path=plug_files[i],
            device=device,
        )
        for i in range(len(plug_files))
    ]

    # Load and store socket meshes
    socket_meshes = [
        load_asset_convex_hull_in_warp(
            urdf_path=socket_files[i],
            device=device,
        )
        for i in range(len(socket_files))
    ]

    return plug_meshes, socket_meshes

def get_max_interpen_dists(
    asset_indices,
    plug_pos,
    plug_quat,
    socket_pos,
    socket_quat,
    wp_plug_meshes,
    wp_socket_meshes,
    wp_device,
    device,
):
    """Get maximum interpenetration distances between plugs and sockets."""

    num_envs = len(plug_pos)
    max_interpen_dists = torch.zeros((num_envs,), dtype=torch.float32, device=device)

    for i in range(num_envs):
        asset_idx = asset_indices[i]

        # Compute transform from plug frame to socket frame
        plug_transform = wp.transform(plug_pos[i], plug_quat[i])
        socket_transform = wp.transform(socket_pos[i], socket_quat[i])
        socket_inv_transform = wp.transform_inverse(socket_transform)
        plug_to_socket_transform = wp.transform_multiply(
            socket_inv_transform, plug_transform
        )

        # Transform plug mesh vertices to socket frame
        plug_points = wp.clone(wp_plug_meshes[asset_idx].points)
        wp.launch(
            kernel=transform_points,
            dim=len(plug_points),
            inputs=[plug_points, plug_points, plug_to_socket_transform],
            device=wp_device,
        )

        # Compute max interpenetration distance between plug and socket
        interpen_dist_plug_socket = wp.zeros(
            (len(plug_points),), dtype=wp.float32, device=wp_device
        )
        wp.launch(
            kernel=get_interpen_dist,
            dim=len(plug_points),
            inputs=[
                plug_points,
                wp_socket_meshes[asset_idx].id,
                interpen_dist_plug_socket,
            ],
            device=wp_device,
        )

        max_interpen_dist = -torch.min(wp.to_torch(interpen_dist_plug_socket))

        # Store interpenetration flag and max interpenetration distance
        if max_interpen_dist > 0.0:
            max_interpen_dists[i] = max_interpen_dist

    return max_interpen_dists


"""
Warp Kernels
"""


# Transform points from source coordinate frame to destination coordinate frame
@wp.kernel
def transform_points(
    src: wp.array(dtype=wp.vec3), dest: wp.array(dtype=wp.vec3), xform: wp.transform
):
    tid = wp.tid()

    p = src[tid]
    m = wp.transform_point(xform, p)

    dest[tid] = m


# Return interpenetration distances between query points (e.g., plug vertices in current pose)
# and mesh surfaces (e.g., of socket mesh in current pose)
@wp.kernel
def get_interpen_dist(
    queries: wp.array(dtype=wp.vec3),
    mesh: wp.uint64,
    interpen_dists: wp.array(dtype=wp.float32),
):
    tid = wp.tid()

    # Declare arguments to wp.mesh_query_point() that will not be modified
    q = queries[tid]  # query point
    max_dist = 1.5  # max distance on mesh from query point

    # Declare arguments to wp.mesh_query_point() that will be modified
    sign = float(
        0.0
    )  # -1 if query point inside mesh; 0 if on mesh; +1 if outside mesh (NOTE: Mesh must be watertight!)
    face_idx = int(0)  # index of closest face
    face_u = float(0.0)  # barycentric u-coordinate of closest point
    face_v = float(0.0)  # barycentric v-coordinate of closest point

    # Get closest point on mesh to query point
    closest_mesh_point_exists = wp.mesh_query_point(
        mesh, q, max_dist, sign, face_idx, face_u, face_v
    )

    # If point exists within max_dist
    if closest_mesh_point_exists:
        # Get 3D position of point on mesh given face index and barycentric coordinates
        p = wp.mesh_eval_position(mesh, face_idx, face_u, face_v)

        # Get signed distance between query point and mesh point
        delta = q - p
        signed_dist = sign * wp.length(delta)

        # If signed distance is negative
        if signed_dist < 0.0:
            # Store interpenetration distance
            interpen_dists[tid] = signed_dist
