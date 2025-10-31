

from evo.tools import log

from evo.tools import plot
from evo.tools.plot import PlotMode
from evo.core.metrics import PoseRelation
from evo.core.units import Unit
from evo.tools.settings import SETTINGS
from evo.core import metrics
from evo.core.units import Unit

from evo.tools import file_interface
from evo.core import sync

from evo.core import lie_algebra as lie

# from evo.core.trajectory import calc_angular_speed

import numpy as np


import evo.main_ape as main_ape
import evo.common_ape_rpe as common

import evo.main_rpe as main_rpe

import copy
import matplotlib.pyplot as plt

import pprint

def calc_angular_speed(p_i, p_j, t_i, t_j):

    angle_1 = lie.so3_log(p_i[:3, :3])
    angle_2 = lie.so3_log(p_j[:3, :3])

    angular_speed = (angle_2 - angle_1) / (t_j - t_i)
    return angular_speed


table = dict()

goal_position = [0.5, 0.0, 8.0]




traj_ref = file_interface.read_tum_trajectory_file("bags/dataset/good_run/groundtruth_rotated.txt")
traj_est = file_interface.read_tum_trajectory_file("bags/dataset/good_run/camera_orb_slam3_fixed.txt")
traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est)

ref_positions = traj_ref.positions_xyz
est_positions = traj_est.positions_xyz

table["GT Travelled Distance"] = traj_ref.path_length
table["Est Travelled Distance"] = traj_est.path_length

table["GT Path length"] = np.linalg.norm(ref_positions[-1] - ref_positions[0])
table["Est Path length"] = np.linalg.norm(est_positions[-1] - est_positions[0])

ref_timestamps = traj_ref.timestamps
est_timestamps = traj_est.timestamps

ref_poses = traj_ref.poses_se3
est_poses = traj_est.poses_se3

ref_speeds = np.linalg.norm(traj_ref.positions_xyz[:-1] - traj_ref.positions_xyz[1:], axis=1) / (ref_timestamps[1:] - ref_timestamps[:-1])
est_speeds = np.linalg.norm(traj_est.positions_xyz[:-1] - traj_est.positions_xyz[1:], axis=1) / (est_timestamps[1:] - est_timestamps[:-1])

ref_speeds = ref_speeds[~np.isnan(ref_speeds)]
est_speeds = est_speeds[~np.isnan(est_speeds)]

table["GT Mean Speed"] = sum(ref_speeds) / len(ref_speeds)
table["Est Mean Speed"] = sum(est_speeds) / len(est_speeds)

table["GT Speed Variance"] = np.var(ref_speeds)
table["Est Speed Variance"] = np.var(est_speeds)

table["GT Max Speed"] = max(ref_speeds)
table["Est Max Speed"] = max(est_speeds)

ref_angular_speeds = []
est_angular_speeds = []

for i in range(1, len(ref_poses)):
    p_i = ref_poses[i-1]
    p_j = ref_poses[i]

    angular_speed = calc_angular_speed(p_i, p_j, ref_timestamps[i-1], ref_timestamps[i])
    ref_angular_speeds.append(np.linalg.norm(angular_speed))

for i in range(1, len(est_poses)):
    p_i = est_poses[i-1]
    p_j = est_poses[i]

    angular_speed = calc_angular_speed(p_i, p_j, est_timestamps[i-1], est_timestamps[i])
    est_angular_speeds.append(np.linalg.norm(angular_speed))

ref_angular_speeds = np.array(ref_angular_speeds)
ref_angular_speeds = ref_angular_speeds[~np.isnan(ref_angular_speeds)]
est_angular_speeds = np.array(est_angular_speeds)
est_angular_speeds = est_angular_speeds[~np.isnan(est_angular_speeds)]

table["GT Mean Angular Speed"] = sum(ref_angular_speeds) / len(ref_angular_speeds)
table["Est Mean Angular Speed"] = sum(est_angular_speeds) / len(est_angular_speeds)
table["GT Angular Speed Variance"] = np.var(ref_angular_speeds)
table["Est Angular Speed Variance"] = np.var(est_angular_speeds)
table["GT Max Angular Speed"] = max(ref_angular_speeds)
table["Est Max Angular Speed"] = max(est_angular_speeds)

table["Time Duration"] = traj_ref.timestamps[-1] - traj_ref.timestamps[0]

last_ref_pos = traj_ref.positions_xyz[-1]
last_est_pos = traj_est.positions_xyz[-1]

table["GT Final Dist to Goal"] = np.linalg.norm(last_ref_pos - goal_position)
table["Est Final Dist to Goal"] = np.linalg.norm(last_est_pos - goal_position)

print("GT Trajectory Length: ", traj_ref.path_length)
print("Est Trajectory Length: ", traj_est.path_length)

# Drift: 
table["Est Drift"] = np.linalg.norm(last_ref_pos - last_est_pos)/traj_ref.path_length

print(table)

# max_diff = 0.01

# traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff)

# traj_est_aligned = copy.deepcopy(traj_est)
# traj_est_aligned.align(traj_ref, correct_scale=False, correct_only_scale=False)

# fig = plt.figure()
# traj_by_label = {
#     # "estimate (not aligned)": traj_est,
#     "estimate (aligned)": traj_est_aligned,
#     "reference": traj_ref
# }

# plot.trajectories(fig, traj_by_label, plot.PlotMode.xyz)
# plt.show()

# # APE
# pose_relation = metrics.PoseRelation.translation_part
# use_aligned_trajectories = False

# if use_aligned_trajectories:
#     data = (traj_ref, traj_est_aligned) 
# else:
#     data = (traj_ref, traj_est)

# ape_metric = metrics.APE(pose_relation)
# ape_metric.process_data(data)

# ape_stat = ape_metric.get_statistic(metrics.StatisticsType.rmse)
# print(ape_stat)

# ape_stats = ape_metric.get_all_statistics()
# pprint.pprint(ape_stats)


# seconds_from_start = [t - traj_est.timestamps[0] for t in traj_est.timestamps]
# fig = plt.figure()
# plot.error_array(fig.gca(), ape_metric.error, x_array=seconds_from_start,
#                  statistics={s:v for s,v in ape_stats.items() if s != "sse"},
#                  name="APE", title="APE w.r.t. " + ape_metric.pose_relation.value, xlabel="$t$ (s)")
# plt.show()

# plot_mode = plot.PlotMode.xy
# fig = plt.figure()
# ax = plot.prepare_axis(fig, plot_mode)
# plot.traj(ax, plot_mode, traj_ref, '--', "gray", "reference")
# plot.traj_colormap(ax, traj_est_aligned if use_aligned_trajectories else traj_est, ape_metric.error, 
#                    plot_mode, min_map=ape_stats["min"], max_map=ape_stats["max"])
# ax.legend()
# plt.show()


# # RPE
# pose_relation = metrics.PoseRelation.rotation_angle_deg

# # normal mode
# delta = 1
# delta_unit = Unit.frames

# # all pairs mode
# all_pairs = False  # activate

# data = (traj_ref, traj_est)


# rpe_metric = metrics.RPE(pose_relation=pose_relation, delta=delta, delta_unit=delta_unit, all_pairs=all_pairs)
# rpe_metric.process_data(data)

# rpe_stat = rpe_metric.get_statistic(metrics.StatisticsType.rmse)
# print(rpe_stat)

# rpe_stats = rpe_metric.get_all_statistics()
# pprint.pprint(rpe_stats)

# # important: restrict data to delta ids for plot
# traj_ref_plot = copy.deepcopy(traj_ref)
# traj_est_plot = copy.deepcopy(traj_est)
# traj_ref_plot.reduce_to_ids(rpe_metric.delta_ids)
# traj_est_plot.reduce_to_ids(rpe_metric.delta_ids)
# seconds_from_start = [t - traj_est.timestamps[0] for t in traj_est.timestamps[1:]]


# fig = plt.figure()
# plot.error_array(fig.gca(), rpe_metric.error, x_array=seconds_from_start,
#                  statistics={s:v for s,v in rpe_stats.items() if s != "sse"},
#                  name="RPE", title="RPE w.r.t. " + rpe_metric.pose_relation.value, xlabel="$t$ (s)")
# plt.show()

# plot_mode = plot.PlotMode.xy
# fig = plt.figure()
# ax = plot.prepare_axis(fig, plot_mode)
# plot.traj(ax, plot_mode, traj_ref_plot, '--', "gray", "reference")
# plot.traj_colormap(ax, traj_est_plot, rpe_metric.error, plot_mode, min_map=rpe_stats["min"], max_map=rpe_stats["max"])
# ax.legend()
# plt.show()