
from unseen_eval.unseen_eval_lib import *


plt.rcParams['text.usetex'] = True
plt.rcParams['font.size'] = FONTSIZE
plt.rcParams['figure.figsize'] = (3.3, 2.5)
plt.rcParams['figure.dpi'] = 600
plt.style.use('_mpl-gallery')


def remove_duplicates(traj):
    """Remove duplicate timestamps from a trajectory."""
    unique_timestamps, unique_indices = np.unique(traj.timestamps, return_index=True)
    new_timestamps = traj.timestamps[unique_indices]
    new_poses_se3 = [traj.poses_se3[i] for i in unique_indices]
    new_orientations_quat_wxyz = traj.orientations_quat_wxyz[unique_indices]
    new_positions_xyz = traj.positions_xyz[unique_indices]
    new_meta = traj.meta.copy()

    new_traj = PoseTrajectory3D(new_positions_xyz,
        new_orientations_quat_wxyz,
        new_timestamps,
        new_poses_se3,
        new_meta)

    return new_traj



def calc_angular_speed(p_i, p_j, t_i, t_j):

    angle_1 = lie.so3_log(p_i[:3, :3])
    angle_2 = lie.so3_log(p_j[:3, :3])

    angular_speed = (angle_2 - angle_1) / (t_j - t_i)
    return angular_speed

def create_table_and_plots(folder, show_plot=False):
    table = dict()

    goal_position = [0.5, 0.0, 8.0]


    N_points = 1000

    # folder = "bags/dataset/good_run/"

    traj_ref = file_interface.read_tum_trajectory_file(folder + "groundtruth_rotated.txt")
    traj_est = file_interface.read_tum_trajectory_file(folder + "camera_orb_slam3_fixed.txt")
    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est)

    traj_ref.downsample(N_points)
    traj_est.downsample(N_points)

    traj_ref = remove_duplicates(traj_ref)
    traj_est = remove_duplicates(traj_est)


    ref_positions = traj_ref.positions_xyz
    est_positions = traj_est.positions_xyz

    # smoothed_ref_positions = savgol_filter(ref_positions, 10500, 3, axis=0)
    # smoothed_est_positions = savgol_filter(est_positions, 10500, 3, axis=0)


    # ref_positions_spline = make_interp_spline(traj_ref.timestamps, traj_ref.positions_xyz, k=3, axis=0)
    # est_positions_spline = make_interp_spline(traj_est.timestamps, traj_est.positions_xyz, k=3, axis=0)

    # ref_positions_resampled = ref_positions_spline(traj_ref.timestamps)
    # est_positions_resampled = est_positions_spline(traj_est.timestamps)

    # ref_speeds_spline = ref_positions_spline.derivative(1)
    # est_speeds_spline = est_positions_spline.derivative(1)

    # ref_speeds = ref_speeds_spline(traj_est.timestamps)
    # est_speeds = est_speeds_spline(traj_est.timestamps)

    table["GT Travelled Distance"] = traj_ref.path_length
    table["Est Travelled Distance"] = traj_est.path_length

    table["GT Path length"] = np.linalg.norm(ref_positions[-1] - ref_positions[0])
    table["Est Path length"] = np.linalg.norm(est_positions[-1] - est_positions[0])

    # print(np.isnan(ref_speeds).any(axis=1))
    ref_timestamps = traj_ref.timestamps
    est_timestamps = traj_est.timestamps

    ref_poses = traj_ref.poses_se3
    est_poses = traj_est.poses_se3

    # Downsample positions and timestamps to avoid nans
    downsampling_factor = 1
    ref_pos_downsampled = ref_positions[::downsampling_factor]
    ref_time_downsampled = ref_timestamps[::downsampling_factor]
    est_pos_downsampled = est_positions[::downsampling_factor]
    est_time_downsampled = est_timestamps[::downsampling_factor]

    ref_speeds = np.linalg.norm(ref_pos_downsampled[:-1] - ref_pos_downsampled[1:], axis=1) / (ref_time_downsampled[1:] - ref_time_downsampled[:-1])
    est_speeds = np.linalg.norm(est_pos_downsampled[:-1] - est_pos_downsampled[1:], axis=1) / (est_time_downsampled[1:] - est_time_downsampled[:-1])


    # ref_speeds = ref_speeds[~np.isnan(ref_speeds)]
    # est_speeds = est_speeds[~np.isnan(est_speeds)]

    table["GT Mean Speed"] = sum(ref_speeds) / len(ref_speeds)
    table["Est Mean Speed"] = sum(est_speeds) / len(est_speeds)
    table["GT Median Speed"] = np.median(ref_speeds)
    table["Est Median Speed"] = np.median(est_speeds)

    table["GT Speed Variance"] = np.var(ref_speeds)
    table["Est Speed Variance"] = np.var(est_speeds)

    table["GT Max Speed"] = max(ref_speeds)
    table["Est Max Speed"] = max(est_speeds)

    ref_angular_speeds = []
    est_angular_speeds = []

    for i in range(1, len(ref_poses), downsampling_factor):
        p_i = ref_poses[i-downsampling_factor]
        p_j = ref_poses[i]

        angular_speed = calc_angular_speed(p_i, p_j, ref_timestamps[i-downsampling_factor], ref_timestamps[i])
        ref_angular_speeds.append(np.linalg.norm(angular_speed))

    for i in range(1, len(est_poses), downsampling_factor):
        p_i = est_poses[i-downsampling_factor]
        p_j = est_poses[i]

        angular_speed = calc_angular_speed(p_i, p_j, est_timestamps[i-downsampling_factor], est_timestamps[i])
        est_angular_speeds.append(np.linalg.norm(angular_speed))

    ref_angular_speeds = np.array(ref_angular_speeds)
    ref_angular_speeds = ref_angular_speeds[~np.isnan(ref_angular_speeds)]
    est_angular_speeds = np.array(est_angular_speeds)
    est_angular_speeds = est_angular_speeds[~np.isnan(est_angular_speeds)]

    table["GT Mean Angular Speed"] = sum(ref_angular_speeds) / len(ref_angular_speeds)
    table["Est Mean Angular Speed"] = sum(est_angular_speeds) / len(est_angular_speeds)
    table["GT Angular Speed Variance"] = np.var(ref_angular_speeds)
    table["Est Angular Speed Variance"] = np.var(est_angular_speeds)
    table["GT Median Angular Speed"] = np.median(ref_angular_speeds)
    table["Est Median Angular Speed"] = np.median(est_angular_speeds)
    table["GT Max Angular Speed"] = max(ref_angular_speeds)
    table["Est Max Angular Speed"] = max(est_angular_speeds)

    table["Time Duration"] = traj_ref.timestamps[-1] - traj_ref.timestamps[0]

    last_ref_pos = traj_ref.positions_xyz[-1]
    last_est_pos = traj_est.positions_xyz[-1]

    table["GT Final Dist to Goal"] = np.linalg.norm(last_ref_pos - goal_position)
    table["Est Final Dist to Goal"] = np.linalg.norm(last_est_pos - goal_position)

    # print("GT Trajectory Length: ", traj_ref.path_length)
    # print("Est Trajectory Length: ", traj_est.path_length)

    # Drift: 
    table["Est Drift"] = np.linalg.norm(last_ref_pos - last_est_pos)/traj_ref.path_length

    # print(table)





    colors = sns.color_palette("coolwarm", n_colors=10)

    fig0 = plt.figure()
    ax0 = fig0.add_subplot(111)
    ax0.plot(est_time_downsampled[:-1], est_speeds, label="Est", color=colors[0],  linewidth=1, zorder=2)
    ax0.plot(ref_time_downsampled[:-1], ref_speeds, label="GT", color='black', linewidth=1, zorder=3)
    ax0.axhline(table["GT Mean Speed"], color=colors[8], linestyle='--', label=f'mean', zorder=4)
    # ax0.axhline(table["GT Median Speed"], color=colors[2], linestyle='-.', label=f'median', zorder=1)
    # ax0.fill_between(ref_time_downsampled[:-1], table["GT Mean Speed"]-table["GT Speed Variance"], table["GT Mean Speed"]+table["GT Speed Variance"], color='grey', alpha=0.3, label=f'$\pm 1\sigma$', zorder=1)

    ax0.set_xlabel(r"$t\,(s)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax0.set_ylabel(r"$v(\frac{m}{s})$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax0.legend(loc="best", ncol=2, fontsize=LABELSIZE)   
    ax0.tick_params(labelsize=LABELSIZE)
    # fig0.tight_layout()

    fig0.savefig(folder + "velocity.pdf", format='pdf')

    fig1 = plt.figure()

    ax1 = fig1.add_subplot(111)

    ax1.plot(est_time_downsampled[:-1], est_angular_speeds, label="Est", color=colors[0],  linewidth=1, zorder=2)
    ax1.plot(ref_time_downsampled[:-1], ref_angular_speeds, label="GT", color='black', linewidth=1, alpha=0.7, zorder=3)
    ax1.axhline(table["GT Mean Angular Speed"], color=colors[8], linestyle='--', label=f'mean', zorder=4)

    ax1.set_xlabel(r"$t\,(s)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax1.set_ylabel(r"$\omega(\frac{rrad}{s})$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax1.legend(loc="best", ncol=2, fontsize=LABELSIZE)   
    ax1.tick_params(labelsize=LABELSIZE)
    # fig1.tight_layout()

    fig1.savefig(folder + "angular_velocity.pdf", format='pdf')


    traj_est_aligned = copy.deepcopy(traj_est)
    traj_est_aligned.align(traj_ref, correct_scale=False, correct_only_scale=False)

    est_positions_aligned = traj_est_aligned.positions_xyz

    # gt_orientations_MAP = [  TF.Rotation.from_quat(orient, scalar_first=True).as_matrix() for orient in traj_ref.orientations_quat_wxyz]
    # est_orientations_MAP = [ TF.Rotation.from_quat(orient, scalar_first=True).as_matrix() for orient in traj_est.orientations_quat_wxyz]
    # est_aligned_orientations_MAP = [ TF.Rotation.from_quat(orient, scalar_first=True).as_matrix() for orient in traj_est_aligned.orientations_quat_wxyz]
    gt_orientations_MAP = [np.linalg.inv(r_DRONE_CAM.as_matrix() @np.linalg.inv(r_DRONE_CAM.as_matrix() @ TF.Rotation.from_quat(orient, scalar_first=True).as_matrix())) for orient in traj_ref.orientations_quat_wxyz]
    est_orientations_MAP = [np.linalg.inv(r_DRONE_CAM.as_matrix() @np.linalg.inv(r_DRONE_CAM.as_matrix() @ TF.Rotation.from_quat(orient, scalar_first=True).as_matrix())) for orient in traj_est.orientations_quat_wxyz]
    est_aligned_orientations_MAP = [np.linalg.inv(r_DRONE_CAM.as_matrix() @np.linalg.inv(r_DRONE_CAM.as_matrix() @ TF.Rotation.from_quat(orient, scalar_first=True).as_matrix())) for orient in traj_est_aligned.orientations_quat_wxyz]



    gt_x = ref_positions[:-1,2]
    gt_y = -ref_positions[:-1,0]
    gt_z = -ref_positions[:-1,1]
    est_x = est_positions_aligned[:-1,2]
    est_y = -est_positions_aligned[:-1,0]
    est_z = -est_positions_aligned[:-1,1]

    gt_roll = TF.Rotation.from_matrix(np.array(gt_orientations_MAP)).as_euler('xyz', degrees=True)[:,0]
    est_roll = TF.Rotation.from_matrix(np.array(est_orientations_MAP)).as_euler('xyz', degrees=True)[:,0]

    gt_pitch = TF.Rotation.from_matrix(np.array(gt_orientations_MAP)).as_euler('xyz', degrees=True)[:,1]
    est_pitch = TF.Rotation.from_matrix(np.array(est_orientations_MAP)).as_euler('xyz', degrees=True)[:,1]
    gt_yaw = TF.Rotation.from_matrix(np.array(gt_orientations_MAP)).as_euler('xyz', degrees=True)[:,2]
    est_yaw = TF.Rotation.from_matrix(np.array(est_orientations_MAP)).as_euler('xyz', degrees=True)[:,2]

    fig2 = plt.figure()

    ax2 = fig2.add_subplot(111)


    ax2.scatter(est_x, est_y, 
                c=est_speeds, cmap="coolwarm",  s=0.2, alpha=0.5,
                label="Est", zorder=3)
    
    ax2.plot(gt_x, gt_y, 
                c='black',   alpha=0.5,
                label="GT", linewidth=1, zorder=2)
    
    ax2.set_xlabel(r"$x\,(m)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax2.set_ylabel(r"$y\,(m)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    # ax2.legend(loc="best", ncol=2, fontsize=LABELSIZE)   
    ax2.tick_params(labelsize=LABELSIZE)
    # fig2.tight_layout()

    fig2.savefig(folder + "traj_xy.pdf", format='pdf')

    fig3 = plt.figure()

    ax3 = fig3.add_subplot(111)


    ax3.scatter(est_x, est_z, 
                c=est_speeds, cmap="coolwarm",  s=0.2, alpha=0.5,
                label="Est", zorder=3)
    ax3.plot(gt_x, gt_z, 
                c='black',   alpha=0.5,
                label="GT", linewidth=1, zorder=2)
    ax3.set_xlabel(r"$x\,(m)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax3.set_ylabel(r"$z\,(m)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    # ax3.legend(loc="best", ncol=2, fontsize=LABELSIZE)   
    ax3.tick_params(labelsize=LABELSIZE)
    # fig3.tight_layout()

    fig3.savefig(folder + "traj_xz.pdf", format='pdf')




    fig4 = plt.figure()

    ax4 = fig4.add_subplot(111)


    ax4.scatter(est_y, est_z, 
                c=est_speeds, cmap="coolwarm",  s=0.2, alpha=0.5,
                label="Est", zorder=3)
    ax4.plot(gt_y, gt_z, 
                c='black',   alpha=0.5,
                label="GT", linewidth=1, zorder=2)
    ax4.set_xlabel(r"$y\,(m)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax4.set_ylabel(r"$z\,(m)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    # ax4.legend(loc="best", ncol=2, fontsize=LABELSIZE)   
    ax4.tick_params(labelsize=LABELSIZE)
    ax4.invert_xaxis()

    # fig4.tight_layout()

    fig4.savefig(folder + "traj_yz.pdf", format='pdf')



    fig_3d = plt.figure()
    ax = fig_3d.add_subplot(111, projection='3d')
    x_dirs = np.array([rot @ np.array([[1],[0],[0]]) for rot in gt_orientations_MAP])

    x_dirs = x_dirs.reshape(-1,3)

    quiver_rate = 50

    ax.quiver(gt_x[::quiver_rate], gt_y[::quiver_rate], gt_z[::quiver_rate], x_dirs[::quiver_rate,0], x_dirs[::quiver_rate,1], x_dirs[::quiver_rate,2], length=0.8, normalize=True, arrow_length_ratio=0.0, linewidths=0.5, color='#B12C00', alpha=0.6, zorder=1)

    ax.plot(gt_x, gt_y, gt_z, 
                c='black', linewidth=2, alpha=0.5,
                label="GT", zorder=3)
    
    ax.plot(gt_x, gt_y, np.min(gt_z)*np.ones_like(gt_z), 
                c='gray', linestyle='--', linewidth=0.5, alpha=0.5)
    ax.plot(gt_x, np.max(gt_y)*np.ones_like(gt_y), gt_z, 
                c='gray', linestyle='--', linewidth=0.5, alpha=0.5)
    ax.plot(np.min(gt_x)*np.ones_like(gt_x), gt_y, gt_z, 
                c='gray', linestyle='--', linewidth=0.5, alpha=0.5)
    ax.scatter(est_x, est_y, est_z, 
                cmap='coolwarm',   alpha=0.5,
                 label="Est",s=0.2, zorder=2)
    ax.scatter(np.min(est_x)*np.ones_like(est_x), est_y, est_z, 
                cmap='coolwarm',   alpha=0.2,
                 s=0.006, zorder=4)
    ax.scatter(est_x, np.max(est_y)*np.ones_like(est_y), est_z, 
                cmap='coolwarm',   alpha=0.2,
                 s=0.006, zorder=4)
    ax.scatter(est_x, est_y, np.min(est_z)*np.ones_like(est_z), 
                cmap='coolwarm',   alpha=0.2,
                 s=0.006, zorder=4)

    ax.view_init(elev=20., azim=-100.0, roll=None, vertical_axis='z')
    ax.tick_params(pad=LABELPADS, labelsize=LABELSIZE)
    # Set axis limits for better visualization
    # ax.set_xlim(x_lim)
    # ax.set_ylim(y_lim)
    # ax.set_zlim(z_lim)
    # set_axes_equal(ax)
    ax.set_xlabel(r'$x$', labelpad=LABELPADS, fontsize=LABELSIZE)
    ax.set_ylabel(r'$y$', labelpad=LABELPADS, fontsize=LABELSIZE)
    ax.set_zlabel(r'$z$', labelpad=LABELPADS, fontsize=LABELSIZE)
    # ax.set_title("All Runs Overlayed")
    # ax.legend(handles=handles, loc="best", ncol=2, fontsize=8)
    ax.legend( loc="best", ncol=2, fontsize=LABELSIZE)
    # ax.grid(False)
    # Get rid of colored axes planes
    # First remove fill
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False

    # Now set color to white (or whatever is "invisible")
    ax.xaxis.pane.set_edgecolor('w')
    ax.yaxis.pane.set_edgecolor('w')
    ax.zaxis.pane.set_edgecolor('w')

    fig_3d.savefig(folder + "traj_3d.pdf", format='pdf')


    """ 
     TIMEPLOTS
    """

    fig5 = plt.figure()
    ax5 = fig5.add_subplot(111)

    ax5.plot(est_time_downsampled[:-1], est_x, label="Est", color=colors[0],  linewidth=1, zorder=2)
    ax5.plot(ref_time_downsampled[:-1], gt_x, label="GT", color='black', linewidth=0.5, zorder=3)

    ax5.set_xlabel(r"$t\,(s)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax5.set_ylabel(r"$x\,(m)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    # ax5.legend(loc="best", ncol=2, fontsize=LABELSIZE)   
    ax5.tick_params(labelsize=LABELSIZE)
    # fig1.tight_layout()

    fig5.savefig(folder + "x_vs_t.pdf", format='pdf')

    fig6 = plt.figure()
    ax6 = fig6.add_subplot(111)

    ax6.plot(est_time_downsampled[:-1], est_y, label="Est", color=colors[0],  linewidth=1, zorder=2)
    ax6.plot(ref_time_downsampled[:-1], gt_y, label="GT", color='black', linewidth=0.5, zorder=3)

    ax6.set_xlabel(r"$t\,(s)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax6.set_ylabel(r"$y\,(m)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    # ax6.legend(loc="best", ncol=2, fontsize=LABELSIZE)   
    ax6.tick_params(labelsize=LABELSIZE)
    # fig1.tight_layout()

    fig6.savefig(folder + "y_vs_t.pdf", format='pdf')

    fig7 = plt.figure()
    ax7 = fig7.add_subplot(111)

    ax7.plot(est_time_downsampled[:-1], est_z, label="Est", color=colors[0],  linewidth=1, zorder=2)
    ax7.plot(ref_time_downsampled[:-1], gt_z, label="GT", color='black', linewidth=0.5, zorder=3)

    ax7.set_xlabel(r"$t\,(s)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax7.set_ylabel(r"$z\,(m)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    # ax7.legend(loc="best", ncol=2, fontsize=LABELSIZE)   
    ax7.tick_params(labelsize=LABELSIZE)
    # fig1.tight_layout()

    fig7.savefig(folder + "z_vs_t.pdf", format='pdf')


    """
     ORIENTATION TIMEPLOTS
    """

    fig8 = plt.figure()
    ax8 = fig8.add_subplot(111)

    ax8.plot(est_time_downsampled, est_roll, label="Est", color=colors[0],  linewidth=1, zorder=2)
    ax8.plot(ref_time_downsampled, gt_roll, label="GT", color='black', linewidth=0.5, zorder=3)
    ax8.set_xlabel(r"$t\,(s)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax8.set_ylabel(r"$\phi\,(^{\circ})$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax8.tick_params(labelsize=LABELSIZE)
    # fig1.tight_layout()
    fig8.savefig(folder + "roll_vs_t.pdf", format='pdf')

    fig9 = plt.figure()
    ax9 = fig9.add_subplot(111)
    ax9.plot(est_time_downsampled, est_pitch, label="Est", color=colors[0],  linewidth=1, zorder=2)
    ax9.plot(ref_time_downsampled, gt_pitch, label="GT", color='black', linewidth=0.5, zorder=3)
    ax9.set_xlabel(r"$t\,(s)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax9.set_ylabel(r"$\theta\,(^{\circ})$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax9.tick_params(labelsize=LABELSIZE)
    # fig1.tight_layout()
    fig9.savefig(folder + "pitch_vs_t.pdf", format='pdf')

    fig10 = plt.figure()
    ax10 = fig10.add_subplot(111)
    ax10.plot(est_time_downsampled, est_yaw, label="Est", color=colors[0],  linewidth=1, zorder=2)
    ax10.plot(ref_time_downsampled, gt_yaw, label="GT", color='black', linewidth=0.5, zorder=3)
    ax10.set_xlabel(r"$t\,(s)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax10.set_ylabel(r"$\psi\,(^{\circ})$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax10.tick_params(labelsize=LABELSIZE)
    # fig1.tight_layout()
    fig10.savefig(folder + "yaw_vs_t.pdf", format='pdf')


    """
    Performance plots 
    """
    table["Est Times"] = est_time_downsampled
    table["GT Times"] = ref_time_downsampled
    table["gt_x"] = gt_x
    table["gt_y"] = gt_y
    table["gt_z"] = gt_z
    table["est_x"] = est_x
    table["est_y"] = est_y
    table["est_z"] = est_z
    table["Est Traj"] = traj_est
    table["GT Traj"] = traj_ref
    table["Est Aligned Traj"] = traj_est_aligned
    table["GT quaterions MAP"] = gt_orientations_MAP
    table["Est aligned quaterions MAP"] = est_aligned_orientations_MAP
    table["Est quaterions MAP"] = est_orientations_MAP

    # # APE
    pose_relation = metrics.PoseRelation.translation_part
    use_aligned_trajectories = False

    if use_aligned_trajectories:
        data = (traj_ref, traj_est_aligned) 
    else:
        data = (traj_ref, traj_est)

    ape_metric = metrics.APE(pose_relation)
    ape_metric.process_data(data)

    ape_stat = ape_metric.get_statistic(metrics.StatisticsType.rmse)
    # print(ape_stat)

    ape_stats = ape_metric.get_all_statistics()
    # pprint.pprint(ape_stats)

    # seconds_from_start = [t - traj_est.timestamps[0] for t in traj_est.timestamps]
    fig_ape = plt.figure()
    ax_ape = fig_ape.add_subplot(111)
    ax_ape.plot(est_time_downsampled, ape_metric.error, label="APE", color=colors[0],  linewidth=1, zorder=2)
    ax_ape.set_xlabel(r"$t\,(s)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax_ape.set_ylabel(r"APE $\,(m)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax_ape.tick_params(labelsize=LABELSIZE)
    # fig_ape.tight_layout()

    table["APE"] = ape_metric.error
    table["Mean APE"] = ape_stats["mean"]
    table["Median APE"] = ape_stats["median"]
    table["Std APE"] = ape_stats["std"]
    table["Min APE"] = ape_stats["min"]
    table["Max APE"] = ape_stats["max"]
    table["RMSE APE"] = ape_stats["rmse"]

    ax_ape.axhspan(
                   ape_stats["mean"] - ape_stats["std"]/2,
                   ape_stats["mean"] + ape_stats["std"]/2,
                    color='grey',
                    alpha=0.5
                )
    ax_ape.axhline(ape_stats["mean"], color=colors[8], linestyle='--', label='mean', zorder=4)
    ax_ape.axhline(ape_stats["rmse"], color=colors[5], linestyle='--', label='rmse', zorder=4)
    ax_ape.axhline(ape_stats["median"], color=colors[3], linestyle='--', label='median', zorder=4)
    ax_ape.legend(loc="upper left", ncol=4, fontsize=LABELSIZE)

    fig_ape.savefig(folder + "ape_vs_t.pdf", format='pdf')


    # # RPE
    pose_relation = metrics.PoseRelation.rotation_angle_deg

    # normal mode
    delta = 1
    delta_unit = Unit.frames

    # all pairs mode
    all_pairs = False  # activate

    data = (traj_ref, traj_est_aligned)


    rpe_metric = metrics.RPE(pose_relation=pose_relation, delta=delta, delta_unit=delta_unit, all_pairs=all_pairs)
    rpe_metric.process_data(data)

    rpe_stat = rpe_metric.get_statistic(metrics.StatisticsType.rmse)
    # print(rpe_stat)

    rpe_stats = rpe_metric.get_all_statistics()
    # pprint.pprint(rpe_stats)

    

    fig_rpe = plt.figure()
    ax_rpe = fig_rpe.add_subplot(111)
    ax_rpe.plot(est_time_downsampled[:-1], rpe_metric.error, label="RPE", color=colors[0],  linewidth=1, zorder=2)
    ax_rpe.set_xlabel(r"$t\,(s)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax_rpe.set_ylabel(r"RPE $\,(m)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax_rpe.tick_params(labelsize=LABELSIZE)
    # fig_rpe.tight_layout()

    table["RPE"] = rpe_metric.error
    table["Mean RPE"] = rpe_stats["mean"]
    table["Median RPE"] = rpe_stats["median"]
    table["Std RPE"] = rpe_stats["std"]
    table["Min RPE"] = rpe_stats["min"]
    table["Max RPE"] = rpe_stats["max"]
    table["RMSE RPE"] = rpe_stats["rmse"]

    ax_rpe.axhspan(
                   rpe_stats["mean"] - rpe_stats["std"]/2,
                   rpe_stats["mean"] + rpe_stats["std"]/2,
                    color='grey',
                    alpha=0.5
                )
    ax_rpe.axhline(rpe_stats["mean"], color=colors[8], linestyle='--', label='mean', zorder=4)
    ax_rpe.axhline(rpe_stats["rmse"], color=colors[5], linestyle='--', label='rmse', zorder=4)
    ax_rpe.axhline(rpe_stats["median"], color=colors[3], linestyle='--', label='median', zorder=4)
    ax_rpe.legend(loc="upper left", ncol=4, fontsize=LABELSIZE)

    fig_rpe.savefig(folder + "rpe_vs_t.pdf", format='pdf')
    # plot.error_array(fig_rpe.gca(), rpe_metric.error, x_array=est_time_downsampled[:-1],
    #                  statistics={s:v for s,v in rpe_stats.items() if s != "sse"},
    #                  name="RPE", title="RPE w.r.t. " + rpe_metric.pose_relation.value, xlabel="$t$ (s)")



    if show_plot:
        plt.show()  

    plt.close()

    


    return table


def main():
    
    parser = argparse.ArgumentParser(description="Compute visual shakiness and tracking error.")
    parser.add_argument("folder", help="Path to directory with sequential images")
    args = parser.parse_args()
    folder = args.folder
    table = create_table_and_plots(folder, show_plot=True)


if __name__ == "__main__":
    main()

# python3 src/unseen_eval/unseen_eval/evo_evaluate_trajectory.py bags/dataset/good_run


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