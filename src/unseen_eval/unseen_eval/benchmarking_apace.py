from unseen_eval.unseen_eval_lib import *


plt.rcParams['text.usetex'] = True
plt.rcParams['font.size'] = FONTSIZE
plt.rcParams['figure.figsize'] = (5.0, 2.5)
plt.rcParams['figure.dpi'] = 600
plt.style.use('_mpl-gallery')


def main(): 

    benchmarking_resources = "src/benchmarking/resource"

    apace_gt = file_interface.read_tum_trajectory_file("src/benchmarking/resource/trajectories/cube_benchmark/apace/gt_trajectory.txt")
    apace_est = file_interface.read_tum_trajectory_file("src/benchmarking/resource/trajectories/cube_benchmark/apace/est_trajectory.txt")

    apace_gt, apace_est = sync.associate_trajectories(apace_gt, apace_est, max_diff=0.01)

    unseen_gt = file_interface.read_tum_trajectory_file("src/benchmarking/resource/trajectories/cube_benchmark/unseen_plan/gt_trajectory.txt")
    unseen_est = file_interface.read_tum_trajectory_file("src/benchmarking/resource/trajectories/cube_benchmark/unseen_plan/est_trajectory.txt")

    unseen_gt, unseen_est = sync.associate_trajectories(unseen_gt, unseen_est, max_diff=0.01)

    unseen_pos =  unseen_gt.positions_xyz
    apace_pos =  apace_gt.positions_xyz

    pose_relation = metrics.PoseRelation.full_transformation

    data_unseen = (unseen_gt, unseen_est)
    data_apace = (apace_gt, apace_est)


    ape_metric_unseen = metrics.APE(pose_relation)
    ape_metric_unseen.process_data(data_unseen)

    unseen_ape_stats = ape_metric_unseen.get_all_statistics()

    ape_metric_apace = metrics.APE(pose_relation)
    ape_metric_apace.process_data(data_apace)
    apace_ape_stats = ape_metric_apace.get_all_statistics()

    unseen_x, unseen_y, unseen_z = unseen_pos[:,2], -unseen_pos[:,0], -unseen_pos[:,1]
    apace_x, apace_y, apace_z = apace_pos[:,2], -apace_pos[:,0], -apace_pos[:,1]


    colors = sns.color_palette("coolwarm", n_colors=10)

    fig_traj = plt.figure()
    ax_traj = fig_traj.add_subplot(111, projection='3d')

    # ax_traj.scatter(unseen_x, unseen_y, unseen_z, label="UNSEEN", cmap = "winter", c=ape_metric_unseen.error, s=0.5)
    ax_traj.plot(unseen_x, unseen_y, unseen_z, color=colors[0], label="UNSEEN", linewidth=1.0, alpha=1.0, zorder=2)

    # ax_traj.scatter(apace_x, apace_y, apace_z, label="APACE", cmap = "winter", c=ape_metric_apace.error, s=0.8, marker='s')
    ax_traj.plot(apace_x, apace_y, apace_z, label="APACE", color=colors[9], linewidth=1.0, alpha=1.0, zorder=1)

    ax_traj.tick_params(pad=LABELPADS, labelsize=LABELSIZE)
    # Set axis limits for better visualization
    # ax_traj.set_xlim(x_lim)
    # ax_traj.set_ylim(y_lim)
    # ax_traj.set_zlim(z_lim)
    # set_axes_equal(ax)
    ax_traj.set_xlabel(r'$x(m)$', labelpad=LABELPADS, fontsize=LABELSIZE)
    ax_traj.set_ylabel(r'$y(m)$', labelpad=LABELPADS, fontsize=LABELSIZE)
    ax_traj.set_zlabel(r'$z(m)$', labelpad=LABELPADS, fontsize=LABELSIZE)
    # ax_traj.set_title("All Runs Overlayed")
    # ax_traj.legend(handles=handles, loc="best", ncol=2, fontsize=8)
    ax_traj.legend( loc="best", ncol=1, fontsize=LABELSIZE)
    # ax_traj.grid(False)
    # Get rid of colored axes planes
    # First remove fill
    ax_traj.xaxis.pane.fill = False
    ax_traj.yaxis.pane.fill = False
    ax_traj.zaxis.pane.fill = False

    # Now set color to white (or whatever is "invisible")
    ax_traj.xaxis.pane.set_edgecolor('w')
    ax_traj.yaxis.pane.set_edgecolor('w')
    ax_traj.zaxis.pane.set_edgecolor('w')

    fig_traj.savefig("src/benchmarking/resource/images/traj_3d.pdf", format='pdf')

    
    
    fig_ape = plt.figure()
    ax_ape = fig_ape.add_subplot(111)
    ax_ape.plot(unseen_gt.timestamps, ape_metric_unseen.error, color=colors[0], label="UN", linewidth=1, zorder=2)
    ax_ape.plot(apace_gt.timestamps, ape_metric_apace.error, color=colors[9], label="AP", linewidth=1, zorder=1)

    ax_ape.set_xlabel(r"$t\,(s)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax_ape.set_ylabel(r"APE $\,(m)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax_ape.set_xlim(left=0)
    ax_ape.set_ylim(bottom=0, top=4.0)
    ax_ape.tick_params(labelsize=LABELSIZE)
    # fig_ape.tight_layout()

    ax_ape.axhline(unseen_ape_stats["rmse"], color=colors[4], linestyle='--', label=r'UN\,rmse', zorder=4)
    ax_ape.axhline(apace_ape_stats["rmse"], color=colors[7], linestyle='--', label=r'AP\,rmse', zorder=4)

    ax_ape.legend(loc="best", ncol=2, fontsize=LABELSIZE)

    fig_ape.savefig("src/benchmarking/resource/images/ape_vs_t.pdf", format='pdf')


    plt.show()


if __name__ == "__main__":
    main()
