import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d, PchipInterpolator

# Enable LaTeX rendering and large fonts
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.size": 18,
    "axes.labelsize": 20,
    "legend.fontsize": 18,
    "xtick.labelsize": 16,
    "ytick.labelsize": 16,
})

# Original data
map_sizes = np.array([0, 500, 2000, 5000, 15000, 40000], dtype=float)
apace_times = np.array([0.0, 0.19, 0.19, 1.2, 5.0, 22.0])
unseen_times = np.array([0.0, 0.2, 0.5, 1.0, 1.2, 1.5])

# Success masks
apace_success = np.array([True, True, False, False, False, False])

# Extend data to start near zero (epsilon for stability)

# Interpolate in linear-x / log-y space
apace_interp = interp1d(map_sizes, apace_times, kind="linear")
unseen_interp = interp1d(map_sizes, unseen_times, kind="linear")
# apace_interp = PchipInterpolator(map_sizes, apace_times)
# unseen_interp = PchipInterpolator(map_sizes, unseen_times)

x_dense = np.linspace(0.0, map_sizes.max(), 41)
y_apace_dense = apace_interp(x_dense)
y_unseen_dense = unseen_interp(x_dense)

# Plot
plt.figure(figsize=(9, 3.5))

# APACE
# plt.plot(
#     x_dense,
#     y_apace_dense,
#     color="darkred",
#     linestyle="--",
#     linewidth=3,
#     label=r"\textbf{APACE} (failure)"
# )
apace_success_vals = apace_times[apace_success]
apace_failure_mask = x_dense >= 1800
apace_success_mask = ~apace_failure_mask
bad_apace = y_apace_dense[apace_failure_mask]
good_apace= y_apace_dense[apace_success_mask]
# failure_mask = y_apace_dense[]

plt.scatter(
    x_dense[apace_failure_mask],
    bad_apace,
    color="darkred",
    s=50,
    marker="x",
    zorder=5, 
    label=r"\textbf{AP} fail"
)

# UNSEEN
plt.plot(
    map_sizes,
    unseen_times,
    color="tab:blue",
    linewidth=2,
    label=r"\textbf{UN}"
)
plt.plot(
    x_dense[apace_success_mask],
    y_apace_dense[apace_success_mask],
    color="darkred",
    linewidth=3,
    zorder=5,
    label=r"\textbf{AP} succ"
)

# plt.xscale("log")
# plt.yscale("log")
plt.xlim(left=0)
plt.xlim(right=9000)

plt.ylim(bottom=0)
plt.ylim(top=2.5)



plt.xlabel(r"Map size [points]")
plt.ylabel(r"Computation time [s]")

plt.grid(True, which="both", linestyle=":", linewidth=0.8)
plt.legend(frameon=True, ncol=3, loc="upper left")
plt.tight_layout()
plt.show()
