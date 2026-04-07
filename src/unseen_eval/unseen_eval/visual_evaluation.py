from unseen_eval.unseen_eval_lib import *


plt.rcParams['text.usetex'] = True
plt.rcParams['font.size'] = FONTSIZE
plt.rcParams['figure.figsize'] = (5.0, 2.5)
plt.rcParams['figure.dpi'] = 600
plt.style.use('_mpl-gallery')

def load_images(folder):
    exts = ('*.png', '*.jpg', '*.jpeg', '*.bmp')
    files = []
    for e in exts:
        files.extend(glob.glob(os.path.join(folder, e)))
    files.sort()
    return [cv2.imread(f, cv2.IMREAD_GRAYSCALE) for f in files]

def butter_highpass(cutoff, fs, order=4):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a


def wobble_index(flow):
    u = flow[...,0]
    r = np.median(u, axis=1)
    r0 = np.median(r)
    return np.sqrt(np.mean((r - r0)**2)) / u.shape[1]

def blur_index_varlap(img):
    return cv2.Laplacian(img, cv2.CV_64F).var()

def blur_index_freq(img, cutoff=0.15):
    f = np.fft.fftshift(np.fft.fft2(img.astype(np.float32)))
    mag2 = np.abs(f)**2
    H,W = img.shape
    yy,xx = np.mgrid[-H//2:H//2, -W//2:W//2]
    r = np.sqrt(xx**2+yy**2)/np.sqrt((H/2)**2+(W/2)**2)
    hi = mag2[r>cutoff].sum()
    lo = mag2[r<=cutoff].sum()+1e-9
    return hi/lo  # lower => blurrier


def extract_motions(images, plot_matches=False):
    orb = cv2.ORB_create(600)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    motions = []
    residuals = []

    trans = []
    wobble = []
    blur_vlap = []
    blur_freq = []
    H, W = images[0].shape[:2]

    n_matches = []
    n_features_in_frame_0 = []
    n_features_in_frame_1 = []
    for i in range(len(images) - 1):
        kp1, des1 = orb.detectAndCompute(images[i], None)
        kp2, des2 = orb.detectAndCompute(images[i + 1], None)

        n_features_in_frame_0.append(len(kp1))
        n_features_in_frame_1.append(len(kp2))
        if des1 is None or des2 is None:
            motions.append(np.zeros(3))
            residuals.append(0)
            continue

        matches = bf.match(des1, des2)
        if len(matches) < 8:
            motions.append(np.zeros(3))
            residuals.append(0)
            continue

        n_matches.append(len(matches))

        if plot_matches:
            multiplier = int(4)


            new_H = int(H * multiplier)
            
            new_W = int(W * multiplier)

            img_0 = cv2.resize(images[i], (new_W, new_H), interpolation=cv2.INTER_CUBIC)
            img_1 = cv2.resize(images[i + 1], (new_W, new_H), interpolation=cv2.INTER_CUBIC)
            kp1_mult = [cv2.KeyPoint(x=kp.pt[0]*multiplier, y=kp.pt[1]*multiplier, size=kp.size*multiplier) for kp in kp1]
            kp2_mult = [cv2.KeyPoint(x=kp.pt[0]*multiplier, y=kp.pt[1]*multiplier, size=kp.size*multiplier) for kp in kp2]

            img_match = cv2.drawMatches(img_0,kp1_mult,img_1,kp2_mult,matches,None, matchColor=(0,255,0), matchesThickness=1, singlePointColor=(255,255,255),  flags=cv2.DRAW_MATCHES_FLAGS_DEFAULT)
            cv2.imwrite(f"src/unseen_eval/resource/results/match_{i:03d}.png", img_match)
            cv2.imshow("Matches", img_match)
            if cv2.waitKey(1) == ord('q'):
                break

        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])

        [M, inliers] = cv2.estimateAffine2D(pts1, pts2, ransacReprojThreshold=3.0)


        if M is not None:
            dx, dy = M[0,2], M[1,2]
            rot = np.arctan2(M[1,0], M[0,0])  # radians
            trans.append((dx,dy,rot))
            # compensate and flow
            b_warp = cv2.warpAffine(images[i + 1], M, [424, 240]) #, flags=cv2.INTER_LINEAR+cv2.WARP_INVERSE_MAP)
            flow = cv2.calcOpticalFlowFarneback(images[i], b_warp, None, 0.5, 3, 25, 3, 5, 1.2, 0)
            wobble.append(wobble_index(flow))

        blur_vlap.append(blur_index_varlap(images[i]))
        blur_freq.append(blur_index_freq(images[i]))


        E, mask = cv2.findEssentialMat(pts1, pts2, focal=1.0, pp=(0., 0.), method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None:
            motions.append(np.zeros(3))
            residuals.append(0)
            continue
        _, R, t, mask_pose = cv2.recoverPose(E, pts1, pts2, focal=1.0, pp=(0., 0.))
        # small-angle rotation vector
        rvec, _ = cv2.Rodrigues(R)
        motions.append(rvec.flatten())

        # Compute per-match residual = Euclidean distance between observed and predicted flow.
        # Since no depth, use rotation+translation to estimate flow direction via epipolar constraint.

        Homo, _ = cv2.findHomography(pts1, pts2, cv2.RANSAC)

        pts2_pred = cv2.perspectiveTransform(pts1[None, :, :], Homo)[0]
        res = np.linalg.norm(pts2 - pts2_pred, axis=1)

        # # median residual flow after removing global motion
        # proj = cv2.projectPoints(np.ones((len(pts1),3)), rvec, t, np.eye(3), None)[0].reshape(-1,2)
        # res = np.linalg.norm((pts2 - pts1) - (proj - np.median(proj,axis=0)), axis=1)
        
        residuals.append(np.median(res))

        # flow = cv2.calcOpticalFlowFarneback(images[i], images[i + 1], None,
        #                             pyr_scale=0.5, levels=3, winsize=15,
        #                             iterations=3, poly_n=5, poly_sigma=1.2, flags=0)
    
    trans = np.array(trans, float)
    # vibration PSD (detrend and welch)
    valid = ~np.isnan(trans).any(1)
    psd = {}
    if valid.sum() > 8:
        nperseg = min(256, valid.sum())
        for i,name in enumerate(["dx","dy","rot"]):
            sig = detrend(trans[valid,i])
            f, Pxx = welch(sig, fs=6.0, nperseg=nperseg)
            psd[name] = (f, Pxx)



    # TODO: Plot features, plot matches and residuals
    
    return {
        "wobble_index": np.array(wobble),
        "blur_varlap": np.array(blur_vlap),
        "blur_freq_ratio": np.array(blur_freq),
        "transforms": trans,
        "psd": psd,
        "residuals": np.array(residuals), 
        "n_matches": np.array(n_matches),
        "n_features_in_frame_0": np.array(n_features_in_frame_0),
        "n_features_in_frame_1": np.array(n_features_in_frame_1)
    }


def compute_shakiness(motions, fps=30.0, cutoff=2.0):
    if len(motions) < 2:
        return 0.0
    b, a = butter_highpass(cutoff, fps)
    filtered = filtfilt(b, a, motions, axis=0)
    rms = np.sqrt(np.mean(np.sum(filtered**2, axis=1)))
    return rms


def read_map_points(filepath: str) -> int:
    """Reads the number of map points from a file."""
    
    if not os.path.exists(filepath):
        return 0

    with open(filepath, "r") as f:
        lines = f.readlines()

    return len(lines)


def analyze_sequence(folder): 
    images = load_images(folder + "/rgb/")
    print(f"Loaded {len(images)} frames.")
    res_dict = extract_motions(images)

    res_dict["mean wobble"] = np.mean(res_dict["wobble_index"])
    res_dict["mean blur varlap"] = np.mean(res_dict["blur_varlap"])
    res_dict["mean blur freq ratio"] = np.mean(res_dict["blur_freq_ratio"])
    res_dict["median tracking residual"] = np.median(res_dict["residuals"])
    res_dict["p95 tracking residual"] = np.percentile(res_dict["residuals"],95)
    res_dict["mean n matches"] = np.mean(res_dict["n_matches"])
    res_dict["mean n features in frame 0"] = np.mean(res_dict["n_features_in_frame_0"])
    res_dict["mean n features in frame 1"] = np.mean(res_dict["n_features_in_frame_1"])
    res_dict["n map points"] = read_map_points(os.path.join(folder, "../map_orb_slam3.txt"))

    store_results(res_dict, folder, plot=False)

    return res_dict


def store_results(res_dict, output_folder, plot=False):

    colors = sns.color_palette("coolwarm", n_colors=10)

    times = np.arange(len(res_dict["wobble_index"]))/6.0  # assuming 6 FPS
    fig_wobble = plt.figure()
    ax_wobble = fig_wobble.add_subplot(111)
    ax_wobble.plot(times, res_dict["wobble_index"], label="Wobble Index", color=colors[0],  linewidth=1, zorder=2)
    ax_wobble.set_xlabel(r"$t (s)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax_wobble.set_ylabel(r"Wobble Index", labelpad=LABELPADS, fontsize=FONTSIZE)
    # ax_wobble.legend(loc="best", ncol=2, fontsize=LABELSIZE)
    ax_wobble.tick_params(labelsize=LABELSIZE)
    fig_wobble.savefig(os.path.join(output_folder, "wobble_index.pdf"), format='pdf')


    fig_blur_varlap = plt.figure()
    ax_blur_varlap = fig_blur_varlap.add_subplot(111)
    ax_blur_varlap.plot(times, res_dict["blur_varlap"], color=colors[0],  linewidth=1, zorder=2)
    ax_blur_varlap.set_xlabel(r"$t (s)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax_blur_varlap.set_ylabel(r"$\sigma^2 (\nabla^2(\mathcal{I}))$", labelpad=LABELPADS, fontsize=FONTSIZE)
    # ax_blur_varlap.legend(loc="best", ncol=2, fontsize=LABELSIZE)    plt.grid()
    ax_blur_varlap.tick_params(labelsize=LABELSIZE)
    ax_blur_varlap.grid()
    fig_blur_varlap.savefig(os.path.join(output_folder, "blur_varlap.pdf"), format='pdf')

    fig_blur_freq_ratio = plt.figure()
    ax_blur_freq_ratio = fig_blur_freq_ratio.add_subplot(111)
    ax_blur_freq_ratio.plot(times, res_dict["blur_freq_ratio"], color=colors[0],  linewidth=1, zorder=2)
    ax_blur_freq_ratio.set_xlabel(r"$t (s)$", labelpad=LABELPADS, fontsize=FONTSIZE)
    ax_blur_freq_ratio.set_ylabel(r"$\rho_{blur}$", labelpad=LABELPADS, fontsize=FONTSIZE)
    # ax_blur_freq_ratio.legend(loc="best", ncol=2, fontsize=LABELSIZE)
    ax_blur_freq_ratio.tick_params(labelsize=LABELSIZE)
    ax_blur_freq_ratio.grid()
    fig_blur_freq_ratio.savefig(os.path.join(output_folder, "blur_freq_ratio.pdf"), format='pdf')

    fig_psd = []
    for key, (f, Pxx) in res_dict["psd"].items():
        fig = plt.figure()
        plt.semilogy(f, Pxx)
        plt.title(f"PSD of {key}")
        plt.xlabel("Frequency (Hz)")
        fig_psd.append(fig)
        plt.ylabel("Power Spectral Density")
        plt.grid()

        plt.savefig(os.path.join(output_folder, f"psd_{key}.pdf"), format='pdf')
    
    # plot matched features (counts) and per-frame residuals
    # fig_features = plt.figure(figsize=[3.3, 3.3])
    fig_features = plt.figure()
    ax_features = fig_features.add_subplot(111)

    nm = res_dict.get("n_matches", np.array([]))
    f0 = res_dict.get("n_features_in_frame_0", np.array([]))
    f1 = res_dict.get("n_features_in_frame_1", np.array([]))

    if nm.size:
        ax_features.plot(times, nm, color=colors[0], linewidth=0.5, marker='o', markersize=0.3, label=r"\# Match")
    if f0.size:
        ax_features.plot(times, f0, linewidth=0.5, color="#FE6244", linestyle='-.', label=r"\# $\mathcal{I}_{i}$")
    if f1.size:
        ax_features.plot(times, f1, linewidth=0.5, color="#060771", linestyle=':', label=r"\# $\mathcal{I}_{i+1}$")

    # ax_features.set_title("Matched features and detected features per frame")
    ax_features.set_xlabel("$t (s)$")
    ax_features.set_ylabel("Feature Count")
    ax_features.legend(loc="best", ncol=1, fontsize=LABELSIZE)
    ax_features.tick_params(labelsize=LABELSIZE)
    ax_features.set_xlim(left=0)
    ax_features.set_ylim(bottom=0)
    ax_features.grid(True)

    fig_features.savefig(os.path.join(output_folder, "features.pdf"), format='pdf')

    fig_residuals = plt.figure()
    ax_residuals = fig_residuals.add_subplot(111)
    res = res_dict.get("residuals", np.array([]))

    if res.size:
        x = np.arange(len(res))
        ax_residuals.plot(times, res, '-o', markersize=0.3, linewidth=0.5, color=colors[0], label="Med.")
        p95 = np.percentile(res, 95)
        ax_residuals.axhline(p95, color='r', linewidth=0.5, linestyle='--', label=r"$\mathbf{r}_{95}$") #+ f"{p95:.2f}")
    else:
        plt.text(0.5, 0.5, "No residuals available", ha="center", va="center")

    ax_residuals.set_xlabel(r"$t (s)$")
    ax_residuals.set_xlim(left=0)
    ax_residuals.set_ylim(bottom=0)
    ax_residuals.set_ylabel(r"Residual $(px)$")
    ax_residuals.legend(loc="best", ncol=2, fontsize=LABELSIZE)
    ax_residuals.tick_params(labelsize=LABELSIZE)
    ax_residuals.grid(True)

    fig_residuals.savefig(os.path.join(output_folder, "residuals.pdf"), format='pdf')

    plt.show()



def main(folder):

    rgb_folder = folder + "/rgb/"

    images = load_images(rgb_folder)

    print(f"Loaded {len(images)} frames.")

    res_dict = extract_motions(images, plot_matches=True)

    # J_vis = compute_shakiness(motions)
    # print(f"Visual shakiness (RMS residual angle, deg): {np.degrees(J_vis):.3f}")

    print(f"Mean wobble : {np.mean(res_dict['wobble_index']):.3f}")
    print(f"Mean blur variance (Laplacian): {np.mean(res_dict['blur_varlap']):.3f}")
    print(f"Mean blur frequency ratio: {np.mean(res_dict['blur_freq_ratio']):.3f}")
    print(f"Median tracking residual (pixels): {np.median(res_dict['residuals']):.3f}")
    print(f"95th percentile tracking residual (pixels): {np.percentile(res_dict['residuals'],95):.3f}")

    store_results(res_dict, folder, plot=True) 

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Compute visual shakiness and tracking error.")
    parser.add_argument("folder", help="Path to directory with sequential images")
    args = parser.parse_args()
    main(args.folder)

# python3 src/unseen_eval/unseen_eval/visual_evaluation.py bags/dataset/rosbag2_2025_08_08-14_12_12/

"""
Use two signals from your frames:

1. how much the scene moved as a rigid body, and
2. how much residual, row-dependent motion remains after removing that rigid motion.

That residual is your distortion.

## Procedure

1. **Prep**

   * Convert to grayscale.
   * If you know camera intrinsics, undistort first.
   * Use a fixed ROI that excludes the propellers and HUD overlays.

2. **Global motion per frame pair**

   * Detect features (ORB/FAST).
   * Match and fit a **2D transform** with RANSAC (start with affine; use homography only if you expect parallax).
   * Save the transform (T_k) mapping frame (k!\to!k!+!1).

3. **Residual flow (distortion)**

   * Warp frame (k!+!1) by (T_k^{-1}) to align it to frame (k).
   * Compute **dense optical flow** between frame (k) and the warped frame (\hat{I}_{k+1}).
   * For rolling-shutter wobble, look at **row statistics** of the x-flow:
     [
     r(y) = \operatorname{median}_x, u(x,y),\qquad \bar r = \operatorname{median}_y, r(y)
     ]
     [
     \text{WobbleIndex}_k = \frac{\sqrt{\frac{1}{H}\sum_y (r(y)-\bar r)^2}}{W}
     ]
     where (u) is horizontal flow, (H,W) are image height and width.
   * This measures row-dependent skew normalized by width. Larger = more distortion.

4. **Motion-blur severity (per frame)**

   * Fast proxy: **variance of Laplacian**. Lower variance = more blur.
   * Better: high/low frequency energy ratio
     [
     B = \frac{\sum_{|\omega|>\omega_c} |F(\omega)|^2}{\sum_{|\omega|\le \omega_c} |F(\omega)|^2}
     ]
     Lower (B) = more blur. Report (1!-!B) as a blur index if you want “higher=worse”.

5. **Vibration frequency**

   * From each (T_k) extract translation ((t_x,t_y)) and rotation (\theta).
   * Detrend with a high-pass or polynomial fit to remove the slow flight path.
   * Compute a PSD (Welch). Peaks give vibration frequencies; aliasing is likely at low FPS, so report both raw peak and the aliased family (f, f_s!-!f) where (f_s) is the frame rate.

6. **Outputs to report**

   * Time series: WobbleIndex(_k), blur index per frame, and rigid ((t_x,t_y,\theta)).
   * PSD of detrended ((t_x,t_y,\theta)).
   * A single scalar per clip: mean and 95th-percentile of WobbleIndex and blur index.

## Pitfalls

* Large parallax or near objects will corrupt global motion. Use multiple ROIs or restrict matches to distant features if possible.
* If the shutter is **global**, wobble index should be near zero; distortion will present mainly as blur and inter-frame jumps.
* Low FPS creates aliasing. Always include (f) and (f_s!-!f) when reporting vibration peaks.

## Reference implementation (OpenCV + NumPy)

```python
import cv2 as cv
import numpy as np
from scipy.signal import welch, detrend

def affine_between(a, b):
    # ORB features + RANSAC affine
    orb = cv.ORB_create(3000)
    kp1, des1 = orb.detectAndCompute(a, None)
    kp2, des2 = orb.detectAndCompute(b, None)
    if des1 is None or des2 is None: return None
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
    m = sorted(bf.match(des1, des2), key=lambda x:x.distance)[:500]
    if len(m) < 6: return None
    pts1 = np.float32([kp1[x.queryIdx].pt for x in m])
    pts2 = np.float32([kp2[x.trainIdx].pt for x in m])
    M, inl = cv.estimateAffine2D(pts1, pts2, ransacReprojThreshold=3.0)
    return M  # 2x3

def wobble_index(flow):
    u = flow[...,0]
    r = np.median(u, axis=1)
    r0 = np.median(r)
    return np.sqrt(np.mean((r - r0)**2)) / u.shape[1]

def blur_index_varlap(img):
    return cv.Laplacian(img, cv.CV_64F).var()

def blur_index_freq(img, cutoff=0.15):
    f = np.fft.fftshift(np.fft.fft2(img.astype(np.float32)))
    mag2 = np.abs(f)**2
    H,W = img.shape
    yy,xx = np.mgrid[-H//2:H//2, -W//2:W//2]
    r = np.sqrt(xx**2+yy**2)/np.sqrt((H/2)**2+(W/2)**2)
    hi = mag2[r>cutoff].sum()
    lo = mag2[r<=cutoff].sum()+1e-9
    return hi/lo  # lower => blurrier

def analyze_sequence(frames, fps):
    H,W = frames[0].shape[:2]
    trans = []
    wobble = []
    blur_vlap = []
    blur_freq = []
    for k in range(len(frames)-1):
        a = cv.cvtColor(frames[k], cv.COLOR_BGR2GRAY)
        b = cv.cvtColor(frames[k+1], cv.COLOR_BGR2GRAY)
        M = affine_between(a,b)
        if M is None: 
            trans.append((np.nan,np.nan,np.nan)); wobble.append(np.nan)
        else:
            # decompose affine
            dx, dy = M[0,2], M[1,2]
            rot = np.arctan2(M[1,0], M[0,0])  # radians
            trans.append((dx,dy,rot))
            # compensate and flow
            b_warp = cv.warpAffine(b, M, (W,H), flags=cv.INTER_LINEAR+cv.WARP_INVERSE_MAP)
            flow = cv.calcOpticalFlowFarneback(a, b_warp, None, 0.5, 3, 25, 3, 5, 1.2, 0)
            wobble.append(wobble_index(flow))
        blur_vlap.append(blur_index_varlap(a))
        blur_freq.append(blur_index_freq(a))
    trans = np.array(trans, float)
    # vibration PSD (detrend and welch)
    valid = ~np.isnan(trans).any(1)
    psd = {}
    if valid.sum() > 8:
        nperseg = min(256, valid.sum())
        for i,name in enumerate(["dx","dy","rot"]):
            sig = detrend(trans[valid,i])
            f, Pxx = welch(sig, fs=fps, nperseg=nperseg)
            psd[name] = (f, Pxx)
    return {
        "wobble_index": np.array(wobble),
        "blur_varlap": np.array(blur_vlap),
        "blur_freq_ratio": np.array(blur_freq),
        "transforms": trans,
        "psd": psd
    }
```

## How to interpret

* **WobbleIndex**: 0.0–0.002 is mild, 0.002–0.01 moderate, >0.01 severe for 1080p footage. Calibrate on your dataset.
* **Blur**: track relative changes over time; absolute thresholds depend on optics and ISO.
* **PSD peaks** near the propeller RPM harmonics indicate vibration. With low FPS, expect mirrored peaks at (f_s!-!f).

If you share a short clip or a few frames with the FPS, I can run this and return the metrics and plots.

"""