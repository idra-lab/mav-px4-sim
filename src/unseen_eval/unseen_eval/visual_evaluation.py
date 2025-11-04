from unseen_eval.unseen_eval_lib import *

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
    orb = cv2.ORB_create(2000)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    motions = []
    residuals = []

    trans = []
    wobble = []
    blur_vlap = []
    blur_freq = []
    H, W = images[0].shape
    print(W, H)
    
    for i in range(len(images) - 1):
        kp1, des1 = orb.detectAndCompute(images[i], None)
        kp2, des2 = orb.detectAndCompute(images[i + 1], None)
        if des1 is None or des2 is None:
            motions.append(np.zeros(3))
            residuals.append(0)
            continue

        matches = bf.match(des1, des2)
        if len(matches) < 8:
            motions.append(np.zeros(3))
            residuals.append(0)
            continue

        if plot_matches:
            img_match = cv2.drawMatches(images[i],kp1,images[i + 1],kp2,matches,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
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

        H, _ = cv2.findHomography(pts1, pts2, cv2.RANSAC)

        pts2_pred = cv2.perspectiveTransform(pts1[None, :, :], H)[0]
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
    return {
        "wobble_index": np.array(wobble),
        "blur_varlap": np.array(blur_vlap),
        "blur_freq_ratio": np.array(blur_freq),
        "transforms": trans,
        "psd": psd,
        "residuals": np.array(residuals)
    }


def compute_shakiness(motions, fps=30.0, cutoff=2.0):
    if len(motions) < 2:
        return 0.0
    b, a = butter_highpass(cutoff, fps)
    filtered = filtfilt(b, a, motions, axis=0)
    rms = np.sqrt(np.mean(np.sum(filtered**2, axis=1)))
    return rms


def analyze_sequence(folder): 
    images = load_images(folder)
    print(f"Loaded {len(images)} frames.")
    res_dict = extract_motions(images)

    return res_dict


def main(folder):
    images = load_images(folder)
    print(f"Loaded {len(images)} frames.")
    res_dict = extract_motions(images)
    # J_vis = compute_shakiness(motions)
    # print(f"Visual shakiness (RMS residual angle, deg): {np.degrees(J_vis):.3f}")
    print(f"Mean wobble : {np.mean(res_dict['wobble_index']):.3f}")
    print(f"Mean blur variance (Laplacian): {np.mean(res_dict['blur_varlap']):.3f}")
    print(f"Mean blur frequency ratio: {np.mean(res_dict['blur_freq_ratio']):.3f}")
    print(f"Median tracking residual (pixels): {np.median(res_dict['residuals']):.3f}")
    print(f"95th percentile tracking residual (pixels): {np.percentile(res_dict['residuals'],95):.3f}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Compute visual shakiness and tracking error.")
    parser.add_argument("folder", help="Path to directory with sequential images")
    args = parser.parse_args()
    main(args.folder)

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