import cv2
import numpy as np
import glob
import os
from scipy.signal import butter, filtfilt

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

def extract_motions(images):
    orb = cv2.ORB_create(2000)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    motions = []
    residuals = []

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

        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])

        E, mask = cv2.findEssentialMat(pts1, pts2, focal=1.0, pp=(0., 0.), method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None:
            motions.append(np.zeros(3))
            residuals.append(0)
            continue
        _, R, t, mask_pose = cv2.recoverPose(E, pts1, pts2, focal=1.0, pp=(0., 0.))
        # small-angle rotation vector
        rvec, _ = cv2.Rodrigues(R)
        motions.append(rvec.flatten())
        # median residual flow after removing global motion
        proj = cv2.projectPoints(np.ones((len(pts1),3)), rvec, t, np.eye(3), None)[0].reshape(-1,2)
        res = np.linalg.norm((pts2 - pts1) - (proj - np.median(proj,axis=0)), axis=1)
        residuals.append(np.median(res))
    return np.array(motions), np.array(residuals)

def compute_shakiness(motions, fps=30.0, cutoff=2.0):
    if len(motions) < 2:
        return 0.0
    b, a = butter_highpass(cutoff, fps)
    filtered = filtfilt(b, a, motions, axis=0)
    rms = np.sqrt(np.mean(np.sum(filtered**2, axis=1)))
    return rms

def main(folder):
    images = load_images(folder)
    print(f"Loaded {len(images)} frames.")
    motions, residuals = extract_motions(images)
    J_vis = compute_shakiness(motions)
    print(f"Visual shakiness (RMS residual angle, deg): {np.degrees(J_vis):.3f}")
    print(f"Median tracking residual (pixels): {np.median(residuals):.3f}")
    print(f"95th percentile tracking residual (pixels): {np.percentile(residuals,95):.3f}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Compute visual shakiness and tracking error.")
    parser.add_argument("folder", help="Path to directory with sequential images")
    args = parser.parse_args()
    main(args.folder)
