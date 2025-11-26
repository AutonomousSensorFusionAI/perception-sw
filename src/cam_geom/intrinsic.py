#!/usr/bin/env python3
import argparse, os, glob, math
import numpy as np
import cv2 as cv
from datetime import datetime

def row(a): return ", ".join(f"{x:.15g}" for x in a.reshape(-1))

def save_yaml(path, name, size, K, D, model):
    w, h = size
    P = np.hstack([K, np.zeros((3,1))])
    if model == "plumb_bob":        # 길이 5로 고정(부족분 0)
        D5 = np.zeros((5,), float); D5[:min(5, D.size)] = D.reshape(-1)[:min(5, D.size)]
        drows, dcols, ddata = 1, 5, row(D5)
    else:                           # equidistant: 길이 4
        drows, dcols, ddata = 1, 4, row(D)
    txt = f"""# generated {datetime.now().isoformat(timespec='seconds')}
image_width: {w}
image_height: {h}
camera_name: {name}
camera_matrix:
  rows: 3
  cols: 3
  data: [{row(K)}]
distortion_model: {model}
distortion_coefficients:
  rows: {drows}
  cols: {dcols}
  data: [{ddata}]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [{row(P)}]
"""
    with open(path, "w") as f: f.write(txt)

def collect(img_dir, cols, rows):
    exts = ("*.png","*.PNG","*.jpg","*.JPG","*.jpeg","*.JPEG")
    files = sorted(sum([glob.glob(os.path.join(img_dir, e)) for e in exts], []))
    if not files: raise SystemExit(f"No images under {img_dir}")
    sample = cv.imread(files[0], cv.IMREAD_COLOR)
    if sample is None: raise SystemExit(f"Failed to read {files[0]}")
    size = (sample.shape[1], sample.shape[0])  # (w,h)

    obj1 = np.zeros((rows*cols,1,3), np.float32)
    obj1[:,0,:2] = np.mgrid[0:cols,0:rows].T.reshape(-1,2)  # square 모르면 1.0로

    obj, img, used = [], [], []
    flags = (cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_FAST_CHECK)
    for f in files:
        im = cv.imread(f, cv.IMREAD_COLOR)
        if im is None or (im.shape[1], im.shape[0]) != size: continue
        gray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
        ok, corners = cv.findChessboardCorners(gray, (cols, rows), flags)
        if not ok: continue
        corners = cv.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                  (cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER,30,1e-3))
        obj.append(obj1.copy()); img.append(corners.reshape(-1,1,2).astype(np.float32)); used.append(f)
    if len(used) < 8: raise SystemExit(f"Need >=8 boards, got {len(used)}")
    return size, obj, img, used

def rmse_all(obj, img, rvecs, tvecs, proj_fn):
    se, N = 0.0, 0
    for i in range(len(img)):
        proj = proj_fn(obj[i], rvecs[i], tvecs[i])
        e = img[i].reshape(-1,2) - proj.reshape(-1,2)
        se += float(np.sum(e*e)); N += e.shape[0]
    return math.sqrt(se/max(1,N))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--img_dir", default="/home/wise/Downloads/ChessboardImages", required=False)
    ap.add_argument("--cols", type=int, default=8)  # 내부 코너(가로)
    ap.add_argument("--rows", type=int, default=6)  # 내부 코너(세로)
    ap.add_argument("--name", default="calib_cam")
    ap.add_argument("--out_dir", default="calib_out")
    ap.add_argument("--alpha", type=float, default=0.0, help="pinhole crop(0)~keep(1)")
    ap.add_argument("--balance", type=float, default=0.0, help="fisheye crop(0)~FOV(1)")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    size, obj, img, used = collect(args.img_dir, args.cols, args.rows)
    print(f"Images used: {len(used)}, size={size}")

    # ---- pinhole
    ret_p, Kp, Dp, rp, tp = cv.calibrateCamera(obj, img, size, None, None)
    rmse_p2 = rmse_all(obj, img, rp, tp, lambda o,r,t: cv.projectPoints(o,r,t,Kp,Dp)[0])
    Kopt, _ = cv.getOptimalNewCameraMatrix(Kp, Dp, size, args.alpha, size)
    raw0 = cv.imread(used[0]); und_p = cv.undistort(raw0, Kp, Dp, None, Kopt)
    cv.imwrite(os.path.join(args.out_dir, "undist_pinhole2.jpg"), und_p)
    save_yaml(os.path.join(args.out_dir, "camera_plumb_bob2.yaml"), args.name, size, Kp, Dp, "plumb_bob")
    print(f"[pinhole] RMS={ret_p:.4f} px, sanity_RMSE={rmse_p2:.4f} px")

    # ---- fisheye
    Kf = np.eye(3, dtype=np.float64); Df = np.zeros((4,1), np.float64)
    flags = (cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv.fisheye.CALIB_CHECK_COND | cv.fisheye.CALIB_FIX_SKEW)
    ret_f, Kf, Df, rf, tf = cv.fisheye.calibrate(obj, img, size, Kf, Df, None, None,
                                                 flags=flags,
                                                 criteria=(cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER,50,1e-6))
    rmse_f2 = rmse_all(obj, img, rf, tf, lambda o,r,t: cv.fisheye.projectPoints(o,r,t,Kf,Df)[0])
    Knew = Kf.copy()
    cv.fisheye.estimateNewCameraMatrixForUndistortRectify(Kf, Df, size, np.eye(3), Knew, args.balance, size, 1.0)
    und_f = cv.fisheye.undistortImage(raw0, Kf, Df, Knew=Knew, new_size=size)
    cv.imwrite(os.path.join(args.out_dir, "undist_fisheye2.jpg"), und_f)
    save_yaml(os.path.join(args.out_dir, "camera_fisheye2.yaml"), args.name, size, Kf, Df, "equidistant")
    print(f"[fisheye] RMS={ret_f:.4f} px, sanity_RMSE={rmse_f2:.4f} px  (balance={args.balance})")

    if args.show:
        side = np.hstack([raw0, und_p, und_f])
        cv.imshow("raw | pinhole | fisheye", side); cv.waitKey(0)

if __name__ == "__main__":
    main()
