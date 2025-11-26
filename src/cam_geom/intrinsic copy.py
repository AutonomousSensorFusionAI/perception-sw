#!/usr/bin/env python3
import argparse, os, glob, math
import numpy as np
import cv2 as cv
from datetime import datetime

def row(a): return ", ".join(f"{x:.15g}" for x in a.reshape(-1))

def save_yaml_plumb_bob(path, name, size, K, D):
    w, h = size
    # D는 길이 5까지만 저장(많이 쓰는 관례). 길이가 더 길면 앞 5개만.
    D5 = np.zeros((5,), float); D5[:min(5, D.size)] = D.reshape(-1)[:min(5, D.size)]
    P = np.hstack([K, np.zeros((3,1))])
    txt = f"""# generated {datetime.now().isoformat(timespec='seconds')}
image_width: {w}
image_height: {h}
camera_name: {name}
camera_matrix:
  rows: 3
  cols: 3
  data: [{row(K)}]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [{row(D5)}]
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

def save_yaml_fisheye(path, name, size, K, D):
    w, h = size
    P = np.hstack([K, np.zeros((3,1))])
    txt = f"""# generated {datetime.now().isoformat(timespec='seconds')}
image_width: {w}
image_height: {h}
camera_name: {name}
camera_matrix:
  rows: 3
  cols: 3
  data: [{row(K)}]
distortion_model: equidistant
distortion_coefficients:
  rows: 1
  cols: 4
  data: [{row(D)}]
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

def collect_points(img_dir, cols, rows, square):
    exts = ("*.png","*.PNG","*.jpg","*.JPG","*.jpeg","*.JPEG")
    files = []
    for e in exts: files += glob.glob(os.path.join(img_dir, e))
    files = sorted(files)
    if not files: raise SystemExit(f"No images under {img_dir}")

    sample = cv.imread(files[0], cv.IMREAD_COLOR)
    if sample is None: raise SystemExit(f"Failed to read {files[0]}")
    size = (sample.shape[1], sample.shape[0])  # (w,h)

    obj1 = np.zeros((rows*cols,1,3), np.float32)
    obj1[:,0,:2] = np.mgrid[0:cols,0:rows].T.reshape(-1,2) * float(square)

    objpoints, imgpoints, used_files = [], [], []
    for f in files:
        im = cv.imread(f, cv.IMREAD_COLOR)
        if im is None or (im.shape[1], im.shape[0]) != size: continue
        gray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
        flags = (cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_FAST_CHECK)
        ok, corners = cv.findChessboardCorners(gray, (cols, rows), flags)
        if not ok: continue
        corners = cv.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                  (cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER,30,1e-3))
        objpoints.append(obj1.copy())
        imgpoints.append(corners.reshape(-1,1,2).astype(np.float32))
        used_files.append(f)

    if len(used_files) < 8:
        raise SystemExit(f"Need >=8 boards, got {len(used_files)}")
    return size, objpoints, imgpoints, used_files

def rmse_all(objpoints, imgpoints, rvecs, tvecs, proj_fn):
    se, N = 0.0, 0
    for i in range(len(imgpoints)):
        proj = proj_fn(objpoints[i], rvecs[i], tvecs[i])
        e = imgpoints[i].reshape(-1,2) - proj.reshape(-1,2)
        se += float(np.sum(e*e)); N += e.shape[0]
    return math.sqrt(se/max(1,N))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--img_dir", required=True)
    ap.add_argument("--cols", type=int, default=8, help="내부 코너(가로)")
    ap.add_argument("--rows", type=int, default=6, help="내부 코너(세로)")
    ap.add_argument("--square", type=float, default=0.03, help="체스보드 한 칸(미터)")
    ap.add_argument("--name", default="calib_cam")
    ap.add_argument("--out_dir", default=".")
    args = ap.parse_args()

    size, objpts, imgpts, used = collect_points(args.img_dir, args.cols, args.rows, args.square)
    print(f"Images used: {len(used)}, size={size}")

    # ---- pinhole(plumb_bob)
    Kp, Dp = None, None
    rp, tp = None, None
    ret_p, Kp, Dp, rp, tp = cv.calibrateCamera(objpts, imgpts, size, None, None)
    rmse_p = ret_p
    rmse_p2 = rmse_all(objpts, imgpts, rp, tp,
                       lambda obj, r,t: cv.projectPoints(obj, r, t, Kp, Dp)[0])
    print(f"[pinhole] RMS={rmse_p:.4f} px, sanity_RMSE={rmse_p2:.4f} px")
    cv.imwrite(os.path.join(args.out_dir, "undist_pinhole.jpg"),
               cv.undistort(cv.imread(used[0]), Kp, Dp))

    # ---- fisheye(equidistant)
    Kf = np.eye(3, dtype=np.float64)
    Df = np.zeros((4,1), dtype=np.float64)
    flags = (cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
             cv.fisheye.CALIB_CHECK_COND |
             cv.fisheye.CALIB_FIX_SKEW)
    ret_f, Kf, Df, rf, tf = cv.fisheye.calibrate(objpts, imgpts, size, Kf, Df, None, None,
                                                 flags=flags,
                                                 criteria=(cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER,50,1e-6))
    rmse_f = ret_f
    rmse_f2 = rmse_all(objpts, imgpts, rf, tf,
                       lambda obj, r,t: cv.fisheye.projectPoints(obj, r, t, Kf, Df)[0])
    print(f"[fisheye] RMS={rmse_f:.4f} px, sanity_RMSE={rmse_f2:.4f} px")
    cv.imwrite(os.path.join(args.out_dir, "undist_fisheye.jpg"),
               cv.fisheye.undistortImage(cv.imread(used[0]), Kf, Df))

    # ---- YAML 저장
    os.makedirs(args.out_dir, exist_ok=True)
    save_yaml_plumb_bob(os.path.join(args.out_dir, "camera_plumb_bob.yaml"),
                        args.name, size, Kp, Dp)
    save_yaml_fisheye(os.path.join(args.out_dir, "camera_fisheye.yaml"),
                      args.name, size, Kf, Df)
    print("Saved:",
          os.path.join(args.out_dir, "camera_plumb_bob.yaml"), "and",
          os.path.join(args.out_dir, "camera_fisheye.yaml"))
    print("Undistorted previews: undist_pinhole.jpg, undist_fisheye.jpg")

    # ---- 선택 가이드
    print("\n[선택 가이드]")
    print("- RMS/preview를 비교하세요. fisheye가 확실히 낮거나 가장자리 직선이 더 자연스러우면 equidistant.")
    print("- 스트림이 이미 디워프된 경우 pinhole가 보통 더 낮습니다.")
    print("- ROS2에 넣을 땐, 선택한 모델에 맞춰 'distortion_model'을 정확히 설정하세요.")

if __name__ == "__main__":
    main()
