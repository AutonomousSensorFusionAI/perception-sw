#!/usr/bin/env python3
# calib_both_ros2.py
import argparse, os, glob, math
import numpy as np
import cv2 as cv
from datetime import datetime

# ---------- 유틸 ----------
def row(a): return ", ".join(f"{x:.15g}" for x in a.reshape(-1))

def save_yaml_plumb_bob(path, name, size, K, D):
    w, h = size
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

def rmse_all(obj, img, rvecs, tvecs, proj_fn):
    se, N = 0.0, 0
    for i in range(len(img)):
        proj = proj_fn(obj[i], rvecs[i], tvecs[i])
        e = img[i].reshape(-1,2) - proj.reshape(-1,2)
        se += float(np.sum(e*e)); N += e.shape[0]
    return math.sqrt(se/max(1,N))

def sanity_K(size, K):
    w, h = size
    fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
    return (0.2*w < fx < 3*w and 0.2*w < fy < 3*w and 0.3*w < cx < 0.7*w and 0.3*h < cy < 0.7*h)

# ---------- 입력 수집 ----------
def collect_points(img_dir, cols, rows, square):
    exts = ("*.png","*.PNG","*.jpg","*.JPG","*.jpeg","*.JPEG")
    files = sorted(sum([glob.glob(os.path.join(img_dir, e)) for e in exts], []))
    if not files: raise SystemExit(f"No images under {img_dir}")
    im0 = cv.imread(files[0], cv.IMREAD_COLOR)
    if im0 is None: raise SystemExit(f"Failed to read {files[0]}")
    size = (im0.shape[1], im0.shape[0])  # (w,h)

    obj1 = np.zeros((rows*cols,1,3), np.float32)
    obj1[:,0,:2] = np.mgrid[0:cols,0:rows].T.reshape(-1,2) * float(square)

    obj, img, used = [], [], []
    use_sb = hasattr(cv, "findChessboardCornersSB")
    for f in files:
        im = cv.imread(f, cv.IMREAD_COLOR)
        if im is None or (im.shape[1], im.shape[0]) != size: continue
        gray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
        if use_sb:
            ok, corners = cv.findChessboardCornersSB(gray, (cols, rows), flags=cv.CALIB_CB_NORMALIZE_IMAGE)
        else:
            flags = (cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_FAST_CHECK)
            ok, corners = cv.findChessboardCorners(gray, (cols, rows), flags)
            if ok:
                corners = cv.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                          (cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER,30,1e-3))
        if not ok: continue
        obj.append(obj1.copy())
        img.append(corners.reshape(-1,1,2).astype(np.float32))
        used.append(f)

    if len(used) < 12:
        raise SystemExit(f"Need >=12 boards, got {len(used)}")
    return size, obj, img, used

# ---------- 메인 ----------
def main():
    ap = argparse.ArgumentParser(description="Pinhole & Fisheye calibration -> ROS2 YAML")
    ap.add_argument("--img_dir", default="/home/wise/Downloads/ChessboardImages", required=False, help="체스보드 이미지 폴더")
    ap.add_argument("--cols", type=int, default=8, help="내부 코너(가로)")
    ap.add_argument("--rows", type=int, default=6, help="내부 코너(세로)")
    ap.add_argument("--square", type=float, default=1.0, help="한 칸 길이(미터). 모르면 1.0")
    ap.add_argument("--name", default="calib_cam")
    ap.add_argument("--out_dir", default="calib_out")
    ap.add_argument("--alpha", type=float, default=0.0, help="pinhole crop(0)~keep(1)")
    ap.add_argument("--balance", type=float, default=0.0, help="fisheye crop(0)~FOV(1)")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    size, obj, img, used = collect_points(args.img_dir, args.cols, args.rows, args.square)
    w, h = size
    print(f"Images used: {len(used)}, size={size}")

    # -------- pinhole --------
    ret_p, Kp, Dp, rp, tp = cv.calibrateCamera(obj, img, size, None, None)
    Kopt, _ = cv.getOptimalNewCameraMatrix(Kp, Dp, size, args.alpha, size)
    raw0 = cv.imread(used[0])
    und_p = cv.undistort(raw0, Kp, Dp, None, Kopt)
    cv.imwrite(os.path.join(args.out_dir, "undist_pinhole.jpg"), und_p)
    save_yaml_plumb_bob(os.path.join(args.out_dir, "camera_plumb_bob.yaml"),
                        args.name, size, Kp, Dp)
    rmse_p2 = rmse_all(obj, img, rp, tp, lambda o,r,t: cv.projectPoints(o,r,t,Kp,Dp)[0])
    print(f"[pinhole] RMS={ret_p:.4f} px, sanity_RMSE={rmse_p2:.4f} px")

    # -------- fisheye --------
    f0 = 0.8*max(w,h)
    K0 = np.array([[f0,0,w/2],[0,f0,h/2],[0,0,1]], np.float64)
    D0 = np.zeros((4,1), np.float64)
    flags = (cv.fisheye.CALIB_USE_INTRINSIC_GUESS |
             cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
             cv.fisheye.CALIB_CHECK_COND |
             cv.fisheye.CALIB_FIX_SKEW |
             cv.fisheye.CALIB_FIX_PRINCIPAL_POINT)
    ret_f, Kf, Df, rf, tf = cv.fisheye.calibrate(
        obj, img, size, K0, D0, None, None,
        flags=flags,
        criteria=(cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER, 60, 1e-7)
    )
    # 타당성 검사(발산 방지)
    if not sanity_K(size, Kf):
        print("[fisheye] Intrinsics out of plausible range -> 데이터/스트림/패턴을 점검하세요.")
    Knew = Kf.copy()
    cv.fisheye.estimateNewCameraMatrixForUndistortRectify(Kf, Df, size, np.eye(3),
                                                          Knew, args.balance, size, 1.0)
    und_f = cv.fisheye.undistortImage(raw0, Kf, Df, Knew=Knew, new_size=size)
    cv.imwrite(os.path.join(args.out_dir, "undist_fisheye.jpg"), und_f)
    save_yaml_fisheye(os.path.join(args.out_dir, "camera_fisheye.yaml"),
                      args.name, size, Kf, Df)
    rmse_f2 = rmse_all(obj, img, rf, tf, lambda o,r,t: cv.fisheye.projectPoints(o,r,t,Kf,Df)[0])
    print(f"[fisheye] RMS={ret_f:.4f} px, sanity_RMSE={rmse_f2:.4f} px  (balance={args.balance})")

    # -------- 미리보기 --------
    if args.show:
        side = np.hstack([raw0, und_p, und_f])
        cv.imshow("raw | pinhole | fisheye", side); cv.waitKey(0)

    # -------- 선택 가이드 출력 --------
    print("\n[가이드]")
    print("- 수치/미리보기를 함께 보고 모델을 선택하세요.")
    print("- 스트림이 이미 디워프된 경우 pinhole이 보통 더 낮은 RMS와 자연스러운 가장자리를 보입니다.")
    print("- RAW(디워프 전)라면 fisheye가 유리합니다. balance=0.0~0.3에서 조정해 보세요.")

if __name__ == "__main__":
    main()
