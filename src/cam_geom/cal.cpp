#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <iostream>
#include <iomanip>

using std::cout; using std::endl;

struct FisheyeCalib {
    cv::Mat K;          // 3x3
    cv::Mat D;          // 1x4 (k1,k2,k3,k4)
    cv::Size imageSize; // 원본 해상도
    double rms = 0.0;   // reprojection RMS (pixels)

    void saveYAML(const std::string& path) const {
        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        fs << "model" << "fisheye"; // 메타
        fs << "image_width"  << imageSize.width;
        fs << "image_height" << imageSize.height;
        fs << "camera_matrix" << K;
        fs << "dist_coeff"    << D;
        fs.release();
    }

};

static bool findCorners(const cv::Mat& gray,
                        cv::Size pattern, std::vector<cv::Point2f>& corners)
{
    int flags = cv::CALIB_CB_ADAPTIVE_THRESH |
                cv::CALIB_CB_NORMALIZE_IMAGE |
                cv::CALIB_CB_FAST_CHECK;
    bool ok = cv::findChessboardCorners(gray, pattern, corners, flags);
    if (!ok) return false;
    cv::cornerSubPix(gray, corners, cv::Size(11,11), cv::Size(-1,-1),
                     cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 1e-3));
    return true;
}

static FisheyeCalib calibrateFisheye(const std::string& imgDir,
                                     cv::Size pattern /*(cols,rows)*/,
                                     float squareSize /*meters or any unit*/)
{
    std::vector<std::vector<cv::Point2f>> imgPts;
    std::vector<std::vector<cv::Point3f>> objPts;
    cv::Size imgSize{0,0};

    std::vector<cv::String> files;
    cv::utils::fs::glob(imgDir, "*.jpg", files, false);
    cv::utils::fs::glob(imgDir, "*.png", files, true);

    if (files.empty()) throw std::runtime_error("No images in "+imgDir);

    // 준비: 체스보드 3D 좌표(단위: squareSize)
    std::vector<cv::Point3f> obj;
    obj.reserve(pattern.area());
    for (int r=0; r<pattern.height; ++r)
        for (int c=0; c<pattern.width; ++c)
            obj.emplace_back(c * squareSize, r * squareSize, 0.0f);

    size_t used = 0;
    for (auto& f : files) {
        cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);
        if (img.empty()) continue;
        if (imgSize.area()==0) imgSize = img.size();
        if (img.size()!=imgSize) {
            // 해상도 혼선 방지: 리사이즈/크롭된 이미지는 스킵
            continue;
        }
        cv::Mat gray; cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        if (!findCorners(gray, pattern, corners)) continue;

        imgPts.push_back(corners);
        objPts.push_back(obj);
        ++used;
    }
    if (used < 8) throw std::runtime_error("Need >=8 valid boards, got "+std::to_string(used));

    // fisheye 캘리브레이션
    FisheyeCalib res;
    res.imageSize = imgSize;
    res.K = cv::Mat::eye(3,3,CV_64F);
    res.D = cv::Mat::zeros(4,1,CV_64F);

    int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
                cv::fisheye::CALIB_CHECK_COND        |
                cv::fisheye::CALIB_FIX_SKEW;
    std::vector<cv::Mat> rvecs, tvecs;
    res.rms = cv::fisheye::calibrate(objPts, imgPts, imgSize, res.K, res.D, rvecs, tvecs, flags,
                     cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 50, 1e-6));
    cout << "[fisheye] RMS reprojection error = " << res.rms << " px\n";
    cout << "K=\n" << res.K << "\nD=\n" << res.D.t() << endl;

    // 간단한 sanity check: fisheye project로 직접 측정 RMSE
    // (cv::fisheye::projectPoints 사용)
    double se=0; size_t N=0;
    for (size_t i=0;i<imgPts.size();++i) {
        std::vector<cv::Point2f> proj;
        cv::fisheye::projectPoints(objPts[i], proj, rvecs[i], tvecs[i], res.K, res.D);
        for (size_t k=0;k<proj.size();++k) {
            double e = cv::norm(proj[k] - imgPts[i][k]);
            se += e*e; ++N;
        }
    }
    double rmse2 = std::sqrt(se / std::max<size_t>(1,N));
    cout << "[fisheye] sanity RMSE = " << rmse2 << " px\n";
    return res;
}

// fisheye → RECT(pinhole) 맵 생성. balance∈[0,1] (0:크롭, 1:최대 시야)
static void makeRectifyMap(const FisheyeCalib& c,
                           double balance,
                           cv::Mat& Krect, cv::Mat& map1, cv::Mat& map2)
{
    cv::Mat R = cv::Mat::eye(3,3,CV_64F); // no rectify rotation
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
        c.K, c.D, c.imageSize, R, Krect, balance, c.imageSize, 1.0);
    cv::fisheye::initUndistortRectifyMap(
        c.K, c.D, R, Krect, c.imageSize, CV_16SC2, map1, map2);
}

// PnP with fisheye: 먼저 RECT로 점을 변환하고 D=0으로 solvePnP
static bool solvePnP_onRectified(const std::vector<cv::Point3f>& obj_m,
                                 const std::vector<cv::Point2f>& img_raw,
                                 const FisheyeCalib& c,
                                 const cv::Mat& Krect,
                                 cv::Mat& rvec, cv::Mat& tvec)
{
    // RAW → RECT (P=Krect)
    std::vector<cv::Point2f> img_rect;
    cv::fisheye::undistortPoints(img_raw, img_rect, c.K, c.D, cv::noArray(), Krect);

    // D=0 으로 PnP (IPPE는 평면일 때 안정적)
    return cv::solvePnP(obj_m, img_rect, Krect, cv::noArray(),
                        rvec, tvec, false, cv::SOLVEPNP_IPPE);
}

int main(int argc, char** argv)
{
    const std::string imgDir = "/home/wise/Downloads/ChessboardImages";
    const cv::Size pattern(8,6);   // 내부 코너 (cols, rows)
    const float square = 0.025f;    // 한 칸 2.5cm

    // 1) fisheye 캘리브레이션
    auto calib = calibrateFisheye(imgDir, pattern, square);
    calib.saveYAML("camera_fisheye.yaml");

    // 2) RECT 맵(가상의 pinhole) 생성
    cv::Mat Krect, map1, map2;
    makeRectifyMap(calib, /*balance=*/0.0, Krect, map1, map2);
    cout << "Krect=\n" << Krect << endl;

    // 3) 샘플 이미지로 확인
    std::vector<cv::String> files;
    cv::utils::fs::glob(imgDir, "*.png", files, false);
    if (!files.empty()) {
        cv::Mat raw = cv::imread(files[0]);
        cv::Mat rect; cv::remap(raw, rect, map1, map2, cv::INTER_LINEAR);
        // cv::imwrite("raw.jpg", raw);
        // cv::imwrite("rectified.jpg", rect);
        cv::imshow("raw", raw);
        cv::imshow("rectified(pinhole)", rect);
        cv::waitKey(0);
    }

    // 4) (선택) 실제 PnP 테스트 예시
    // obj_m: 차량 좌표계의 지면 점들 (m), img_raw: 그에 대응하는 RAW 픽셀들
    std::vector<cv::Point3f> obj_m = {  {4.845f, 5.08f, 0}, 
                                        {4.73f, -3.95f, 0},
                                        {15.74f, 4.68f, 0}, 
                                        {14.235f, -8.44f, 0} };
    std::vector<cv::Point2f> img_raw = {{323,554}, 
                                        {967,549}, 
                                        {517,514}, 
                                        {940,501} };

    cv::Mat rvec, tvec;
    if (solvePnP_onRectified(obj_m, img_raw, calib, Krect, rvec, tvec)) {
        cout << "PnP on RECT ok. rvec=" << rvec.t() << "\n tvec=" << tvec.t() << endl;

        // 재투영(RAW 도메인) RMSE 확인: fisheye::projectPoints 사용
        std::vector<cv::Point2f> proj_rect;
        cv::projectPoints(obj_m, rvec, tvec, Krect, cv::noArray(), proj_rect);

        // RECT→RAW로 되돌려 비교할 수도 있지만, 실무에선 img_rect로 GT를 맞춰 비교
        std::vector<cv::Point2f> gt_rect;
        cv::fisheye::undistortPoints(img_raw, gt_rect, calib.K, calib.D, cv::noArray(), Krect);

        double se=0; for (size_t i=0;i<proj_rect.size();++i){ double e=cv::norm(proj_rect[i]-gt_rect[i]); se+=e*e; }
        cout << std::fixed << std::setprecision(3)
             << "RMSE (RECT domain, D=0) = " << std::sqrt(se/proj_rect.size()) << " px\n";
    } else {
        cout << "PnP failed.\n";
    }
    return 0;
}
