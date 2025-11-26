#include "camera_geometry.hpp"

std::string video_path = "/home/wise/ssd/ssd_Downloads/sjy3/sjy3_projects/lane_detection_test/test_720_clipped.mp4";
std::string image_path = "/home/wise/Downloads/0001.jpg";

#define W_ORG 1280
#define H_ORG 720


 // ===== [ADD] 유틸: 행렬 정규화(스케일 제거) =====
static cv::Mat normalizeH(const cv::Mat& H) {
    CV_Assert(H.rows==3 && H.cols==3);
    cv::Mat Hn = H.clone();
    double s = Hn.at<double>(2,2);
    if (std::abs(s) < 1e-12) {
        // fallback: Frobenius norm
        s = cv::norm(Hn);
    }
    Hn /= s;
    return Hn;
}

// ===== [ADD] 유틸: H(이론) 구성 =====
// H_theory = K * [r1 r2 t]  (Z=0 지면 가정, 언디스토션(P=K) 픽셀 도메인)
static cv::Mat makeH_theory(const cv::Mat& K, const cv::Mat& R_w2c, const cv::Mat& t_w2c) {
    cv::Mat M(3,3,CV_64F);
    R_w2c.col(0).copyTo(M.col(0)); // r1
    R_w2c.col(1).copyTo(M.col(1)); // r2
    t_w2c.copyTo(M.col(2));        // t
    return K * M;
}

// ===== [ADD] 평면 호모그래피로부터 (R,t) 초기치 복원 =====
static void extrinsicFromHomography(const cv::Mat& H_w2i, const cv::Mat& K,
                                    cv::Mat& R_w2c, cv::Mat& t_w2c)
{
    cv::Mat Kinv = K.inv();
    cv::Mat B = Kinv * H_w2i;                   // [b1 b2 b3]
    cv::Mat b1 = B.col(0), b2 = B.col(1), b3 = B.col(2);

    double lambda = 1.0 / cv::norm(b1);
    cv::Mat r1 = lambda * b1;
    cv::Mat r2 = lambda * b2;
    cv::Mat r3 = r1.cross(r2);

    cv::Mat R(3,3,CV_64F);
    r1.copyTo(R.col(0));
    r2.copyTo(R.col(1));
    r3.copyTo(R.col(2));

    // 정규직교화 (SVD)
    cv::SVD svd(R);
    R = svd.u * svd.vt;

    cv::Mat t = lambda * b3;

    R_w2c = R.clone();
    t_w2c = t.clone();
}

// ===== [ADD] 원인분리: A/B/C RMSE + H 비교 =====
static void diagnoseRMSE(const std::vector<cv::Point3d>& obj3d,
                         const std::vector<cv::Point2d>& img2d_raw,
                         const cv::Mat& K, const cv::Mat& D,
                         const cv::Mat& R_w2c, const cv::Mat& t_w2c)
{
    // A) RAW 도메인 RMSE: projectPoints vs 원본 픽셀
    cv::Mat rvec; cv::Rodrigues(R_w2c, rvec);
    cv::Mat tvec = t_w2c.clone();

    std::vector<cv::Point2d> proj_raw;
    cv::projectPoints(obj3d, rvec, tvec, K, D, proj_raw);

    auto rmse = [](const std::vector<cv::Point2d>& a,
                   const std::vector<cv::Point2d>& b){
        CV_Assert(a.size()==b.size());
        double se=0;
        for (size_t i=0;i<a.size();++i) {
            double e = cv::norm(a[i]-b[i]);
            se += e*e;
        }
        return std::sqrt(se / a.size());
    };
    double rmseA = rmse(proj_raw, img2d_raw);

    // B) RECT 도메인: (project→undistort) vs (측정→undistort), 모두 P=K
    std::vector<cv::Point2d> proj_rect, meas_rect;
    cv::undistortPoints(proj_raw, proj_rect, K, D, cv::noArray(), K);
    cv::undistortPoints(img2d_raw, meas_rect, K, D, cv::noArray(), K);
    double rmseB = rmse(proj_rect, meas_rect);

    // C) RECT 도메인: H=K[r1 r2 t]로 (X,Y,0) → 픽셀 vs meas_rect
    cv::Mat Hth = makeH_theory(K, R_w2c, t_w2c);
    Hth = normalizeH(Hth);
    double seC=0;
    for (size_t i=0;i<obj3d.size();++i) {
        cv::Mat X = (cv::Mat_<double>(3,1) << obj3d[i].x, obj3d[i].y, 1.0);
        cv::Mat ph = Hth * X;
        double w = ph.at<double>(2);
        cv::Point2d p(ph.at<double>(0)/w, ph.at<double>(1)/w);
        double e = cv::norm(p - meas_rect[i]);
        seC += e*e;
    }
    double rmseC = std::sqrt(seC / obj3d.size());

    // D) H_dlt vs H_theory 비교 (같은 대응쌍으로 직접 추정)
    //    meas_rect (RECT 픽셀)과 obj XY(지면)로 findHomography
    std::vector<cv::Point2f> XY2, UVr2;
    XY2.reserve(obj3d.size()); UVr2.reserve(obj3d.size());
    for (size_t i=0;i<obj3d.size();++i) {
        XY2.emplace_back((float)obj3d[i].x, (float)obj3d[i].y);
        UVr2.emplace_back((float)meas_rect[i].x, (float)meas_rect[i].y);
    }
    cv::Mat Hdlt = cv::findHomography(XY2, UVr2, 0);
    Hdlt.convertTo(Hdlt, CV_64F);
    Hdlt = normalizeH(Hdlt);

    cv::Mat diff = cv::abs(Hdlt - Hth);
    double maxAbsDiff;
    cv::minMaxLoc(diff, nullptr, &maxAbsDiff);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n[RMSE] A RAW(projectPoints vs raw):       " << rmseA << " px\n";
    std::cout << "[RMSE] B RECT(proj→undist vs meas→undist): " << rmseB << " px\n";
    std::cout << "[RMSE] C RECT(H_theory vs meas→undist):    " << rmseC << " px\n";
    std::cout << "[H] max|H_dlt - H_theory| (scaled):        " << maxAbsDiff << "\n";
}

// ===== [ADD] 레거시 H를 초기치로 써서 RefineLM 하는 예 (옵션) =====
static void refineWithHinit(const std::vector<cv::Point3d>& obj3d,
                            const std::vector<cv::Point2d>& img2d_raw,
                            const cv::Mat& K, const cv::Mat& D,
                            const cv::Mat& Hdlt,
                            cv::Mat& R_w2c, cv::Mat& t_w2c)
{
    // 1) H로 초기치 얻기
    extrinsicFromHomography(Hdlt, K, R_w2c, t_w2c);

    // 2) RefineLM
    cv::Mat rvec; cv::Rodrigues(R_w2c, rvec);
    cv::Mat tvec = t_w2c.clone();
    cv::solvePnPRefineLM(obj3d, img2d_raw, K, D, rvec, tvec);

    cv::Rodrigues(rvec, R_w2c);
    t_w2c = tvec.clone();
}

int main()
{
    /// intrinsic
    /// TODO: 정의하는 부분 깔끔하게 변경
    // cv::Mat K = (cv::Mat_<double>(3, 3) << 
    //             998.448098702023, 0., 626.10517681474914,
    //             0., 965.60320161042659, 389.02140114294514,
    //             0., 0., 1.
    //         );
    // cv::Mat D = (cv::Mat_<double>(1, 5) << 
    //             -0.53496753163969957, 0.38034527449155769, 0.0072045688856163005, 0.037416277471075607, -0.14706304746237694
    //         );

    /* 250911 fisheye 모델로 다시 */
    cv::Mat K = (cv::Mat_<double>(3, 3) << 
                    557.286481458488, 0, 645.434763202952,
                    0, 560.2662472618434, 422.5041216026116,
                    0, 0, 1
                );
    cv::Mat D = (cv::Mat_<double>(1, 4) << 
                -0.04717653045611134, 0.08466535058959987, -0.1122427643930151, 0.02860302134167773
                ); 

    /// solvePnP -> extrinsic
    // 3D 지면 좌표 (X: 전방+, Y: 좌측+)
    std::vector<cv::Point3d> p_3d;
    std::vector<cv::Point2d> p_2d;

    /* 250902 KNUT */
    p_3d.push_back(cv::Point3d(4.845, 5.08, 0.0));
    p_2d.push_back(cv::Point2d(323, 554)); 
    // p_2d.push_back(cv::Point2d(313, 556)); 

    p_3d.push_back(cv::Point3d(4.73, -3.95, 0.0));   
    p_2d.push_back(cv::Point2d(967, 549)); 
    // p_2d.push_back(cv::Point2d(967, 552)); 

    p_3d.push_back(cv::Point3d(15.74, 4.68, 0.0));   
    p_2d.push_back(cv::Point2d(517, 514)); 
    // p_2d.push_back(cv::Point2d(522, 517)); 

    p_3d.push_back(cv::Point3d(14.235, -8.44, 0.0));   
    p_2d.push_back(cv::Point2d(940, 501)); 
    // p_2d.push_back(cv::Point2d(943, 502)); 

    std::vector<cv::Point2d> p_2d_undist;
    cv::undistortPoints(p_2d, p_2d_undist, K, D, cv::noArray(), K);

    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(p_3d, p_2d, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE);
    // bool success = cv::solvePnP(p_3d, p_2d_undist, K, D, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    if (!success) {
        std::cerr << "solvePnP failed!" << std::endl;
        return -1;
    }

    cv::Mat R_w2c; cv::Rodrigues(rvec, R_w2c);
    cv::Mat t_w2c = tvec; // tvec은 이미 월드->카메라 이동 벡터 t_w2c
    
    // 파라미터 업데이트
    CamCalib calib = CamCalib::Make(K, D, R_w2c, t_w2c);

    std::cout << "--- solvePnP Results ---" << std::endl;
    std::cout << "Cw    (Camera center in world ):\n" << calib.Cw << std::endl << std::endl;
    std::cout << "R_w2c (Rotation Matrix, World to Camera):\n" << R_w2c << std::endl << std::endl;
    std::cout << "t_w2c (Translation Vector, World to Camera):\n" << t_w2c << std::endl << std::endl;

    /* 거리 측정 검증 */
    for (int ii; ii<p_2d.size(); ++ii) {
        std::cout << "\n--- pixel to ground(world) Verification ---" << std::endl;
        std::cout << "point " << ii+1 << ": pic [" << p_2d[ii] << "] -> world ["
                                      << pixToGndRaw(p_2d[ii], calib) << "]"
                                      << std::endl;
    }

    /* reprojection error 검증 */
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    cv::Mat undistorted_image; // 보정된 이미지를 저장할 빈 Mat 객체 생성
    cv::undistort(image, undistorted_image, K, D);
    // cv::fisheye::undistortImage(image, undistorted_image, K, D);

    /////
    std::cout << "\n--- Reprojection Verification ---" << std::endl;

    /* 실제 점 Projection */
    std::vector<cv::Point2d> p_proj, p_proj_undist;
    
    std::vector<cv::Point3d> guide;
    guide.push_back(cv::Point3d(25.0, 3.0, 0.0));
    guide.push_back(cv::Point3d(25.0, -3.0, 0.0));
    guide.push_back(cv::Point3d(5.0, 3.0, 0.0));
    guide.push_back(cv::Point3d(5.0, -3.0, 0.0));
    
    // cv::projectPoints(p_3d, rvec, tvec, K, D, p_proj);
    // cv::undistortPoints(p_proj, p_proj_undist, K, D, cv::noArray(), K);
    cv::projectPoints(guide, rvec, tvec, K, D, p_proj);
    cv::undistortPoints(p_proj, p_proj_undist, K, D, cv::noArray(), K);

    std::cout << std::fixed << std::setprecision(2);
    double total_error = 0.0;

    for (size_t i = 0; i < p_3d.size(); ++i) {
        // cv::Point2d original_pixel = p_2d[i];
        cv::Point2d original_pixel = p_2d_undist[i];
        cv::Point2d world_point_2d(p_3d[i].x, p_3d[i].y);
        
        try {
            cv::Point2d reprojected_pixel = gndToPix(world_point_2d, calib);
            double error = cv::norm(original_pixel - reprojected_pixel);
            total_error += error * error;

            std::cout << "Point " << i+1 << ": World " << world_point_2d 
                      << " -> Reprojected Pixel " << reprojected_pixel 
                      << " (Original: " << original_pixel << "), Error: " << error << " pixels" << std::endl;

            // pic->gnd
            cv::Point2d p_2dTo3d = pixToGndRaw(p_2d[i], calib);
            std::cout << "Point " << i+1 << ": Pixel " << p_2d[i]
                      << " -> World " << p_2dTo3d 
                      << " | error: X " << p_3d[i].x - p_2dTo3d.x << "m, Y " << p_3d[i].y - p_2dTo3d.y << "m"
                      << std::endl;

            // 원본 측정점
            cv::circle(undistorted_image, original_pixel, 10, cv::Scalar(255, 0, 0), 2);
            cv::circle(undistorted_image, original_pixel, 3, cv::Scalar(255, 0, 0), 2);
            // cv::putText(undistorted_image, "GT " + std::to_string(i+1), original_pixel + cv::Point2d(10, 0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            
            // 계산된 재투영점
            cv::drawMarker(undistorted_image, reprojected_pixel, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 20, 2);
            cv::drawMarker(undistorted_image, p_proj_undist[i], cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 10, 2);
            
        } catch (const std::runtime_error& e) {
            std::cerr << "gndToPix Error for point " << i+1 << ": " << e.what() << std::endl;
        }
    }

    double rmse = std::sqrt(total_error / p_3d.size());
    std::cout << "\nOverall Reprojection Error (RMSE): " << rmse << " pixels" << std::endl;


    /* 에러 분석 */
    // // [ADD] 원인분석: A/B/C RMSE + H 비교
    // diagnoseRMSE(p_3d, p_2d, K, D, R_w2c, t_w2c);

    // // [ADD - 옵션] 레거시와 동일한 도메인(RECT, P=K)이라면,
    // //              같은 짝으로 H_dlt를 만들고 이를 초기치로 RefineLM 해볼 수 있음
    // {
    //     std::vector<cv::Point2d> meas_rect;
    //     cv::undistortPoints(p_2d, meas_rect, K, D, cv::noArray(), K);

    //     std::vector<cv::Point2f> XY, UVr;
    //     XY.reserve(p_3d.size()); UVr.reserve(p_3d.size());
    //     for (size_t i=0;i<p_3d.size(); ++i) {
    //         XY.emplace_back((float)p_3d[i].x, (float)p_3d[i].y);
    //         UVr.emplace_back((float)meas_rect[i].x, (float)meas_rect[i].y);
    //     }
    //     cv::Mat Hdlt = cv::findHomography(XY, UVr, 0);
    //     Hdlt.convertTo(Hdlt, CV_64F);

    //     cv::Mat Rinit, tinit;
    //     refineWithHinit(p_3d, p_2d, K, D, Hdlt, Rinit, tinit);

    //     std::cout << "\n[Refine-with-Hinit] done.\n";
    //     diagnoseRMSE(p_3d, p_2d, K, D, Rinit, tinit);
    // }

    // 이미지 표시
    std::string window_title = "RMSE: " + std::to_string(rmse) + " pixels (blue: GT, red: reprojected)";
    cv::imshow(window_title, undistorted_image);

    std::cout << "\nVerification image displayed. Press any key to exit." << std::endl;
    cv::waitKey(0);

    return 0;
    
}