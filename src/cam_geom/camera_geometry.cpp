#include "camera_geometry.hpp"

//  // ===== [ADD] 유틸: 행렬 정규화(스케일 제거) =====
// static cv::Mat normalizeH(const cv::Mat& H) {
//     CV_Assert(H.rows==3 && H.cols==3);
//     cv::Mat Hn = H.clone();
//     double s = Hn.at<double>(2,2);
//     if (std::abs(s) < 1e-12) {
//         // fallback: Frobenius norm
//         s = cv::norm(Hn);
//     }
//     Hn /= s;
//     return Hn;
// }

// // ===== [ADD] 유틸: H(이론) 구성 =====
// // H_theory = K * [r1 r2 t]  (Z=0 지면 가정, 언디스토션(P=K) 픽셀 도메인)
// static cv::Mat makeH_theory(const cv::Mat& K, const cv::Mat& R_w2c, const cv::Mat& t_w2c) {
//     cv::Mat M(3,3,CV_64F);
//     R_w2c.col(0).copyTo(M.col(0)); // r1
//     R_w2c.col(1).copyTo(M.col(1)); // r2
//     t_w2c.copyTo(M.col(2));        // t
//     return K * M;
// }

// // ===== [ADD] 평면 호모그래피로부터 (R,t) 초기치 복원 =====
// static void extrinsicFromHomography(const cv::Mat& H_w2i, const cv::Mat& K,
//                                     cv::Mat& R_w2c, cv::Mat& t_w2c)
// {
//     cv::Mat Kinv = K.inv();
//     cv::Mat B = Kinv * H_w2i;                   // [b1 b2 b3]
//     cv::Mat b1 = B.col(0), b2 = B.col(1), b3 = B.col(2);

//     double lambda = 1.0 / cv::norm(b1);
//     cv::Mat r1 = lambda * b1;
//     cv::Mat r2 = lambda * b2;
//     cv::Mat r3 = r1.cross(r2);

//     cv::Mat R(3,3,CV_64F);
//     r1.copyTo(R.col(0));
//     r2.copyTo(R.col(1));
//     r3.copyTo(R.col(2));

//     // 정규직교화 (SVD)
//     cv::SVD svd(R);
//     R = svd.u * svd.vt;

//     cv::Mat t = lambda * b3;

//     R_w2c = R.clone();
//     t_w2c = t.clone();
// }

// // ===== [ADD] 원인분리: A/B/C RMSE + H 비교 =====
// static void diagnoseRMSE(const std::vector<cv::Point3d>& obj3d,
//                          const std::vector<cv::Point2d>& img2d_raw,
//                          const cv::Mat& K, const cv::Mat& D,
//                          const cv::Mat& R_w2c, const cv::Mat& t_w2c)
// {
//     // A) RAW 도메인 RMSE: projectPoints vs 원본 픽셀
//     cv::Mat rvec; cv::Rodrigues(R_w2c, rvec);
//     cv::Mat tvec = t_w2c.clone();

//     std::vector<cv::Point2d> proj_raw;
//     cv::projectPoints(obj3d, rvec, tvec, K, D, proj_raw);

//     auto rmse = [](const std::vector<cv::Point2d>& a,
//                    const std::vector<cv::Point2d>& b){
//         CV_Assert(a.size()==b.size());
//         double se=0;
//         for (size_t i=0;i<a.size();++i) {
//             double e = cv::norm(a[i]-b[i]);
//             se += e*e;
//         }
//         return std::sqrt(se / a.size());
//     };
//     double rmseA = rmse(proj_raw, img2d_raw);

//     // B) RECT 도메인: (project→undistort) vs (측정→undistort), 모두 P=K
//     std::vector<cv::Point2d> proj_rect, meas_rect;
//     cv::undistortPoints(proj_raw, proj_rect, K, D, cv::noArray(), K);
//     cv::undistortPoints(img2d_raw, meas_rect, K, D, cv::noArray(), K);
//     double rmseB = rmse(proj_rect, meas_rect);

//     // C) RECT 도메인: H=K[r1 r2 t]로 (X,Y,0) → 픽셀 vs meas_rect
//     cv::Mat Hth = makeH_theory(K, R_w2c, t_w2c);
//     Hth = normalizeH(Hth);
//     double seC=0;
//     for (size_t i=0;i<obj3d.size();++i) {
//         cv::Mat X = (cv::Mat_<double>(3,1) << obj3d[i].x, obj3d[i].y, 1.0);
//         cv::Mat ph = Hth * X;
//         double w = ph.at<double>(2);
//         cv::Point2d p(ph.at<double>(0)/w, ph.at<double>(1)/w);
//         double e = cv::norm(p - meas_rect[i]);
//         seC += e*e;
//     }
//     double rmseC = std::sqrt(seC / obj3d.size());

//     // D) H_dlt vs H_theory 비교 (같은 대응쌍으로 직접 추정)
//     //    meas_rect (RECT 픽셀)과 obj XY(지면)로 findHomography
//     std::vector<cv::Point2f> XY2, UVr2;
//     XY2.reserve(obj3d.size()); UVr2.reserve(obj3d.size());
//     for (size_t i=0;i<obj3d.size();++i) {
//         XY2.emplace_back((float)obj3d[i].x, (float)obj3d[i].y);
//         UVr2.emplace_back((float)meas_rect[i].x, (float)meas_rect[i].y);
//     }
//     cv::Mat Hdlt = cv::findHomography(XY2, UVr2, 0);
//     Hdlt.convertTo(Hdlt, CV_64F);
//     Hdlt = normalizeH(Hdlt);

//     cv::Mat diff = cv::abs(Hdlt - Hth);
//     double maxAbsDiff;
//     cv::minMaxLoc(diff, nullptr, &maxAbsDiff);

//     std::cout << std::fixed << std::setprecision(3);
//     std::cout << "\n[RMSE] A RAW(projectPoints vs raw):       " << rmseA << " px\n";
//     std::cout << "[RMSE] B RECT(proj→undist vs meas→undist): " << rmseB << " px\n";
//     std::cout << "[RMSE] C RECT(H_theory vs meas→undist):    " << rmseC << " px\n";
//     std::cout << "[H] max|H_dlt - H_theory| (scaled):        " << maxAbsDiff << "\n";
// }

// // ===== [ADD] 레거시 H를 초기치로 써서 RefineLM 하는 예 (옵션) =====
// static void refineWithHinit(const std::vector<cv::Point3d>& obj3d,
//                             const std::vector<cv::Point2d>& img2d_raw,
//                             const cv::Mat& K, const cv::Mat& D,
//                             const cv::Mat& Hdlt,
//                             cv::Mat& R_w2c, cv::Mat& t_w2c)
// {
//     // 1) H로 초기치 얻기
//     extrinsicFromHomography(Hdlt, K, R_w2c, t_w2c);

//     // 2) RefineLM
//     cv::Mat rvec; cv::Rodrigues(R_w2c, rvec);
//     cv::Mat tvec = t_w2c.clone();
//     cv::solvePnPRefineLM(obj3d, img2d_raw, K, D, rvec, tvec);

//     cv::Rodrigues(rvec, R_w2c);
//     t_w2c = tvec.clone();
// }
