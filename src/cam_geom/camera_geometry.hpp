#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <iomanip> 

struct CamCalib
{
    cv::Mat K; //3x3, CV_64F
    cv::Mat D; //1x5, CV_64F
    cv::Mat R_w2c; //3x3, CV_64F w2c from solvePnP
    cv::Mat t_w2c; //3x1, CV_64F w2c

    cv::Mat R_c2w; //3x3
    // cv::Mat t_c2w; //3x1

    cv::Mat Cw;    //camera center in world
    cv::Mat H_w2i; //homography world(ground) to image 
    cv::Mat H_i2w; //homography image to world(ground)

    /*  */

    static CamCalib Make(const cv::Mat& K_,    const cv::Mat& D_,
                         const cv::Mat& R_w2c, const cv::Mat& t_w2c )
    {
        CamCalib c;
        c.K = K_.clone(); 
        c.D = D_.clone();
        c.R_w2c = R_w2c.clone(); 
        c.t_w2c = t_w2c.clone();

        c.R_c2w = c.R_w2c.t();
        c.Cw    = -c.R_w2c.t() * c.t_w2c;

        cv::Mat M(3,3,CV_64F);           // z=0 평면 상의 점에만 적용하는 호모그래피 
        c.R_w2c.col(0).copyTo(M.col(0)); // r1
        c.R_w2c.col(1).copyTo(M.col(1)); // r2
        c.t_w2c.copyTo(M.col(2));        // t

        c.H_w2i = c.K * M;
        c.H_i2w = c.H_w2i.inv();

        return c;
    }
};

inline cv::Point2d gndToPix(const cv::Point2d& XY, const CamCalib& c)
{
    cv::Mat Xw = (cv::Mat_<double>(3,1) << XY.x, XY.y, 1.0);

    cv::Mat ph = c.H_w2i * Xw;

    double w = ph.at<double>(2);

    if (std::abs(w) < 1e-12) throw std::runtime_error("gndToPix: w=0");

    // 여기서 다시 distort를 해야 원래 픽셀이랑 일치함
    // 애초에 카메라 드라이버 쪽에서 undistortion을 해서 /image_raw를 쏘면?
    return {ph.at<double>(0)/w, ph.at<double>(1)/w}; // 왜곡보정 안되어있음
}

inline cv::Point2d pixToGndRaw(const cv::Point2d& uv, const CamCalib& c) // 원본 이미지 픽셀 -> 지면 좌표
{
    std::vector<cv::Point2d> src{uv}, dst;
    // cv::undistortPoints(src, dst, c.K, c.D);
    cv::undistortPoints(src, dst, c.K, c.D, cv::noArray(), c.K);
    // const double xn = dst[0].x, yn = dst[0].y;
    const cv::Point2d undistorted_uv = dst[0];

    cv::Mat p_img = (cv::Mat_<double>(3,1) << undistorted_uv.x, undistorted_uv.y, 1.0);
    cv::Mat p_gnd = c.H_i2w * p_img;

    double w = p_gnd.at<double>(2);
    if (std::abs(w) < 1e-9) throw std::runtime_error("pixToGndRaw: w is close to zero.");

    return {p_gnd.at<double>(0)/w, p_gnd.at<double>(1)/w};
}

inline cv::Point2d pixToGndUndist(const cv::Point2d& uv, const CamCalib& c) // 왜곡 보정 이미지 픽셀 -> 지면 좌표
{
    // 왜곡보정된 픽셀 좌표 그대로 homography 적용
    cv::Mat p_img = (cv::Mat_<double>(3,1) << uv.x, uv.y, 1.0);
    cv::Mat p_gnd = c.H_i2w * p_img; 

    double w = p_gnd.at<double>(2);
    if (std::abs(w) < 1e-9) throw std::runtime_error("pixToGndUndist: w is close to zero.");

    return {p_gnd.at<double>(0)/w, p_gnd.at<double>(1)/w};
}

// inline cv::Point2d pixToGndRay(const cv::Point2d& uv_raw, const CamCalib& c) {
//     // RAW 픽셀 받는 버전
//     std::vector<cv::Point2d> src{uv_raw}, und;
//     cv::undistortPoints(src, und, c.K, c.D);        // -> 정규화 (x_n,y_n)
//     cv::Vec3d d_cam(und[0].x, und[0].y, 1.0);

//     cv::Mat Rcw = c.R_w2c.t();
//     cv::Mat Cw  = -Rcw * c.t_w2c;
//     cv::Vec3d d_w = cv::Vec3d(Rcw * cv::Mat(d_cam));

//     double dz = d_w[2]; if (std::abs(dz) < 1e-12) throw std::runtime_error("near-horizon");
//     double s  = -Cw.at<double>(2) / dz; if (s <= 0) throw std::runtime_error("behind-camera");
//     cv::Vec3d Pw = cv::Vec3d(Cw) + s * d_w;
//     return {Pw[0], Pw[1]};
// }

class getCameraParameters
{

private:


public:


};

static cv::Mat normalizeH(const cv::Mat& H);
static cv::Mat makeH_theory(const cv::Mat& K, const cv::Mat& R_w2c, const cv::Mat& t_w2c);
static void extrinsicFromHomography(const cv::Mat& H_w2i, const cv::Mat& K,
                                    cv::Mat& R_w2c, cv::Mat& t_w2c);
static void diagnoseRMSE(const std::vector<cv::Point3d>& obj3d,
                         const std::vector<cv::Point2d>& img2d_raw,
                         const cv::Mat& K, const cv::Mat& D,
                         const cv::Mat& R_w2c, const cv::Mat& t_w2c);
static void refineWithHinit(const std::vector<cv::Point3d>& obj3d,
                            const std::vector<cv::Point2d>& img2d_raw,
                            const cv::Mat& K, const cv::Mat& D,
                            const cv::Mat& Hdlt,
                            cv::Mat& R_w2c, cv::Mat& t_w2c);
