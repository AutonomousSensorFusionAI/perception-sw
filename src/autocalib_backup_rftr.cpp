#include "calibration.hpp"

// 전역 변수: color_bar를 HSV에서 BGR로 변환한 색상 막대 이미지
cv::Mat color_bar = cv::Mat(1, 13 * 3 * 3, CV_8UC3);
unsigned char *pBar = color_bar.data;

void Create_ColorBar()
{
    int H[13] = {180, 120, 60, 160, 100, 40, 150, 90, 30, 140, 80, 20, 10};
    int S[3]  = {255, 100, 30};
    int V[3]  = {255, 180, 90};

    // 1행에 13*3*3 크기의 HSV 컬러 배열 생성
    cv::Mat color = cv::Mat(1, 13 * 3 * 3, CV_8UC3);
    unsigned char *pColor = color.data;

    int h = 0, s = 0, v = 0;
    for (int ba = 0; ba < 13 * 3 * 3; ba++, h++, s++, v++)
    {
        if (h == 13)
            h = 0;
        if (s == 3 * 13)
            s = 0;
        if (v == 3 * 13 * 3)
            v = 0;

        pColor[ba * 3 + 0] = H[h];
        pColor[ba * 3 + 1] = S[s / 13];
        pColor[ba * 3 + 2] = V[v / (13 * 3)];
    }
    cv::cvtColor(color, color_bar, CV_HSV2BGR);
}

Calibrator::Calibrator(const std::string mask_dir,
                       const std::string lidar_file,
                       const std::string calib_file,
                       const std::string img_file,
                       const std::string error_file)
{
    // 캘리브레이션 파일 로드
    DataLoader::LoadCalibFile(calib_file, intrinsic_, extrinsic_, dist_);
    init_extrinsic_ = extrinsic_;

    // 이미지 로드
    img_file_ = img_file;
    cv::Mat img = cv::imread(img_file);
    IMG_H = img.rows;
    IMG_W = img.cols;
    masks_ = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC4);
    DataLoader::LoadMaskFile(mask_dir, intrinsic_, dist_, masks_, mask_point_num_);
    N_MASK = mask_point_num_.size();

    // 초기 외부 파라미터 에러 적용
    float var[6] = {0};
    std::ifstream file(error_file);
    if (!file.is_open())
    {
        std::cout << "open file " << error_file << " failed." << std::endl;
        exit(1);
    }
    file >> var[0] >> var[1] >> var[2] >> var[3] >> var[4] >> var[5];
    std::cout << "Initial error set to (r p y x y z):" << var[0] << " " << var[1] << " " << var[2]
              << " " << var[3] << " " << var[4] << " " << var[5] << std::endl;
    Eigen::Matrix4f deltaT = Util::GetDeltaT(var);
    extrinsic_ *= deltaT;

    // 포인트 클라우드 로드 및 전처리
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_origin(new pcl::PointCloud<pcl::PointXYZI>);
    DataLoader::LoadLidarFile(lidar_file, pc_origin);
    std::cout << "----------Start processing data----------" << std::endl;
    ProcessPointcloud(pc_origin);

    Create_ColorBar();
}

void Calibrator::ProcessPointcloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_origin)
{
    // 초기 외부 파라미터를 기준으로 포인트 필터링
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    int margin = 300;
    float intensity_max = 1;
    std::vector<Var6> vars;
    Util::GenVars(1, 5, 1, 0.5, vars);

    for (const auto &src_pt : pc_origin->points)
    {
        if (!std::isfinite(src_pt.x) || !std::isfinite(src_pt.y) || !std::isfinite(src_pt.z))
            continue;

        Eigen::Vector4f vec(src_pt.x, src_pt.y, src_pt.z, 1);
        int x, y;

        for (Var6 var : vars)
        {
            Eigen::Matrix4f extrinsic = extrinsic_ * Util::GetDeltaT(var.value);
            if (ProjectOnImage(vec, extrinsic, x, y, margin))
            {
                intensity_max = MAX(intensity_max, src_pt.intensity);
                pc_filtered->points.push_back(src_pt);
                break;
            }
        }
    }

    if (intensity_max > 1)
        intensity_max /= 2;

    // 포인트 클라우드 세그먼테이션
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    std::vector<pcl::PointIndices> seg_indices;
    Segment_pc(pc_filtered, normals, seg_indices);

    // 새 타입의 포인트 클라우드 구성
    pc_.reset(new pcl::PointCloud<PointXYZINS>);
    for (unsigned i = 0; i < pc_filtered->size(); i++)
    {
        PointXYZINS pt;
        pt.x         = (*pc_filtered)[i].x;
        pt.y         = (*pc_filtered)[i].y;
        pt.z         = (*pc_filtered)[i].z;
        pt.intensity = (*pc_filtered)[i].intensity / intensity_max;
        pt.curvature = (*normals)[i].curvature;
        curvature_max_ = MAX(curvature_max_, pt.curvature);
        pt.segment = -1;
        pc_->points.push_back(pt);
    }

    // 각 세그먼트에 인덱스 할당
    for (int i = 0; i < N_SEG; i++)
    {
        for (int index : seg_indices[i].indices)
        {
            (*pc_)[index].segment = i;
        }
    }
}

void Calibrator::Segment_pc(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                            pcl::PointCloud<pcl::Normal>::Ptr normals,
                            std::vector<pcl::PointIndices> &seg_indices)
{
    // 노멀 추정
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> norm_est;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(40);
    norm_est.setInputCloud(cloud);
    norm_est.compute(*normals);

    // 평면 분할 (Plane segmentation)
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr indices_plane(new pcl::PointIndices);
    pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.2);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(3000);
    seg.setDistanceThreshold(0.2);
    seg.setInputCloud(cloud);
    seg.setInputNormals(normals);
    seg.segment(*indices_plane, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZI> extract(true);
    extract.setInputCloud(cloud);
    pcl::PointIndices::Ptr indices_notplane(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    int plane_size = indices_plane->indices.size();
    pcl::PointIndices::Ptr indices_plane_all(new pcl::PointIndices);

    while (plane_size > 2000)
    {
        std::cout << "Plane points: " << plane_size << std::endl;
        seg_indices.push_back(*indices_plane);
        seg_point_num_.push_back(plane_size);

        // 평면 인덱스 누적
        indices_plane_all->indices.insert(indices_plane_all->indices.end(),
                                          indices_plane->indices.begin(),
                                          indices_plane->indices.end());
        extract.setIndices(indices_plane_all);
        extract.filter(*cloud_out);
        extract.getRemovedIndices(*indices_notplane);

        seg.setIndices(indices_notplane);
        seg.segment(*indices_plane, *coefficients);
        plane_size = indices_plane->indices.size();
    }
    std::cout << "Plane points < 1500, stop extracting plane." << std::endl;

    // 유클리드 클러스터 추출 (Euclidean cluster extraction)
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    std::vector<pcl::PointIndices> eu_cluster_indices;
    ec.setClusterTolerance(0.25);
    ec.setMaxClusterSize(10000);
    ec.setMinClusterSize(50);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.setIndices(indices_notplane);
    ec.extract(eu_cluster_indices);

    std::cout << "Euclidean cluster number: " << eu_cluster_indices.size() << std::endl;
    seg_indices.insert(seg_indices.end(), eu_cluster_indices.begin(), eu_cluster_indices.end());
    for (auto it = eu_cluster_indices.begin(); it != eu_cluster_indices.end(); it++)
    {
        seg_point_num_.push_back((*it).indices.size());
    }

    N_SEG = seg_indices.size();
    std::cout << "Extract " << N_SEG << " segments from point cloud." << std::endl;
}

void Calibrator::Calibrate()
{
    // 초기 및 에러 보정 결과를 이미지에 투영하여 저장
    VisualProjection(init_extrinsic_, img_file_, "init_proj.png");
    VisualProjectionSegment(init_extrinsic_, img_file_, "init_proj_seg.png");
    VisualProjection(extrinsic_, img_file_, "error_proj.png");
    VisualProjectionSegment(extrinsic_, img_file_, "error_proj_seg.png");

    std::cout << "----------Start calibration----------" << std::endl;
    if (!CalScore(extrinsic_, max_score_, true))
    {
        std::cout << "The initial extrinsic parameter error is too large." << std::endl;
        exit(1);
    }
    std::cout << "init_score: " << max_score_ << std::endl;

    // 단계별 탐색 수행
    BruteForceSearch(10, 0.5, 0, 0, true);   // [-5, 5] 범위
    BruteForceSearch(6, 0.15, 0, 0, true);     // [-0.9, 0.9] 범위
    RandomSearch(5000, 0.1, 0.5, true);         // [-0.5, 0.5] 범위

    std::cout << "---------------Result---------------" << std::endl;
    PrintCurrentError();
    VisualProjection(extrinsic_, img_file_, "refined_proj.png");
    VisualProjectionSegment(extrinsic_, img_file_, "refined_proj_seg.png");
}

bool Calibrator::CalScore(Eigen::Matrix4f T, float &score, bool is_coarse)
{
    std::vector<std::vector<float>> mask_normal(N_MASK);
    std::vector<std::vector<float>> mask_intensity(N_MASK);
    std::vector<std::unordered_map<int, int>> mask_segment(N_MASK);
    int min_mask_point_num = 900;

    // 각 포인트를 이미지에 투영하여 마스크별 데이터 수집
    for (const auto &src_pt : pc_->points)
    {
        Eigen::Vector4f vec(src_pt.x, src_pt.y, src_pt.z, 1);
        int x, y;
        if (ProjectOnImage(vec, T, x, y, 0))
        {
            cv::Vec4b mask_id = masks_.at<cv::Vec4b>(y, x);
            for (int c = 0; c < 4; c++)
            {
                if (mask_id[c] != 0)
                {
                    mask_normal[mask_id[c] - 1].push_back(src_pt.curvature);
                    mask_intensity[mask_id[c] - 1].push_back(src_pt.intensity);
                    if (src_pt.segment != -1)
                    {
                        mask_segment[mask_id[c] - 1][src_pt.segment]++;
                    }
                }
                else
                {
                    break;
                }
            }
        }
    }

    // consistency 계산을 위한 변수들
    std::vector<float> normal_sims, intensity_sims, segment_sims;
    std::vector<float> weight_normal, weight_intensity, weight_seg;
    float adjust = 0.0f;

    for (int i = 0; i < N_MASK; i++)
    {
        if (mask_point_num_[i] < min_mask_point_num)
            continue;
        int points_on_mask = mask_intensity[i].size();
        if (points_on_mask < 20)
            continue;
        adjust = 1 - 0.5 * pow(points_on_mask, -0.5);

        // Normal consistency
        float normal_sim = (1 - Util::Std(mask_normal[i]) / curvature_max_) * adjust;
        if (is_coarse)
            weight_normal.push_back(points_on_mask);
        normal_sims.push_back(normal_sim);

        // Intensity consistency
        float intensity_sim = (1 - Util::Std(mask_intensity[i])) * adjust;
        if (is_coarse)
            weight_intensity.push_back(points_on_mask);
        intensity_sims.push_back(intensity_sim);

        // Segment consistency
        if (!mask_segment[i].empty())
        {
            std::vector<int> seg_ratio;
            for (auto it = mask_segment[i].begin(); it != mask_segment[i].end(); it++)
                seg_ratio.push_back(it->second);

            sort(seg_ratio.begin(), seg_ratio.end(), std::greater<int>());
            if (seg_ratio[0] < 10)
                continue;

            float k = 1;
            int sum = 0;
            float segment_sim = 0;
            for (unsigned j = 0; j < seg_ratio.size(); j++)
            {
                segment_sim += seg_ratio[j] * k;
                k *= 0.4;
                sum += seg_ratio[j];
            }
            segment_sim = (segment_sim / sum) * adjust;
            segment_sims.push_back(segment_sim);
            if (is_coarse)
                weight_seg.push_back(sum);
        }
    }

    if (normal_sims.empty() || intensity_sims.empty() || segment_sims.empty())
    {
        std::cout << "Not enough points on masks. Skip this extrinsic." << std::endl;
        return false;
    }

    float normal_score   = Util::WeightMean(normal_sims, weight_normal);
    float intensity_score = Util::WeightMean(intensity_sims, weight_intensity);
    float segment_score  = Util::WeightMean(segment_sims, weight_seg);
    score = 0.3 * normal_score + 0.3 * intensity_score + 0.4 * segment_score;

    return true;
}

void Calibrator::BruteForceSearch(int rpy_range, float rpy_resolution, int xyz_range, float xyz_resolution, bool is_coarse)
{
    std::cout << "Start brute-force search around [-" << rpy_range * rpy_resolution << ", "
              << rpy_range * rpy_resolution << "] degree and [-" << xyz_range * xyz_resolution 
              << ", " << xyz_range * xyz_resolution << "] m" << std::endl;

    float best_var[6] = {0};
    float score;
    std::vector<Var6> vars;
    Util::GenVars(rpy_range, rpy_resolution, xyz_range, xyz_resolution, vars);

    for (auto var : vars)
    {
        CalScore(extrinsic_ * Util::GetDeltaT(var.value), score, is_coarse);
        if (score > max_score_)
        {
            max_score_ = score;
            for (size_t k = 0; k < 6; k++)
                best_var[k] = var.value[k];

            std::cout << "match score increase to: " << max_score_ << ", var: "
                      << best_var[0] << " " << best_var[1] << " " << best_var[2] << " "
                      << best_var[3] << " " << best_var[4] << " " << best_var[5] << std::endl;
        }
    }

    std::cout << "best var: " << best_var[0] << " " << best_var[1] << " " << best_var[2] << " "
              << best_var[3] << " " << best_var[4] << " " << best_var[5] << std::endl;
    Eigen::Matrix4f deltaT = Util::GetDeltaT(best_var);
    extrinsic_ *= deltaT;
}

void Calibrator::RandomSearch(int search_count, float xyz_range, float rpy_range, bool is_coarse)
{
    std::cout << "Start random search around [-" << rpy_range << ", " << rpy_range
              << "] degree and [-" << xyz_range << ", " << xyz_range << "] m" << std::endl;

    float var[6] = {0};
    float bestVal[6] = {0};

    std::default_random_engine generator((clock() - time(0)) / (double)CLOCKS_PER_SEC);
    std::uniform_real_distribution<double> distribution_xyz(-xyz_range, xyz_range);
    std::uniform_real_distribution<double> distribution_rpy(-rpy_range, rpy_range);

    for (int i = 0; i < search_count; i++)
    {
        var[0] = distribution_rpy(generator);
        var[1] = distribution_rpy(generator);
        var[2] = distribution_rpy(generator);
        var[3] = distribution_xyz(generator);
        var[4] = distribution_xyz(generator);
        var[5] = distribution_xyz(generator);

        Eigen::Matrix4f deltaT = Util::GetDeltaT(var);
        float score;
        if (!CalScore(extrinsic_ * deltaT, score, is_coarse))
            continue;

        if (score > max_score_)
        {
            max_score_ = score;
            for (size_t k = 0; k < 6; k++)
                bestVal[k] = var[k];

            std::cout << "match score increase to: " << max_score_ << ", val: "
                      << bestVal[0] << " " << bestVal[1] << " " << bestVal[2] << " "
                      << bestVal[3] << " " << bestVal[4] << " " << bestVal[5] << std::endl;
        }
    }

    std::cout << "best val: " << bestVal[0] << " " << bestVal[1] << " " << bestVal[2] << " "
              << bestVal[3] << " " << bestVal[4] << " " << bestVal[5] << std::endl;
    extrinsic_ *= Util::GetDeltaT(bestVal);
}

void Calibrator::PrintCurrentError()
{
    Eigen::Matrix4f error_T = extrinsic_ * init_extrinsic_.inverse();
    std::cout << "Error: " 
              << RAD2DEG(Util::GetRoll(error_T)) << " "
              << RAD2DEG(Util::GetPitch(error_T)) << " "
              << RAD2DEG(Util::GetYaw(error_T)) << " "
              << Util::GetX(error_T) << " "
              << Util::GetY(error_T) << " "
              << Util::GetZ(error_T) << std::endl;
}

Eigen::Matrix4f Calibrator::GetFinalTransformation()
{
    return extrinsic_;
}

void Calibrator::VisualProjection(Eigen::Matrix4f T, std::string img_file, std::string save_name)
{
    cv::Mat img_color = cv::imread(img_file);
    if (intrinsic_.cols() == 3)
        Util::UndistImg(img_color, intrinsic_, dist_);

    std::vector<cv::Point2f> lidar_points;
    for (const auto &src_pt : pc_->points)
    {
        Eigen::Vector4f vec(src_pt.x, src_pt.y, src_pt.z, 1);
        int x, y;
        if (ProjectOnImage(vec, T, x, y, 0))
        {
            if (src_pt.intensity > 0.4)
                lidar_points.push_back(cv::Point2f(x, y));
        }
    }

    for (const cv::Point &point : lidar_points)
        cv::circle(img_color, point, 3, cv::Scalar(0, 255, 0), -1, 0);

    cv::imwrite(save_name, img_color);
    std::cout << "Image saved: " << save_name << std::endl;
}

void Calibrator::VisualProjectionSegment(Eigen::Matrix4f T, std::string img_file, std::string save_name)
{
    cv::Mat img_color = cv::imread(img_file);
    if (intrinsic_.cols() == 3)
        Util::UndistImg(img_color, intrinsic_, dist_);

    std::vector<std::vector<cv::Point2f>> lidar_points(N_SEG);
    for (const auto &src_pt : pc_->points)
    {
        Eigen::Vector4f vec(src_pt.x, src_pt.y, src_pt.z, 1);
        int x, y;
        if (ProjectOnImage(vec, T, x, y, 0))
        {
            int seg = src_pt.segment;
            if (seg != -1)
                lidar_points[seg].push_back(cv::Point2f(x, y));
        }
    }

    for (int i = 1; i < N_SEG; i++)
    {
        cv::Vec3b color = color_bar.at<cv::Vec3b>(0, i);
        for (const cv::Point &point : lidar_points[i])
        {
            cv::circle(img_color, point, 3, cv::Scalar(color[0], color[1], color[2]), -1, 0);
        }
    }

    cv::imwrite(save_name, img_color);
    std::cout << "Image saved: " << save_name << std::endl;
}

bool Calibrator::ProjectOnImage(const Eigen::Vector4f &vec, const Eigen::Matrix4f &T, int &x, int &y, int margin)
{
    Eigen::Vector3f vec2;
    if (intrinsic_.cols() == 4)
    {
        vec2 = intrinsic_ * T * vec;
    }
    else
    {
        Eigen::Vector4f cam_point = T * vec;
        Eigen::Vector3f cam_vec(cam_point(0), cam_point(1), cam_point(2));
        vec2 = intrinsic_ * cam_vec;
    }

    if (vec2(2) <= 0)
        return false;

    x = static_cast<int>(cvRound(vec2(0) / vec2(2)));
    y = static_cast<int>(cvRound(vec2(1) / vec2(2)));

    if (x >= -margin && x < IMG_W + margin &&
        y >= -margin && y < IMG_H + margin)
        return true;

    return false;
}
