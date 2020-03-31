#include "openvslam/camera/perspective.h"

#include <iostream>

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>


#if CV_MAJOR_VERSION == 3 and CV_MINOR_VERSION<=2
/*
 * We backport undistortPoints() from OpenCV 3.3.1
 */
#include <opencv2/imgproc/detail/distortion_model.hpp>

namespace cv {

static void cvUndistortPointsInternal( const CvMat* _src, CvMat* _dst, const CvMat* _cameraMatrix,
                   const CvMat* _distCoeffs,
                   const CvMat* matR, const CvMat* matP, cv::TermCriteria criteria)
{
    double A[3][3], RR[3][3], k[14]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    CvMat matA=cvMat(3, 3, CV_64F, A), _Dk;
    CvMat _RR=cvMat(3, 3, CV_64F, RR);
    cv::Matx33d invMatTilt = cv::Matx33d::eye();
    cv::Matx33d matTilt = cv::Matx33d::eye();

    CV_Assert( CV_IS_MAT(_src) && CV_IS_MAT(_dst) &&
        (_src->rows == 1 || _src->cols == 1) &&
        (_dst->rows == 1 || _dst->cols == 1) &&
        _src->cols + _src->rows - 1 == _dst->rows + _dst->cols - 1 &&
        (CV_MAT_TYPE(_src->type) == CV_32FC2 || CV_MAT_TYPE(_src->type) == CV_64FC2) &&
        (CV_MAT_TYPE(_dst->type) == CV_32FC2 || CV_MAT_TYPE(_dst->type) == CV_64FC2));

    CV_Assert( CV_IS_MAT(_cameraMatrix) &&
        _cameraMatrix->rows == 3 && _cameraMatrix->cols == 3 );

    cvConvert( _cameraMatrix, &matA );


    if( _distCoeffs )
    {
        CV_Assert( CV_IS_MAT(_distCoeffs) &&
            (_distCoeffs->rows == 1 || _distCoeffs->cols == 1) &&
            (_distCoeffs->rows*_distCoeffs->cols == 4 ||
             _distCoeffs->rows*_distCoeffs->cols == 5 ||
             _distCoeffs->rows*_distCoeffs->cols == 8 ||
             _distCoeffs->rows*_distCoeffs->cols == 12 ||
             _distCoeffs->rows*_distCoeffs->cols == 14));

        _Dk = cvMat( _distCoeffs->rows, _distCoeffs->cols,
            CV_MAKETYPE(CV_64F,CV_MAT_CN(_distCoeffs->type)), k);

        cvConvert( _distCoeffs, &_Dk );
        if (k[12] != 0 || k[13] != 0)
        {
            cv::detail::computeTiltProjectionMatrix<double>(k[12], k[13], NULL, NULL, NULL, &invMatTilt);
            cv::detail::computeTiltProjectionMatrix<double>(k[12], k[13], &matTilt, NULL, NULL);
        }
    }

    if( matR )
    {
        CV_Assert( CV_IS_MAT(matR) && matR->rows == 3 && matR->cols == 3 );
        cvConvert( matR, &_RR );
    }
    else
        cvSetIdentity(&_RR);

    if( matP )
    {
        double PP[3][3];
        CvMat _P3x3, _PP=cvMat(3, 3, CV_64F, PP);
        CV_Assert( CV_IS_MAT(matP) && matP->rows == 3 && (matP->cols == 3 || matP->cols == 4));
        cvConvert( cvGetCols(matP, &_P3x3, 0, 3), &_PP );
        cvMatMul( &_PP, &_RR, &_RR );
    }

    const CvPoint2D32f* srcf = (const CvPoint2D32f*)_src->data.ptr;
    const CvPoint2D64f* srcd = (const CvPoint2D64f*)_src->data.ptr;
    CvPoint2D32f* dstf = (CvPoint2D32f*)_dst->data.ptr;
    CvPoint2D64f* dstd = (CvPoint2D64f*)_dst->data.ptr;
    int stype = CV_MAT_TYPE(_src->type);
    int dtype = CV_MAT_TYPE(_dst->type);
    int sstep = _src->rows == 1 ? 1 : _src->step/CV_ELEM_SIZE(stype);
    int dstep = _dst->rows == 1 ? 1 : _dst->step/CV_ELEM_SIZE(dtype);

    double fx = A[0][0];
    double fy = A[1][1];
    double ifx = 1./fx;
    double ify = 1./fy;
    double cx = A[0][2];
    double cy = A[1][2];

    int n = _src->rows + _src->cols - 1;
    for( int i = 0; i < n; i++ )
    {
        double x, y, x0 = 0, y0 = 0, u, v;
        if( stype == CV_32FC2 )
        {
            x = srcf[i*sstep].x;
            y = srcf[i*sstep].y;
        }
        else
        {
            x = srcd[i*sstep].x;
            y = srcd[i*sstep].y;
        }
        u = x; v = y;
        x = (x - cx)*ifx;
        y = (y - cy)*ify;

        if( _distCoeffs ) {
            // compensate tilt distortion
            cv::Vec3d vecUntilt = invMatTilt * cv::Vec3d(x, y, 1);
            double invProj = vecUntilt(2) ? 1./vecUntilt(2) : 1;
            x0 = x = invProj * vecUntilt(0);
            y0 = y = invProj * vecUntilt(1);

            double error = std::numeric_limits<double>::max();
            // compensate distortion iteratively

            for( int j = 0; ; j++ )
            {
                if ((criteria.type & cv::TermCriteria::COUNT) && j >= criteria.maxCount)
                    break;
                if ((criteria.type & cv::TermCriteria::EPS) && error < criteria.epsilon)
                    break;
                double r2 = x*x + y*y;
                double icdist = (1 + ((k[7]*r2 + k[6])*r2 + k[5])*r2)/(1 + ((k[4]*r2 + k[1])*r2 + k[0])*r2);
                double deltaX = 2*k[2]*x*y + k[3]*(r2 + 2*x*x)+ k[8]*r2+k[9]*r2*r2;
                double deltaY = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y+ k[10]*r2+k[11]*r2*r2;
                x = (x0 - deltaX)*icdist;
                y = (y0 - deltaY)*icdist;

                if(criteria.type & cv::TermCriteria::EPS)
                {
                    double r4, r6, a1, a2, a3, cdist, icdist2;
                    double xd, yd, xd0, yd0;
                    cv::Vec3d vecTilt;

                    r2 = x*x + y*y;
                    r4 = r2*r2;
                    r6 = r4*r2;
                    a1 = 2*x*y;
                    a2 = r2 + 2*x*x;
                    a3 = r2 + 2*y*y;
                    cdist = 1 + k[0]*r2 + k[1]*r4 + k[4]*r6;
                    icdist2 = 1./(1 + k[5]*r2 + k[6]*r4 + k[7]*r6);
                    xd0 = x*cdist*icdist2 + k[2]*a1 + k[3]*a2 + k[8]*r2+k[9]*r4;
                    yd0 = y*cdist*icdist2 + k[2]*a3 + k[3]*a1 + k[10]*r2+k[11]*r4;

                    vecTilt = matTilt*cv::Vec3d(xd0, yd0, 1);
                    invProj = vecTilt(2) ? 1./vecTilt(2) : 1;
                    xd = invProj * vecTilt(0);
                    yd = invProj * vecTilt(1);

                    double x_proj = xd*fx + cx;
                    double y_proj = yd*fy + cy;

                    error = sqrt( pow(x_proj - u, 2) + pow(y_proj - v, 2) );
                }
            }
        }

        double xx = RR[0][0]*x + RR[0][1]*y + RR[0][2];
        double yy = RR[1][0]*x + RR[1][1]*y + RR[1][2];
        double ww = 1./(RR[2][0]*x + RR[2][1]*y + RR[2][2]);
        x = xx*ww;
        y = yy*ww;

        if( dtype == CV_32FC2 )
        {
            dstf[i*dstep].x = (float)x;
            dstf[i*dstep].y = (float)y;
        }
        else
        {
            dstd[i*dstep].x = x;
            dstd[i*dstep].y = y;
        }
    }
}


void undistortPoints( InputArray _src, OutputArray _dst,
        InputArray _cameraMatrix,
        InputArray _distCoeffs,
        InputArray _Rmat,
        InputArray _Pmat,
        TermCriteria criteria)
{
    Mat src = _src.getMat(), cameraMatrix = _cameraMatrix.getMat();
    Mat distCoeffs = _distCoeffs.getMat(), R = _Rmat.getMat(), P = _Pmat.getMat();

    CV_Assert( src.isContinuous() && (src.depth() == CV_32F || src.depth() == CV_64F) &&
              ((src.rows == 1 && src.channels() == 2) || src.cols*src.channels() == 2));

    _dst.create(src.size(), src.type(), -1, true);
    Mat dst = _dst.getMat();

    CvMat _csrc = src, _cdst = dst, _ccameraMatrix = cameraMatrix;
    CvMat matR, matP, _cdistCoeffs, *pR=0, *pP=0, *pD=0;
    if( !R.empty() )
        pR = &(matR = R);
    if( !P.empty() )
        pP = &(matP = P);
    if( !distCoeffs.empty() )
        pD = &(_cdistCoeffs = distCoeffs);
    cvUndistortPointsInternal(&_csrc, &_cdst, &_ccameraMatrix, pD, pR, pP, criteria);
}
}	// namespace cv
#endif


namespace openvslam {
namespace camera {

perspective::perspective(const std::string& name, const setup_type_t& setup_type, const color_order_t& color_order,
                         const unsigned int cols, const unsigned int rows, const double fps,
                         const double fx, const double fy, const double cx, const double cy,
                         const double k1, const double k2, const double p1, const double p2, const double k3,
                         const double focal_x_baseline)
    : base(name, setup_type, model_type_t::Perspective, color_order, cols, rows, fps, focal_x_baseline, focal_x_baseline / fx),
      fx_(fx), fy_(fy), cx_(cx), cy_(cy), fx_inv_(1.0 / fx), fy_inv_(1.0 / fy),
      k1_(k1), k2_(k2), p1_(p1), p2_(p2), k3_(k3) {
    spdlog::debug("CONSTRUCT: camera::perspective");

    cv_cam_matrix_ = (cv::Mat_<float>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
    cv_dist_params_ = (cv::Mat_<float>(5, 1) << k1_, k2_, p1_, p2_, k3_);

    eigen_cam_matrix_ << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
    eigen_dist_params_ << k1_, k2_, p1_, p2_, k3_;

    img_bounds_ = compute_image_bounds();

    inv_cell_width_ = static_cast<double>(num_grid_cols_) / (img_bounds_.max_x_ - img_bounds_.min_x_);
    inv_cell_height_ = static_cast<double>(num_grid_rows_) / (img_bounds_.max_y_ - img_bounds_.min_y_);
}

perspective::perspective(const YAML::Node& yaml_node)
    : perspective(yaml_node["Camera.name"].as<std::string>(),
                  load_setup_type(yaml_node),
                  load_color_order(yaml_node),
                  yaml_node["Camera.cols"].as<unsigned int>(),
                  yaml_node["Camera.rows"].as<unsigned int>(),
                  yaml_node["Camera.fps"].as<double>(),
                  yaml_node["Camera.fx"].as<double>(),
                  yaml_node["Camera.fy"].as<double>(),
                  yaml_node["Camera.cx"].as<double>(),
                  yaml_node["Camera.cy"].as<double>(),
                  yaml_node["Camera.k1"].as<double>(),
                  yaml_node["Camera.k2"].as<double>(),
                  yaml_node["Camera.p1"].as<double>(),
                  yaml_node["Camera.p2"].as<double>(),
                  yaml_node["Camera.k3"].as<double>(),
                  yaml_node["Camera.focal_x_baseline"].as<double>(0.0)) {}

perspective::~perspective() {
    spdlog::debug("DESTRUCT: camera::perspective");
}

void perspective::show_parameters() const {
    show_common_parameters();
    std::cout << "  - fx: " << fx_ << std::endl;
    std::cout << "  - fy: " << fy_ << std::endl;
    std::cout << "  - cx: " << cx_ << std::endl;
    std::cout << "  - cy: " << cy_ << std::endl;
    std::cout << "  - k1: " << k1_ << std::endl;
    std::cout << "  - k2: " << k2_ << std::endl;
    std::cout << "  - p1: " << p1_ << std::endl;
    std::cout << "  - p2: " << p2_ << std::endl;
    std::cout << "  - k3: " << k3_ << std::endl;
    std::cout << "  - min x: " << img_bounds_.min_x_ << std::endl;
    std::cout << "  - max x: " << img_bounds_.max_x_ << std::endl;
    std::cout << "  - min y: " << img_bounds_.min_y_ << std::endl;
    std::cout << "  - max y: " << img_bounds_.max_y_ << std::endl;
}

image_bounds perspective::compute_image_bounds() const {
    spdlog::debug("compute image bounds");

    if (k1_ == 0 && k2_ == 0 && p1_ == 0 && p2_ == 0 && k3_ == 0) {
        // any distortion does not exist

        return image_bounds{0.0, cols_, 0.0, rows_};
    }
    else {
        // distortion exists

        // corner coordinates: (x, y) = (col, row)
        const std::vector<cv::KeyPoint> corners{cv::KeyPoint(0.0, 0.0, 1.0),      // left top
                                                cv::KeyPoint(cols_, 0.0, 1.0),    // right top
                                                cv::KeyPoint(0.0, rows_, 1.0),    // left bottom
                                                cv::KeyPoint(cols_, rows_, 1.0)}; // right bottom

        std::vector<cv::KeyPoint> undist_corners;
        undistort_keypoints(corners, undist_corners);

        return image_bounds{std::min(undist_corners.at(0).pt.x, undist_corners.at(2).pt.x),
                            std::max(undist_corners.at(1).pt.x, undist_corners.at(3).pt.x),
                            std::min(undist_corners.at(0).pt.y, undist_corners.at(1).pt.y),
                            std::max(undist_corners.at(2).pt.y, undist_corners.at(3).pt.y)};
    }
}

void perspective::undistort_keypoints(const std::vector<cv::KeyPoint>& dist_keypts, std::vector<cv::KeyPoint>& undist_keypts) const {
    // cv::undistortPoints does not accept an empty input
    if (dist_keypts.empty()) {
        undist_keypts.clear();
        return;
    }

    // fill cv::Mat with distorted keypoints
    cv::Mat mat(dist_keypts.size(), 2, CV_32F);
    for (unsigned long idx = 0; idx < dist_keypts.size(); ++idx) {
        mat.at<float>(idx, 0) = dist_keypts.at(idx).pt.x;
        mat.at<float>(idx, 1) = dist_keypts.at(idx).pt.y;
    }

    // undistort
    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, cv_cam_matrix_, cv_dist_params_, cv::Mat(), cv_cam_matrix_,
                        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 20, 1e-6));
    mat = mat.reshape(1);

    // convert to cv::Mat
    undist_keypts.resize(dist_keypts.size());
    for (unsigned long idx = 0; idx < undist_keypts.size(); ++idx) {
        undist_keypts.at(idx).pt.x = mat.at<float>(idx, 0);
        undist_keypts.at(idx).pt.y = mat.at<float>(idx, 1);
        undist_keypts.at(idx).angle = dist_keypts.at(idx).angle;
        undist_keypts.at(idx).size = dist_keypts.at(idx).size;
        undist_keypts.at(idx).octave = dist_keypts.at(idx).octave;
    }
}

void perspective::convert_keypoints_to_bearings(const std::vector<cv::KeyPoint>& undist_keypts, eigen_alloc_vector<Vec3_t>& bearings) const {
    bearings.resize(undist_keypts.size());
    for (unsigned long idx = 0; idx < undist_keypts.size(); ++idx) {
        const auto x_normalized = (undist_keypts.at(idx).pt.x - cx_) / fx_;
        const auto y_normalized = (undist_keypts.at(idx).pt.y - cy_) / fy_;
        const auto l2_norm = std::sqrt(x_normalized * x_normalized + y_normalized * y_normalized + 1.0);
        bearings.at(idx) = Vec3_t{x_normalized / l2_norm, y_normalized / l2_norm, 1.0 / l2_norm};
    }
}

void perspective::convert_bearings_to_keypoints(const eigen_alloc_vector<Vec3_t>& bearings, std::vector<cv::KeyPoint>& undist_keypts) const {
    undist_keypts.resize(bearings.size());
    for (unsigned long idx = 0; idx < bearings.size(); ++idx) {
        const auto x_normalized = bearings.at(idx)(0) / bearings.at(idx)(2);
        const auto y_normalized = bearings.at(idx)(1) / bearings.at(idx)(2);

        undist_keypts.at(idx).pt.x = fx_ * x_normalized + cx_;
        undist_keypts.at(idx).pt.y = fy_ * y_normalized + cy_;
    }
}

bool perspective::reproject_to_image(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Vec3_t& pos_w, Vec2_t& reproj, float& x_right) const {
    // convert to camera-coordinates
    const Vec3_t pos_c = rot_cw * pos_w + trans_cw;

    // check if the point is visible
    if (pos_c(2) <= 0.0) {
        return false;
    }

    // reproject onto the image
    const auto z_inv = 1.0 / pos_c(2);
    reproj(0) = fx_ * pos_c(0) * z_inv + cx_;
    reproj(1) = fy_ * pos_c(1) * z_inv + cy_;
    x_right = reproj(0) - focal_x_baseline_ * z_inv;

    // check if the point is visible
    return (img_bounds_.min_x_ < reproj(0) && reproj(0) < img_bounds_.max_x_
            && img_bounds_.min_y_ < reproj(1) && reproj(1) < img_bounds_.max_y_);
}

bool perspective::reproject_to_bearing(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Vec3_t& pos_w, Vec3_t& reproj) const {
    // convert to camera-coordinates
    reproj = rot_cw * pos_w + trans_cw;

    // check if the point is visible
    if (reproj(2) <= 0.0) {
        return false;
    }

    // reproject onto the image
    const auto z_inv = 1.0 / reproj(2);
    const auto x = fx_ * reproj(0) * z_inv + cx_;
    const auto y = fy_ * reproj(1) * z_inv + cy_;

    // convert to a bearing
    reproj.normalize();

    // check if the point is visible
    return (img_bounds_.min_x_ < x && x < img_bounds_.max_x_
            && img_bounds_.min_y_ < y && y < img_bounds_.max_y_);
}

nlohmann::json perspective::to_json() const {
    return {{"model_type", get_model_type_string()},
            {"setup_type", get_setup_type_string()},
            {"color_order", get_color_order_string()},
            {"cols", cols_},
            {"rows", rows_},
            {"fps", fps_},
            {"focal_x_baseline", focal_x_baseline_},
            {"num_grid_cols", num_grid_cols_},
            {"num_grid_rows", num_grid_rows_},
            {"fx", fx_},
            {"fy", fy_},
            {"cx", cx_},
            {"cy", cy_},
            {"k1", k1_},
            {"k2", k2_},
            {"p1", p1_},
            {"p2", p2_},
            {"k3", k3_}};
}

} // namespace camera
} // namespace openvslam
