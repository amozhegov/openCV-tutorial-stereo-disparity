#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <sys/stat.h>
#include <fstream>
#include "common_code.hpp"


void rectifyStereoImages(const StereoParams &sti, cv::Mat &left, cv::Mat &rigth) {

    cv::Mat rect_l, rect_r, proj_mat_l, proj_mat_r, Q;
    cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
    cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;
    cv::stereoRectify(sti.mtxL, sti.distL, sti.mtxR, sti.distR, left.size(), sti.Rot, sti.Trns,
                      rect_l, rect_r, proj_mat_l, proj_mat_r,
                      Q, cv::CALIB_ZERO_DISPARITY, 0);
    cv::initUndistortRectifyMap(sti.mtxL, sti.distL, rect_l, proj_mat_l,
                                left.size(), CV_16SC2,
                                Left_Stereo_Map1, Left_Stereo_Map2);
    cv::initUndistortRectifyMap(sti.mtxR, sti.distR,
                                rect_r, proj_mat_r,
                                left.size(), CV_16SC2,
                                Right_Stereo_Map1, Right_Stereo_Map2);
    cv::Mat AuxImage, Right_nice;
    cv::remap(left, AuxImage, Left_Stereo_Map1, Left_Stereo_Map2,
              cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
    AuxImage.copyTo(left);
    cv::remap(rigth, AuxImage, Right_Stereo_Map1, Right_Stereo_Map2,
              cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
    AuxImage.copyTo(rigth);
}

void load_calibration_parameters(cv::FileStorage &fs,
                            cv::Size &camera_size,
                            float &error,
                            cv::Mat &camera_matrix_left,
                            cv::Mat &camera_matrix_right,
                            cv::Mat &dist_coeffs_left,
                            cv::Mat &dist_coeffs_right,
                            cv::Mat &rvec,
                            cv::Mat &tvec,
                            cv::Mat &E,
                            cv::Mat &F) {
    CV_Assert(fs.isOpened());
    //TODO
    fs["image-width"] >> camera_size.width;
    fs["image-height"] >> camera_size.height;
    fs["error"] >> error;
    fs["left-camera-matrix"] >> camera_matrix_left;
    fs["right-camera-matrix"] >> camera_matrix_right;
    fs["left-distorsion-coefficients"] >> dist_coeffs_left;
    fs["right-distorsion-coefficients"] >> dist_coeffs_right;
    fs["rvec"] >> rvec;
    fs["tvec"] >> tvec;
    fs["E"] >> E;
    fs["F"] >> F;

    //
    CV_Assert(fs.isOpened());
    CV_Assert(camera_matrix_left.type() == CV_64FC1 && camera_matrix_left.rows == 3 && camera_matrix_left.cols == 3);
    CV_Assert(camera_matrix_right.type() == CV_64FC1 && camera_matrix_right.rows == 3 && camera_matrix_right.cols == 3);
    CV_Assert(dist_coeffs_left.type() == CV_64FC1 && dist_coeffs_left.rows == 1 && dist_coeffs_left.cols == 5);
    CV_Assert(dist_coeffs_right.type() == CV_64FC1 && dist_coeffs_right.rows == 1 && dist_coeffs_right.cols == 5);

    return;
}

void writeToOBJ(const std::string &path, const std::vector<cv::Point3f> &points) {
    std::ofstream file(path, std::ios::binary);
    for (auto p: points)
        file << "v " << p.x << " " << p.y << " " << p.z << std::endl;
}


bool IsPathExist(const std::string &s) {
    struct stat buffer;
    return (stat(s.c_str(), &buffer) == 0);
}
