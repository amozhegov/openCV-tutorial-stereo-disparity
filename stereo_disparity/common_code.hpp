#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

//Structure that contains the Stereo Pair Calbration information.
//This will be calculated using stereo_calibrate
struct StereoParams {
    cv::Mat mtxL, distL, R_L, T_L;
    cv::Mat mtxR, distR, R_R, T_R;
    cv::Mat Rot, Trns, Emat, Fmat;
};

struct OnMouseParams {
    cv::Mat img;
    std::string wname;
};

void rectifyStereoImages(const StereoParams &sti, cv::Mat &left, cv::Mat &rigth);

bool IsPathExist(const std::string &s);

void
load_calibration_parameters(cv::FileStorage &fs,
                            cv::Size &camera_size,
                            float &error,
                            cv::Mat &camera_matrix_left,
                            cv::Mat &camera_matrix_right,
                            cv::Mat &dist_coeffs_left,
                            cv::Mat &dist_coeffs_right,
                            cv::Mat &rvec,
                            cv::Mat &tvec,
                            cv::Mat &E,
                            cv::Mat &F);

void writeToPCD(const std::string &path, std::vector<cv::Point3f> points);

void writeToOBJ(const std::string &path, const std::vector<cv::Point3f> &points);

