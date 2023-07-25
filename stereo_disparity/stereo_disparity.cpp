// ./stereo_disparity ../data/m001.jpg ../calibration.yml out.obj
#include <iostream>
#include <exception>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d.hpp>

#include "common_code.hpp"

const cv::String keys =
        "{help h usage ? |      | print this message.}"
        "{@input        |<none>| path of the image.}"
        "{@calibration        |<none>| filename of the calibration file.}"
        "{@output        |<none>| OBJ output file.}";


int
main(int argc, char *const *argv) {
    int retCode = EXIT_SUCCESS;

    try {
        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Stereo Calibration");
        if (parser.has("help")) {
            parser.printMessage();
            return EXIT_SUCCESS;
        }
        if (argc != 4) {
            std::cerr
                    << "Usage: stereo_disparity stereo_image.jpg calibration.yml out.obj"
                    << std::endl;
            return EXIT_FAILURE;
        }
        cv::Mat cam_mat_left, cam_mat_right, dist_coef_left, dist_coef_right, R, T, E, F;
        StereoParams st_parameters;
        OnMouseParams mouse_parameters_or;
        OnMouseParams mouse_parameters_rect;
        std::string wname_or = "Original";
        std::string wname_rect = "Rectification";
        float error;
        cv::Size camera_size = cv::Size(0, 0);
        std::string img_name = parser.get<cv::String>("@input");
        std::string calibration_file = parser.get<cv::String>("@calibration");
        std::string output_file = parser.get<cv::String>("@output");
        if (!IsPathExist(img_name)) {
            std::cerr << "Image <" << img_name << "> doesn't exist" << std::endl;
            return EXIT_FAILURE;
        }
        if (!IsPathExist(calibration_file)) {
            std::cerr << "Calibration file <" << calibration_file << "> doesn't exist" << std::endl;
            return EXIT_FAILURE;
        }
        auto fs = cv::FileStorage();
        fs.open(calibration_file, cv::FileStorage::READ);
        load_calibration_parameters(fs, camera_size, error, st_parameters.mtxL, st_parameters.mtxR, st_parameters.distL,
                                    st_parameters.distR, st_parameters.Rot, st_parameters.Trns, st_parameters.Emat,
                                    st_parameters.Fmat);
        cv::Mat img = cv::imread(img_name);
        cv::Mat img_left = img(cv::Range(0, img.rows), cv::Range(0, round(img.cols / 2)));
        cv::Mat img_right = img(cv::Range(0, img.rows), cv::Range(round(img.cols / 2), img.cols));
        rectifyStereoImages(st_parameters, img_left, img_right);
        cv::Mat img_rect;
        cv::hconcat(img_left, img_right, img_rect);
        cv::Mat new_left;
        cv::Mat new_right;
        cv::cvtColor(img_left, new_left, cv::COLOR_BGRA2GRAY);
        cv::cvtColor(img_right, new_right, cv::COLOR_BGRA2GRAY);
        cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create();
        cv::Mat disp, disparity;
        sbm->compute(new_left, new_right, disp);
        disp.convertTo(disparity, CV_32F, 1.0);
        disparity = disparity / 16.f;
        float B = sqrt((float) pow(st_parameters.Trns.at<double>(0, 0), 2) +
                       (float) pow(st_parameters.Trns.at<double>(1, 0), 2) +
                       (float) pow(st_parameters.Trns.at<double>(2, 0), 2));
        auto f = (float) st_parameters.mtxL.at<double>(0, 0);
        auto cx = (float) st_parameters.mtxL.at<double>(0, 2);
        auto cy = (float) st_parameters.mtxL.at<double>(1, 2);
        float X, Y, Z;
        std::vector<cv::Point3f> _3dpoints;
        for (auto x = 0; x < disparity.cols; x++) {
            for (auto y = 0; y < disparity.rows; y++) {
                if (disparity.at<float>(y, x) > 10.0) {
                    Z = B * f / disparity.at<float>(y, x);
                    X = (x - cx) * Z / f;
                    Y = (y - cy) * Z / f;
                    _3dpoints.emplace_back(X, Y, Z);
                }

            }
        }
        writeToOBJ(output_file, _3dpoints);
    }
    catch (std::exception &e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}