//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name) // The convention is upper-left for image and lower-left for texture: (0, 0) to (width, height)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)  // uv lower left == image upper left so (1 - v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img); // must be of type int
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    // Based on https://sites.cs.ucsb.edu/~lingqi/teaching/resources/GAMES101_Lecture_09.pdf
    Eigen::Vector3f getColorBilinear(float u, float v) {

        float u_img = u * width;
        float v_img = (1 - v) * height;

        // 1. Find the pixel coordinates containing the (u_img, v_img)
        int x1 = std::floor(u_img);
        int x2 = std::min(x1 + 1, width - 1);
        int y1 = std::floor(v_img);
        int y2 = std::min(y1 + 1, height - 1);

        // 2. Use the four pixel coordinates(Not the 4 sample locations) to perform bilinear interpolations. Note: Draw out first
        Eigen::Vector3f u00 = Eigen::Vector3f(image_data.at<cv::Vec3b>(y1, x1)[0],
            image_data.at<cv::Vec3b>(y1, x1)[1],
            image_data.at<cv::Vec3b>(y1, x1)[2]) / 255.0f;

        Eigen::Vector3f u01 = Eigen::Vector3f(image_data.at<cv::Vec3b>(y2, x1)[0],
            image_data.at<cv::Vec3b>(y2, x1)[1],
            image_data.at<cv::Vec3b>(y2, x1)[2]) / 255.0f;

        Eigen::Vector3f u10 = Eigen::Vector3f(image_data.at<cv::Vec3b>(y1, x2)[0],
            image_data.at<cv::Vec3b>(y1, x2)[1],
            image_data.at<cv::Vec3b>(y1, x2)[2]) / 255.0f;

        Eigen::Vector3f u11 = Eigen::Vector3f(image_data.at<cv::Vec3b>(y2, x2)[0],
            image_data.at<cv::Vec3b>(y2, x2)[1],
            image_data.at<cv::Vec3b>(y2, x2)[2]) / 255.0f;

        // 3. Distance from (u_img, v_img) to the lower-left pixel horizontally and vertically, s & t naturally within [0, 1]
        float s = u_img - x1;
        float t = v_img - y1;
        
        // 4. 2 Horizontal interpolations. Note: Direct interpolation on integer values can lead to overflow, hence we work on float and then convert back
        Eigen::Vector3f u0 = u00 + s * (u10 - u00);  
        Eigen::Vector3f u1 = u01 + s * (u11 - u01);

        // 5. 1 Vertical interpolation
        Eigen::Vector3f color = u0 + t * (u1 - u0);
        return Eigen::Vector3f(color[0], color[1], color[2]) * 255.0f;
    }
};
#endif //RASTERIZER_TEXTURE_H
