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
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        if (u < 0) u = 0;
        if (u > 1) u = 1;
        if (v < 0) v = 0;
        if (v > 1) v = 1;
        
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        if(u<0) u=0;
        if(u>1) u=1;
        if(v<0) v=0;
        if(v>1) v=1;
        float u_img = u * width;
        float v_img = (1-v) * height;

        auto u_min = std::floor(u_img);
        auto u_max = std::min((float)width,std::ceil(u_img));
        auto v_min = std::floor(v_img);
        auto v_max = std::min((float)height,std::ceil(v_img));

        auto A00 = image_data.at<cv::Vec3b>(v_max, u_min);
        auto A01 = image_data.at<cv::Vec3b>(v_max, u_max);
        auto A10 = image_data.at<cv::Vec3b>(v_min, u_min);
        auto A11 = image_data.at<cv::Vec3b>(v_min, u_max);

        float w1 = (u_img-u_min)/(u_max-u_min);
        float w2 = (v_img-v_min)/(v_max-v_min);

        auto bot = (1-w1) * A10 + w1*A11;
        auto top = (1-w1) * A00 + w1*A01;
        auto color = (1-w2)*bot+w2*top;

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
