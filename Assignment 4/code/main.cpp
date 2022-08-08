#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if(control_points.size()==1)
        return cv::Point2f(control_points[0]);
    std::vector<cv::Point2f> points;
    for (int i =1;i<control_points.size();i++)
    {
        points.emplace_back(control_points[i].x*t+control_points[i-1].x*(1-t),control_points[i].y*t+control_points[i-1].y*(1-t));
    }
    return recursive_bezier(points,t);
    
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.

    for (double t = 0; t < 1; t+=0.001)
    {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }
    
}

void bezier_interpolated(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    for (double t = 0; t < 1; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);

        cv::Point2i p0(point.x-std::floor(point.x)<0.5?std::floor(point.x):std::ceil(point.x),
                        point.y - std::floor(point.y)<0.5?std::floor(point.y):std::ceil(point.y));
        std::vector<cv::Point2i> p_around = {p0,cv::Point2i(p0.x-1,p0.y),
                            cv::Point2i(p0.x,p0.y-1),cv::Point2i(p0.x-1,p0.y-1)};
        float sum_d = 0.f;
        float max_d = std::sqrt(2);
        std::vector<float> dis;
        for(int i = 0;i<4;i++)
        {
            cv::Point2f pp(p_around[i].x+0.5f,p_around[i].y+0.5f);
            float d = max_d - std::sqrt(std::pow(point.x-pp.x,2)+std::pow(point.y-pp.y,2));
            dis.push_back(d);
            sum_d += d;
        }
        for(int i = 0;i<4;i++)
        {
            float w = dis[i]/sum_d;
            window.at<cv::Vec3b>(p_around[i].y, p_around[i].x)[1] = std::min(255.f,window.at<cv::Vec3b>(p_around[i].y, p_around[i].x)[1]+255.f*w);
        }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            // naive_bezier(control_points, window);
            bezier_interpolated(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("Have interpolation.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
