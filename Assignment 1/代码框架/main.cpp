#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();;
    double angle = rotation_angle / 180.0 * MY_PI;
    rotationMatrix << cos(angle),-sin(angle),0,0,
                    sin(angle),cos(angle),0,0,
                    0,0,1,0,
                    0,0,0,1;
    model = rotationMatrix * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    double r,l,t,b;
    double fov_angle = eye_fov / 180.0 *MY_PI;
    t = zNear * tan(fov_angle / 2);
    b = -t;
    r = t * aspect_ratio;
    l = -r;
    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f persp = Eigen::Matrix4f::Identity();
    scale << 2/(r-l),0,0,0,
            0,2/(t-b),0,0,
            0,0,2/(zFar-zNear),0,
            0,0,0,1;
    trans << 1,0,0,-(r+l)/2,
            0,1,0,-(t+b)/2,
            0,0,1,-(-zNear-zFar)/2,
            0,0,0,1;
    persp << -zNear,0,0,0,
            0,-zNear,0,0,
            0,0,-zNear-zFar,-zNear*zFar,
            0,0,1,0;
    projection = scale * trans * persp;
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f Rodri4f = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f Rodri3f = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f N = Eigen::Matrix3f::Identity();

    N << 0,-axis.z(),axis.y(),
            axis.z(),0,-axis.x(),
            -axis.y(),axis.x(),0;

    double rotate_angle = angle/180.0*MY_PI;
    Rodri3f = cos(rotate_angle)*I+(1-cos(rotate_angle)) * axis * axis.transpose() + sin(rotate_angle)*N;
    Rodri4f.block(0,0,3,3) = Rodri3f;
    Rodri4f(3,3)=1;
    return Rodri4f;
    /*Eigen::Matrix4f Rodri = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f N = Eigen::Matrix4f::Identity();
    Eigen::Vector4f raxis;
    
    double rotate_angle = angle/180.0*MY_PI;
    raxis << axis.x(),axis.y(),axis.z(),0;

    N << 0,-axis.z(),axis.y(),0,
            axis.z(),0,-axis.x(),0,
            -axis.y(),axis.x(),0,0,
            0,0,0,1;
    Rodri = cos(rotate_angle)*I+(1-cos(rotate_angle)) * raxis * raxis.transpose() + sin(rotate_angle)*N;
    Rodri(3,3)=1;
    return Rodri;*/
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;  //定义命令行开关标志，默认为关
    std::string filename = "output.png";

    Eigen::Vector3f rotated_axis(0,0,1);
    float rangle = 0, ra;

    if (argc >= 3) {  //接收到的参数大于三个，即检测到通过命令行传入参数时
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {  //接收到的参数为四个，那么说明命令行输入了文件名参数
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);  //设定700*700像素的光栅器视口

    Eigen::Vector3f eye_pos = {0, 0, 5};  //设定相机位置

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};  //设定三顶点序号,用于画图时确定需要处理几个顶点，这里表示的是三个顶点

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);  //保存多个图形的顶点和序号，本次作业只涉及一个图形

    int key = 0;  //键盘输入
    int frame_count = 0;  //帧序号

    if (command_line) { //如果命令行开关标志为开（这一段代码是为了应用命令行传入的参数，比如初始角度和文件名）
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);  //初始化帧缓存和深度缓存（本次作业本次作业只涉及一个图形，所以不涉及深度）

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        r.set_Rodrigues(get_rotation(rotated_axis, rangle));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imwrite(filename, image);

        return 0;
    }

    bool rFlag = false;

    std::cout << "please enter the axis and the angle: "<< std::endl;
    std::cin >> rotated_axis.x() >> rotated_axis.y() >> rotated_axis.z() >> ra;  //定义罗德里格斯旋转轴和角

    while (key != 27) {  //只要没有检测到按下ESC就循环(ESC的ASCII码是27)
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        if(rFlag)
            r.set_Rodrigues(get_rotation(rotated_axis,rangle));
        else
            r.set_Rodrigues(get_rotation({0,0,1},0));
        
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);  //显示图像
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';  //显示当前是第几帧画面

        if (key == 'a') {  //按下a，逆时针旋转10°
            angle += 10;
        }
        else if (key == 'd') {  //按下d，顺时针旋转10°
            angle -= 10;
        }
        else if (key == 'e')  //按下e，绕给定旋转轴旋转ra°
        {
            rFlag = true;
            rangle += ra;
        }
        else if (key == 'q')  //按下q，绕给定旋转轴反向旋转ra°
        {
            rFlag = true;
            rangle -= ra;
        }
    }

    return 0;
}
