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
    translate << 1, 0, 0, -eye_pos[0], 
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2], 
                 0, 0, 0, 1;

    view = translate * view;

    return view; // view matrix translates camera and models simutanously such that camera is centered at the origin releative to the world coordiante system
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    float alpha = rotation_angle * MY_PI / 180.0; 
    model(0, 0) = std::cos(alpha); model(0, 1) = -std::sin(alpha);
    model(1, 0) = std::sin(alpha); model(1, 1) = std::cos(alpha);

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it
    
    // Inspired by https://cs184.eecs.berkeley.edu/sp24/lecture/4-97/transforms
    float near = -zNear;
    float far = -zFar;
    float top = near * std::tan(eye_fov * MY_PI / 180.0f / 2.0f); 
    float bottom = -top;
    float right = top * aspect_ratio;
    float left = -right;
    projection(0, 0) = near / right; 
    projection(1, 1) = near / top;
    projection(2, 2) = -(far + near) / (far - near);
    projection(2, 3) = -2 * far * near / (far - near);
    projection(3, 2) = -1;

    return projection;
}

// Rodrigues' Rotation Formula: Axis + Angle 
Eigen::Matrix4f get_rotation(Vector3f axis, float angle) { // Axis about origin

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
        
    Eigen::Matrix3f N; N.setZero();
    N(0, 1) = -axis.z(); 
    N(0, 2) = axis.y();
    N(1, 0) = axis.z();
    N(1, 2) = -axis.x();
    N(2, 0) = -axis.y();
    N(2, 1) = axis.x();

    Eigen::Matrix3f R = std::cos(angle * MY_PI / 180.0f) * I + (1 - std::cos(angle * MY_PI / 180.0f)) * axis * axis.transpose() + std::sin(angle * MY_PI / 180.0f) * N;

    // Rotation in homogenous coordinate
    Eigen::Matrix4f R4 = Eigen::Matrix4f::Identity();
    R4.block<3, 3>(0, 0) = R;

    return R4;
}


int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }


    //// Testing get_rotation
    //Eigen::Matrix4f model_matrix = get_model_matrix(angle);

    //// Print the matrix
    //std::cout << "Model Matrix:" << std::endl;
    //std::cout << model_matrix << std::endl;

    //Eigen::Vector3f axis(0, 0, 1);
    //// Get the rotation matrix
    //Eigen::Matrix4f rotation_matrix = get_rotation(axis, angle);

    //// Print the matrix
    //std::cout << "Rotation matrix around axis (0, 0, 1) with angle " << angle << " degrees:" << std::endl;
    //std::cout << rotation_matrix << std::endl;


    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
