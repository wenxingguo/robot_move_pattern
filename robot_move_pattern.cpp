#include "robot_move_pattern.h"
#include <Eigen/Geometry>


std::vector<double> moveLInBaseCoor(const std::vector<double>& current_tool_pose, const std::vector<double>& direction, const double d)
{
    assert(current_tool_pose.size() == 6);
    assert(direction.size() == 3);
    Eigen::Vector3d direction_vector(direction[0], direction[1], direction[2]);
    direction_vector /= direction_vector.squaredNorm();
   
    std::vector<double> result = current_tool_pose;
    for (int i = 0; i < direction_vector.rows(); ++i) {
        result[i] += direction_vector[i] * d;
    }
    return result;
}


std::vector<double> moveLInToolCoor(const std::vector<double>& current_tool_pose, const std::vector<double>& direction, const double d)
{

    assert(current_tool_pose.size() == 6);
    assert(direction.size() == 3);
    Eigen::Vector3d rotation_vector(current_tool_pose[3], current_tool_pose[4], current_tool_pose[5]);
    Eigen::Vector3d direction_vector(direction[0], direction[1], direction[2]);
    direction_vector /= direction_vector.squaredNorm();

    Eigen::AngleAxisd rotation_axis(rotation_vector.norm(), rotation_vector.normalized());

    Eigen::Matrix3d rotation_matrix = rotation_axis.matrix();
    Eigen::Vector3d direction_vector_inBasecoor = rotation_matrix * direction_vector;

    direction_vector_inBasecoor = direction_vector_inBasecoor.normalized();

    std::vector<double> result = current_tool_pose;
    for (int i = 0; i < direction_vector_inBasecoor.rows(); ++i) {
        result[i] += direction_vector_inBasecoor[i] * d;
    }
    return result;
}

std::vector<double> roateInToolCoor(const std::vector<double>& current_tool_pose, const std::vector<double>& direction, const double theta)
{
    assert(current_tool_pose.size() == 6);
    assert(direction.size() == 3);
    Eigen::Vector3d rotation_vector(current_tool_pose[3], current_tool_pose[4], current_tool_pose[5]);
    Eigen::Vector3d direction_vector(direction[0], direction[1], direction[2]);
    direction_vector /= direction_vector.squaredNorm();

    Eigen::AngleAxisd rotation_axis_tool(rotation_vector.norm(), rotation_vector.normalized());
    Eigen::AngleAxisd rotation_axis(theta, direction_vector);
    Eigen::AngleAxisd rotation_axis_final(rotation_axis_tool * rotation_axis);
    Eigen::Vector3d rotation_vector_with_angle = rotation_axis_final.axis() * rotation_axis_final.angle();
    std::vector<double> result = current_tool_pose;
    for (int i = 0; i < rotation_vector_with_angle.rows(); ++i) {
        result[i + 3] = rotation_vector_with_angle[i];
    }

    return result;
}

std::vector<double> roateInBaseCoor(const std::vector<double>& current_tool_pose, const std::vector<double>& direction, const double theta)
{
    assert(current_tool_pose.size() == 6);
    assert(direction.size() == 3);

    Eigen::Vector3d rotation_vector(current_tool_pose[3], current_tool_pose[4], current_tool_pose[5]);

    Eigen::AngleAxisd rotation_axis(rotation_vector.norm(), rotation_vector.normalized());

    Eigen::Matrix3d rotation_matrix = rotation_axis.matrix();

    // 该向量是在Base坐标系下，将其转化到工具坐标系下
    Eigen::Vector3d direction_vector(direction[0], direction[1], direction[2]);
    direction_vector /= direction_vector.squaredNorm();

    Eigen::Vector3d direction_vector_in_tool = rotation_matrix.transpose() * direction_vector; // 将base坐标系下的方向转化到工具坐标系
    std::vector<double> std_direction_vector({ direction_vector_in_tool[0], direction_vector_in_tool[1], direction_vector_in_tool[2] });

    return roateInToolCoor(current_tool_pose, std_direction_vector, theta);
}
