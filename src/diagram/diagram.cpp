#include "diagram/diagram.h"
#include <iostream>
#include <numeric>
#include <cmath>
#include <limits>
#include <algorithm>
#include <vector>
#include <array>

Diagram::Diagram() : q{0.0f, 0.0f, 0.0f}, radius(0), dt(0.1) {
}

Diagram::Diagram(float x, float y, float rotation, float radius)
    : q{x, y, rotation}, radius(radius), dt(0.1) {
}

void Diagram::initialize() {
    genLimitConstant();
}

float Diagram::funcRadius(float theta) const {
    return 1.0f; // Placeholder
}

float Diagram::funcRadiusD(float theta) const {
    return 0.0f; // Placeholder
}

std::array<float, 2> Diagram::funcDiagram(float theta) const {
    float r = funcRadius(theta - q[2]);
    return {q[0] + r * std::cos(theta), q[1] + r * std::sin(theta)};
}

std::array<float, 2> Diagram::funcGradient(float theta) const {
    float r = funcRadius(theta - q[2]);
    float dr = funcRadiusD(theta - q[2]);
    return {
        -r * std::sin(theta) + dr * std::cos(theta),
        r * std::cos(theta) + dr * std::sin(theta)
    };
}

std::array<float, 2> Diagram::point(float theta) const {
    return funcDiagram(theta);
}

std::vector<std::array<float, 2>> Diagram::points(int npts, float tmin, float trange) const {
    std::vector<std::array<float, 2>> result;
    for (int i = 0; i < npts; ++i) {
        float theta = tmin + i * (trange / (npts - 1));
        result.push_back(funcDiagram(theta));
    }
    return result;
}

std::array<float, 2> Diagram::tangentVector(float theta) const {
    auto grad = funcGradient(theta);
    float norm = normalize(grad);
    return {grad[0] / norm, grad[1] / norm};
}

std::array<float, 2> Diagram::normalVector(float theta) const {
    auto tangent = tangentVector(theta);
    return {tangent[1], -tangent[0]};
}

std::array<float, 2> Diagram::rotVector(float theta) const {
    return {std::cos(theta), std::sin(theta)};
}

std::array<float, 2> Diagram::localVelocity(float theta) const {
    auto tangent_ = tangentVector(theta);
    float radius_ = funcRadius(theta);
    return {v[0] + v[2] * radius_ * tangent_[0], v[1] + v[2] * radius_ * tangent_[1]};
}
Eigen::MatrixXf Diagram::localVelocityGrad(float theta, float dt) const {
    Eigen::MatrixXf dv = Eigen::MatrixXf::Identity(3, 3) * dt;
    std::array<float, 2> rot_vec = rotVector(theta + M_PI / 2);
    float func_r = funcRadius(theta);

    Eigen::MatrixXf outer_product = dv.col(2) * Eigen::RowVector2f(rot_vec[0] * func_r, rot_vec[1] * func_r);
    Eigen::MatrixXf grad = dv.leftCols<2>() + outer_product;
    return grad / dt;
}
Eigen::MatrixXf Diagram::localVelocityGrad(float theta, float dt, const std::array<std::array<float, 3>, 4>& dv) const {
    std::array<float, 2> rot_vec = rotVector(theta + M_PI / 2);
    float func_r = funcRadius(theta);

    Eigen::MatrixXf dv_mat(4, 3);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 3; ++j)
            dv_mat(i, j) = dv[i][j];

    Eigen::VectorXf dv_col2 = dv_mat.col(2);
    Eigen::RowVector2f rot_scaled(rot_vec[0] * func_r, rot_vec[1] * func_r);

    Eigen::MatrixXf outer_product = dv_col2 * rot_scaled;

    Eigen::MatrixXf grad = dv_mat.leftCols(2) + outer_product;

    return grad / dt;

}

void Diagram::genLimitConstant() {
    const int npts = 1000; // Number of points for integration
    const float dtheta = 2 * M_PI / npts;
    const float thickness = 0.005f;

    float area = 0.0f;
    float moment = 0.0f;

    for (int i = 0; i < npts; ++i) {
        float theta = i * dtheta;
        float r = funcRadius(theta);
        area += (1.0f / 2.0f) * (2.0f * thickness * r - thickness * thickness);
        moment += (1.0f / 3.0f) * r * r * r - (thickness / 4.0f) * r * r;
    }

    area *= dtheta;
    moment *= dtheta;

    limit_constant = Eigen::Matrix3f::Zero();
    limit_constant(0, 0) = area;
    limit_constant(1, 1) = area;
    limit_constant(2, 2) = moment;
}

std::array<float, 3> Diagram::cal_collision_data(const Diagram& diagram2) const {
    int resolution = 25;
    float angle_range = M_PI;
    float center_angle = std::atan2(diagram2.q[1] - q[1], diagram2.q[0] - q[0]);

    // Check target is in diagram
    float betDistance_ = (q - diagram2.q).head<2>().norm();
    float checkDistance_ = diagram2.funcRadius(center_angle + M_PI - diagram2.q[2]) + funcRadius(center_angle - q[2]);
    bool is_collision = false;
    
    if (checkDistance_ > betDistance_){
        is_collision = true;
    }

    std::vector<std::array<float, 2>> rotated_points1 = points(resolution, center_angle - angle_range / 2, angle_range);
    std::vector<std::array<float, 2>> rotated_points2 = diagram2.points(resolution, center_angle + M_PI - angle_range / 2, angle_range);

    // Get rough closest points
    float min_dist = 1000.0;
    int min_i = 0;
    int min_j = 0;
    for (size_t i = 0; i < rotated_points1.size() - 1; i++) {
        for (size_t j = 0; j < rotated_points2.size() - 1; j++) {
            // float _distance = normalize(rotated_points1[i] - rotated_points2[j]);
            float _distance = normalize(rotated_points1[i], rotated_points2[j]);
            if (_distance < min_dist) {
                min_dist = _distance;
                min_i = i;
                min_j = j;
            }
        }
    }

    // Find inner points
    angle_range = M_PI / 10;
    float target_angle = std::atan2(rotated_points1[min_i][1] - q[1], rotated_points1[min_i][0] - q[0]);
    float obs_angle = std::atan2(rotated_points2[min_j][1] - diagram2.q[1], rotated_points2[min_j][0] - diagram2.q[0]);

    rotated_points1 = points(40, target_angle - angle_range, angle_range * 2);
    rotated_points2 = diagram2.points(40, obs_angle - angle_range, angle_range * 2);

    // Add extra point for calculating cross product
    rotated_points2.push_back(diagram2.point(obs_angle + M_PI / 3));
    rotated_points2.push_back(diagram2.point(obs_angle + M_PI));
    rotated_points2.push_back(diagram2.point(obs_angle - M_PI / 3));

    bool is_overlap = false;
    std::vector<int> overlap_idx_list;

    for (size_t i = 0; i < rotated_points1.size(); ++i) {
        bool all_same_sign = true; // Check if all cross product signs are the same

        float previous_sign = 0; // Store previous cross product sign
        for (size_t j = 0; j < rotated_points2.size(); ++j) {
            size_t next_index = (j + 1) % rotated_points2.size();
            std::array<float, 2> edge = subtractArrays(rotated_points2[next_index], rotated_points2[j]);
            std::array<float, 2> vector_to_target = subtractArrays(rotated_points1[i], rotated_points2[j]);

            float cross_product = edge[0] * vector_to_target[1] - edge[1] * vector_to_target[0];

            float current_sign = (cross_product > 0) ? 1 : (cross_product < 0) ? -1 : 0;
            if (previous_sign == 0) {
                previous_sign = current_sign; // Initialize
            } else if (current_sign != 0 && current_sign != previous_sign) {
                all_same_sign = false; // Sign changed
                break;
            }
        }

        if (all_same_sign) {
            is_overlap = true;
            overlap_idx_list.push_back(i);
        }
    }

    if (is_overlap) {
        std::vector<float> distance_list;
        std::vector<int> idx_list;

        for (int i : overlap_idx_list) {
            std::vector<float> _distance;
            for (size_t j = 0; j < rotated_points2.size(); ++j) {
                _distance.push_back(normalize(subtractArrays(rotated_points1[i], rotated_points2[j])));
            }
            auto min_iter = std::min_element(_distance.begin(), _distance.end());
            float min_value = *min_iter;
            size_t min_index = std::distance(_distance.begin(), min_iter);

            distance_list.push_back(min_value);
            idx_list.push_back(min_index);
        }

        auto max_iter = std::max_element(distance_list.begin(), distance_list.end());
        float max_value = *max_iter;
        size_t max_index = std::distance(distance_list.begin(), max_iter);

        min_i = overlap_idx_list[max_index];
        min_j = idx_list[max_index];
        min_dist = -max_value;
    } else {
        min_dist = 1000.0;
        for (size_t i = 0; i < rotated_points1.size() - 1; i++) {
            for (size_t j = 0; j < rotated_points2.size() - 1; j++) {
                float _distance = normalize(subtractArrays(rotated_points1[i], rotated_points2[j]));
                if (_distance < min_dist) {
                    min_dist = _distance;
                    min_i = i;
                    min_j = j;
                }
            }
        }
    }

    target_angle = std::atan2(rotated_points1[min_i][1] - q[1], rotated_points1[min_i][0] - q[0]);
    obs_angle = std::atan2(rotated_points2[min_j][1] - diagram2.q[1], rotated_points2[min_j][0] - diagram2.q[0]);
    
    if (is_collision && min_dist > 0){
        min_dist *= -1;
    }

    std::array<float, 3> collision_data = {
        target_angle,
        obs_angle,
        min_dist,
    };

    return collision_data;
}

float Diagram::normalize(const std::array<float, 2>& v) const {
    return std::sqrt(std::pow(v[0], 2) + std::pow(v[1], 2));
}

float Diagram::normalize(const std::array<float, 2>& v1, const std::array<float, 2>& v2) const {
    return std::sqrt(std::pow(v1[0] - v2[0], 2) + std::pow(v1[1] - v2[1], 2));
}

std::array<float, 2> Diagram::subtractArrays(const std::array<float, 2>& a, const std::array<float, 2>& b) const {
    return {a[0] - b[0], a[1] - b[1]};
}

Diagram::~Diagram() {
}
