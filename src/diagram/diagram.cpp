#include "diagram/diagram.h"
#include <iostream>
#include <numeric>
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <algorithm>

Diagram::Diagram() : q(Eigen::Vector3f::Zero()), radius(0), dt(0.1) {
}

Diagram::Diagram(float x, float y, float rotation, float radius)
    : q(Eigen::Vector3f(x, y, rotation)), radius(radius), dt(0.1) {
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

Eigen::Vector2f Diagram::funcDiagram(float theta) const {
    float r = funcRadius(theta - q[2]);
    return Eigen::Vector2f(q[0] + r * std::cos(theta), q[1] + r * std::sin(theta));
}

Eigen::Vector2f Diagram::funcGradient(float theta) const {
    float r = funcRadius(theta - q[2]);
    float dr = funcRadiusD(theta - q[2]);
    return Eigen::Vector2f(
        -r * std::sin(theta) + dr * std::cos(theta),
        r * std::cos(theta) + dr * std::sin(theta)
    );
}

Eigen::Vector2f Diagram::point(float theta) const {
    return funcDiagram(theta);
}

std::vector<Eigen::Vector2f> Diagram::points(int npts, float tmin, float trange) const {
    std::vector<Eigen::Vector2f> result;
    Eigen::VectorXf theta = Eigen::VectorXf::LinSpaced(npts, tmin, tmin + trange);
    for (int i = 0; i < npts; ++i) {
        result.push_back(funcDiagram(theta[i]));
    }
    return result;
}

Eigen::Vector2f Diagram::tangentVector(float theta) const {
    auto grad = funcGradient(theta);
    return grad.normalized();
}

Eigen::Vector2f Diagram::normalVector(float theta) const {
    auto tangent = tangentVector(theta);
    return Eigen::Vector2f(tangent[1], -tangent[0]);
}

Eigen::Vector2f Diagram::localVelocity(float theta) const {
    auto tangent_ = tangentVector(theta);
    auto radius_ = funcRadius(theta);
    return v.head<2>() + v[2] * radius_ * tangent_;
}

Eigen::MatrixXf Diagram::localVelocityGrad(float theta, float dt, const Eigen::MatrixXf& dv) const {
    // auto rot_vec = rotVector(theta + M_PI / 2); // Assume rotVector is implemented similarly to tangentVector.
    // auto func_r = funcRadius(theta);           // Assume funcRadius is implemented similarly as before.

    // // Compute the outer product
    // Eigen::MatrixXf grad = dv.leftCols<2>() + (dv.col(2).array() * func_r).matrix().asDiagonal() * rot_vec.transpose();
    // return grad;

    Eigen::Vector2f rot_vec = rotVector(theta + M_PI / 2); // 2x1 vector
    float func_r = funcRadius(theta);

    // Compute the outer product
    Eigen::MatrixXf outer_product = dv.col(2) * (rot_vec * func_r).transpose(); // (N x 1) * (1 x 2) = N x 2
    Eigen::MatrixXf grad = dv.leftCols<2>() + outer_product; // Add N x 2 matrices
    return grad;


}

Eigen::Vector2f Diagram::rotVector(float theta) const {
    return Eigen::Vector2f(std::cos(theta), std::sin(theta));
}

void Diagram::genLimitConstant() {
    const int npts = 1000; // Number of points for integration
    const float dtheta = 2 * M_PI / npts;
    const float thickness = 0.005f;

    float area = 0.0f; // Integral over the surface
    float moment = 0.0f; // Moment of the shape

    for (int i = 0; i < npts; ++i) {
        float theta = i * dtheta;
        float r = funcRadius(theta); // Radius at angle theta
        area += 2 * thickness * r - thickness * thickness; // Surface area contribution
        moment += (1.0f / 3.0f) * r * r * r - (thickness / 4.0f) * r * r; // Moment contribution
    }

    area *= dtheta; // Final area
    moment *= dtheta; // Final moment

    limit_constant = Eigen::Matrix3f::Zero();
    limit_constant(0, 0) = area;
    limit_constant(1, 1) = area;
    limit_constant(2, 2) = moment;

    std::cout << "Generated limit constants: Area = " << area << ", Moment = " << moment << std::endl;
}

std::vector<float> Diagram::cal_collision_data(const Diagram& diagram2) const {

    int resolution = 25;
    float angle_range = M_PI;
    float center_angle = std::atan2(diagram2.q[1] - q[1], diagram2.q[0] - q[0]);

    auto rotated_points1 = points(resolution, center_angle - angle_range / 2, angle_range);
    auto rotated_points2 = diagram2.points(resolution, center_angle + M_PI - angle_range / 2, angle_range);

    // Get rough closest points
    float min_dist = 1000.0;
    int min_i = 0;
    int min_j = 0;
    for (size_t i = 0; i < rotated_points1.size() - 1; i++) {
        for (size_t j = 0; j < rotated_points2.size() - 1; j++) {
            float _distance = (rotated_points1[i] - rotated_points2[j]).norm();
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
            Eigen::Vector2f edge = rotated_points2[next_index] - rotated_points2[j];
            Eigen::Vector2f vector_to_target = rotated_points1[i] - rotated_points2[j];

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
        std::cout << "Overlap !!!!!!!!!! " << overlap_idx_list.size() << " points" << std::endl;
        std::vector<float> distance_list;
        std::vector<int> idx_list;

        for (int i : overlap_idx_list) {
            std::vector<float> _distance;
            for (size_t j = 0; j < rotated_points2.size(); ++j) {
                _distance.push_back((rotated_points1[i] - rotated_points2[j]).norm());
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
                float _distance = (rotated_points1[i] - rotated_points2[j]).norm();
                if (_distance < min_dist) {
                    min_dist = _distance;
                    min_i = i;
                    min_j = j;
                }
            }
        }
    }

    std::vector<float> collision_data(3, 0.0f); // Stores angle1, angle2, and distance
    target_angle = std::atan2(rotated_points1[min_i][1] - q[1], rotated_points1[min_i][0] - q[0]);
    obs_angle = std::atan2(rotated_points2[min_j][1] - diagram2.q[1], rotated_points2[min_j][0] - diagram2.q[0]);

    collision_data = {
        target_angle,
        obs_angle,
        min_dist,
    };

    return collision_data;
}

Diagram::~Diagram() {
}
