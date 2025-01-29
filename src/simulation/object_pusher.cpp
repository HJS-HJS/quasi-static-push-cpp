#include "simulation/object_pusher.h"
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <cmath>

// Constructor
ObjectPusher::ObjectPusher(int n_finger, float finger_angle,
                           const std::string& type, 
                           const std::map<std::string, float>& diagram_param,
                           float width, float width_limit_max, float width_limit_min,
                           float center_x, float center_y, float rotation)
    : q(center_x, center_y, rotation, width),
      v(Eigen::Vector4f::Zero()),
      width_limit_max(width_limit_max),
      width_limit_min(width_limit_min) {

    for (int i = 0; i < n_finger; ++i) {
        if (type == "circle")            pushers.push_back(std::make_unique<Circle>(0.0f, 0.0f, 0.0f, diagram_param.at("r")));
        else if (type == "ellipse")      pushers.push_back(std::make_unique<Ellipse>(0.0f, 0.0f, 0.0f, diagram_param.at("a"), diagram_param.at("b")));
        else if (type == "superellipse") pushers.push_back(std::make_unique<SuperEllipse>(0.1f, 0.0f, 0.0f, diagram_param.at("a"), diagram_param.at("b"), diagram_param.at("n")));
        else if (type == "rpolygon")     pushers.push_back(std::make_unique<RPolygon>(0.0f, 0.0f, 0.0f, diagram_param.at("a"), diagram_param.at("k")));
        else if (type == "srpolygon")    pushers.push_back(std::make_unique<SmoothRPolygon>(0.0f, 0.0f, 0.0f, diagram_param.at("a"), diagram_param.at("k")));
    }

    // Initialize pushers
    finger_angle = finger_angle * M_PI / 180;
    float f_heading = (finger_angle - M_PI) / 2;

    m_q_rel.resize(n_finger, 3); // n_finger x 3 크기로 행렬 설정
    for (int i = 0; i < n_finger; ++i) {
        m_q_rel(i, 0) = std::cos(-finger_angle * i + f_heading); // x 좌표
        m_q_rel(i, 1) = std::sin(-finger_angle * i + f_heading); // y 좌표
        m_q_rel(i, 2) = (float)(finger_angle / 2) + (float)((M_PI - finger_angle) * i); // z 좌표
    }

    apply_q(q);
    apply_v(v);
}

const std::vector<std::unique_ptr<Diagram>>& ObjectPusher::get_pushers() const {
    return pushers;
}

// Apply position configuration
void ObjectPusher::apply_q(const Eigen::Vector4f& q) {
    this->q = q;

    // Compute rotation matrix
    Eigen::Matrix2f rot_matrix;
    rot_matrix <<
        std::cos(q[2]), -std::sin(q[2]),
        std::sin(q[2]),  std::cos(q[2]);

    for (size_t idx = 0; idx < pushers.size(); ++idx) {
        Eigen::Vector2f rel_pos = m_q_rel.row(idx).head<2>().transpose() * q[3];
        // Eigen::Vector2f rel_pos = m_q_rel[idx].head<2>() * q[3];
        Eigen::Vector2f global_pos = rot_matrix * rel_pos + q.head<2>();

        pushers[idx]->q[0] = global_pos[0];
        pushers[idx]->q[1] = global_pos[1];
        
        // pushers[idx]->q[2] = q[2] + m_q_rel[idx][2];
        pushers[idx]->q[2] = q[2] + m_q_rel(idx, 2);
    }

    // Clamp width to limits
    if (q[3] > width_limit_max) {
        this->q[3] = width_limit_max;
    } else if (q[3] < width_limit_min) {
        this->q[3] = width_limit_min;
    }
}

// Apply velocity configuration
void ObjectPusher::apply_v(const Eigen::Vector4f& velocity) {
    this->v = velocity;

    Eigen::Matrix3f rot_matrix;
    rot_matrix <<
        std::cos(q[2]), -std::sin(q[2]), 0,
        std::sin(q[2]),  std::cos(q[2]), 0,
        0,              0,              1;

    Eigen::Vector3f w(0, 0, velocity[2]); // Angular velocity vector

    for (size_t idx = 0; idx < pushers.size(); ++idx) {
        Eigen::Vector3f rel_pos = m_q_rel.row(idx).transpose() * q[3];
        Eigen::Vector3f cross_product = w.cross(rel_pos);

        pushers[idx]->v[0] = velocity[0] + cross_product[0];
        pushers[idx]->v[1] = velocity[1] + cross_product[1];
        pushers[idx]->v[2] = velocity[2];
    }
}

std::vector<Eigen::Matrix<float, 4, 3>> ObjectPusher::pusher_dv(float dt) const {
    // 결과를 저장할 벡터 초기화
    std::vector<Eigen::Matrix<float, 4, 3>> d_set(pushers.size(), Eigen::Matrix<float, 4, 3>::Zero());

    // 공통적으로 사용되는 값들
    Eigen::Matrix<float, 4, 3> empty_dv = Eigen::Matrix<float, 4, 3>::Zero();
    empty_dv.topLeftCorner<3, 3>() = Eigen::Matrix3f::Identity(); // 상위 3x3 부분에 단위행렬 삽입
    empty_dv *= dt; // dt로 스케일링

    // Z축 회전 속도 벡터
    Eigen::Vector3f w(0, 0, dt);
    // Compute rotation matrix
    Eigen::Matrix2f rot_matrix;
    rot_matrix <<
        std::cos(q[2]), -std::sin(q[2]),
        std::sin(q[2]),  std::cos(q[2]);

    for (size_t i = 0; i < pushers.size(); ++i) {
        // 각 푸셔의 d_set 초기화
        d_set[i] = empty_dv;

        // Rotation delta t 계산
        Eigen::Vector3f rel_vec = m_q_rel.row(i).transpose(); // i번째 행을 (3x1) 벡터로 가져옴
        Eigen::Vector2f rotation_delta = (w.cross(rel_vec)).head<2>() * q[3];
        d_set[i].row(2).head<2>() += rotation_delta;

        // Gripper width delta t 계산
        Eigen::Vector2f gripper_delta = (rel_vec.head<2>()) * dt;
        d_set[i].row(3).head<2>() += gripper_delta;
    }

    return d_set;
}


// Get radii of all pushers
Eigen::VectorXf ObjectPusher::get_r() const {
    Eigen::VectorXf radii(pushers.size());
    for (size_t i = 0; i < pushers.size(); ++i) {
        radii[i] = pushers[i]->radius;
    }
    return radii;
}

// Get center
Eigen::Vector2f ObjectPusher::get_c() const {
    return q.head<2>();
}

// Get rotation
float ObjectPusher::get_rot() const {
    return q[2];
}

// Get rotation matrix
Eigen::Matrix3f ObjectPusher::get_rot_matrix() const {
    Eigen::Matrix3f rot_matrix;
    rot_matrix <<
        std::cos(q[2]), -std::sin(q[2]), 0,
        std::sin(q[2]),  std::cos(q[2]), 0,
        0,              0,              1;
    return rot_matrix;
}

// Get number of pushers
size_t ObjectPusher::size() const {
    return pushers.size();
}

// Access pusher by index
Diagram* ObjectPusher::operator[](size_t index) {
    if (index >= pushers.size()) throw std::out_of_range("Index out of range");
    return pushers[index].get();
}

// Destructor
ObjectPusher::~ObjectPusher() = default;
