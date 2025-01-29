#ifndef OBJECT_PUSHER_H
#define OBJECT_PUSHER_H

#include <vector>
#include <memory>
#include <map>
#include <string>
#include <Eigen/Dense>
#include "diagram/diagram_all.h"

class ObjectPusher {
public:
    // Constructor
    ObjectPusher(int n_finger, float finger_angle,
                 const std::string& type, 
                 const std::map<std::string, float>& diagram_param,
                 float width, float width_limit_max, float width_limit_min,
                 float center_x, float center_y, float rotation);

    // Public members
    Eigen::Vector4f q; // [center_x, center_y, rotation, width]
    Eigen::Vector4f v; // velocity
    std::vector<std::unique_ptr<Diagram>> pushers;

    // Apply configurations to all pushers
    void apply_q(const Eigen::Vector4f& q);
    void apply_v(const Eigen::Vector4f& velocity);

    // Access pushers
    const std::vector<std::unique_ptr<Diagram>>& get_pushers() const;

    // Get change in velocity (delta v)
    // std::vector<Eigen::Vector3f> pusher_dv(float dt = 0.001f) const;
    std::vector<Eigen::Matrix<float, 4, 3>> pusher_dv(float dt) const;

    // Getters
    Eigen::VectorXf get_r() const;
    Eigen::Vector2f get_c() const;
    float get_rot() const;
    Eigen::Matrix3f get_rot_matrix() const;

    // Operator overloads
    size_t size() const;
    Diagram* operator[](size_t index);

    // Iterator support
    auto begin() -> decltype(pushers.begin()) {
        return pushers.begin();
    }

    auto end() -> decltype(pushers.end()) {
        return pushers.end();
    }

    auto begin() const -> decltype(pushers.begin()) {
        return pushers.begin();
    }

    auto end() const -> decltype(pushers.end()) {
        return pushers.end();
    }

    // Destructor
    ~ObjectPusher();

private:
    float width_limit_max;
    float width_limit_min;
    Eigen::MatrixXf m_q_rel;
};

#endif // OBJECT_PUSHER_H
