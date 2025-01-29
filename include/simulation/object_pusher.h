#ifndef OBJECT_PUSHER_H
#define OBJECT_PUSHER_H

#include <vector>
#include <memory>
#include <map>
#include <string>
#include <Eigen/Dense>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <cmath>
#include <vector>
#include <array>
#include <map>
#include <memory>
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
    std::array<float, 4> q; // [center_x, center_y, rotation, width]
    std::array<float, 4> v; // velocity
    std::vector<std::unique_ptr<Diagram>> pushers;

    // Apply configurations to all pushers
    void apply_q(const std::array<float, 4>& q);
    void apply_v(const std::array<float, 4>& velocity);

    // Access pushers
    const std::vector<std::unique_ptr<Diagram>>& get_pushers() const;

    // Get change in velocity (delta v)
    std::vector<std::array<std::array<float, 3>, 4>> pusher_dv(float dt) const;


    // Getters
    std::vector<float> get_r() const;
    std::array<float, 2> get_c() const;
    float get_rot() const;

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
    std::vector<std::array<float, 3>> m_q_rel;
};

#endif // OBJECT_PUSHER_H
