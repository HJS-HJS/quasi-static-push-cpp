#include "simulation/object_pusher.h"


// Constructor
ObjectPusher::ObjectPusher(int n_finger, float finger_angle,
                           const std::string& type, 
                           const std::map<std::string, float>& diagram_param,
                           float width, float width_limit_max, float width_limit_min,
                           float center_x, float center_y, float rotation)
    : q{center_x, center_y, rotation, width},
      v{0.0f, 0.0f, 0.0f, 0.0f},
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

    for (int i = 0; i < n_finger; ++i) {
        m_q_rel.push_back({
            std::cos(-finger_angle * i + f_heading),
            std::sin(-finger_angle * i + f_heading),
            -float(finger_angle / 2) - (float(M_PI - finger_angle) * i)
        });
    }

    apply_q(q);
    apply_v(v);
}

const std::vector<std::unique_ptr<Diagram>>& ObjectPusher::get_pushers() const {
    return pushers;
}

// Apply position configuration
void ObjectPusher::apply_q(const std::array<float, 4>& q) {
    this->q = q;
    float cos_r = std::cos(q[2]);
    float sin_r = std::sin(q[2]);

    // Compute rotation matrix
    // Eigen::Matrix2f rot_matrix;
    // rot_matrix <<
    //     std::cos(q[2]), std::sin(q[2]),
    //     std::sin(q[2]), -std::cos(q[2]);

    for (size_t idx = 0; idx < pushers.size(); ++idx) {
        float rel_x = m_q_rel[idx][0] * q[3];
        float rel_y = m_q_rel[idx][1] * q[3];
        float global_x = cos_r * rel_x + sin_r * rel_y + q[0];
        float global_y = sin_r * rel_x - cos_r * rel_y + q[1];
        
        pushers[idx]->q = {global_x, global_y, q[2] + m_q_rel[idx][2]};
    }

    // Clamp width to limits
    this->q[3] = std::clamp(this->q[3], width_limit_min, width_limit_max);
}

// Apply velocity configuration
void ObjectPusher::apply_v(const std::array<float, 4>& velocity) {
    this->v = velocity;
    for (size_t idx = 0; idx < pushers.size(); ++idx) {
        float rel_x = m_q_rel[idx][0] * q[3];
        float rel_y = m_q_rel[idx][1] * q[3];
        float cross_x = -velocity[2] * rel_y;
        float cross_y = velocity[2] * rel_x;
        
        pushers[idx]->v = {velocity[0] + cross_x, velocity[1] + cross_y, velocity[2]};
    }
}

std::vector<std::array<std::array<float, 3>, 4>> ObjectPusher::pusher_dv(float dt) const {
    std::vector<std::array<std::array<float, 3>, 4>> d_set(pushers.size(),
     {{
        {dt  , 0.0f, 0.0f},
        {0.0f, dt,   0.0f}, 
        {0.0f, 0.0f, dt  }, 
        {0.0f, 0.0f, 0.0f}
        }});
    
    float cos_r = std::cos(q[2]);
    float sin_r = std::sin(q[2]);

    for (size_t i = 0; i < pushers.size(); ++i) {

        float rel_x = m_q_rel[i][0];
        float rel_y = m_q_rel[i][1];

        // 회전 변환 적용
        float new_rel_x = (rel_x * cos_r + rel_y * sin_r);
        float new_rel_y = (rel_x * sin_r - rel_y * cos_r);

        float cross_x = -dt * new_rel_y * q[3];
        float cross_y = dt * new_rel_x * q[3];

        d_set[i][2][0] += cross_x;
        d_set[i][2][1] += cross_y;

        d_set[i][3][0] += new_rel_x * dt;
        d_set[i][3][1] += new_rel_y * dt;
    }
    
    return d_set;
}


// Get radii of all pushers
std::vector<float> ObjectPusher::get_r() const {
    std::vector<float> radii;
    for (const auto& pusher : pushers) {
        radii.push_back(pusher->radius);
    }
    return radii;
}

// Get center
std::array<float, 2> ObjectPusher::get_c() const {
    return {q[0], q[1]};
}

// Get rotation
float ObjectPusher::get_rot() const {
    return q[2];
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
