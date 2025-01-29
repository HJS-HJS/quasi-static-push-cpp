#include "simulation/object_slider.h"
#include <algorithm>
#include <stdexcept>

void ObjectSlider::add(std::unique_ptr<Diagram> item) {
    sliders.push_back(std::move(item));
}

void ObjectSlider::add(Diagram* item) {
    sliders.push_back(std::unique_ptr<Diagram>(item));
}

const std::vector<std::unique_ptr<Diagram>>& ObjectSlider::get_sliders() const {
    return sliders;
}

size_t ObjectSlider::size() const {
    return sliders.size();
}

Diagram* ObjectSlider::operator[](size_t i) {
    if (i >= sliders.size()) {
        throw std::out_of_range("Index out of range");
    }
    return sliders[i].get();
}

void ObjectSlider::apply_q(const Eigen::VectorXf& q) {
    for (size_t i = 0; i < sliders.size(); ++i) {
        sliders[i]->q = q.segment<3>(i * 3);
    }
}

void ObjectSlider::apply_v(const Eigen::VectorXf& v) {
    for (size_t i = 0; i < sliders.size(); ++i) {
        sliders[i]->v = v.segment<3>(i * 3);
    }
}

Eigen::VectorXf ObjectSlider::get_q() const {
    Eigen::VectorXf result(sliders.size() * 3);
    for (size_t i = 0; i < sliders.size(); ++i) {
        result.segment<3>(i * 3) = sliders[i]->q;
    }
    return result;
}

Eigen::VectorXf ObjectSlider::get_v() const {
    Eigen::VectorXf result(sliders.size() * 3);
    for (size_t i = 0; i < sliders.size(); ++i) {
        result.segment<3>(i * 3) = sliders[i]->v;
    }
    return result;
}

Eigen::VectorXf ObjectSlider::get_radius() const {
    Eigen::VectorXf result(sliders.size());
    for (size_t i = 0; i < sliders.size(); ++i) {
        result[i] = sliders[i]->radius;
    }
    return result;
}

void ObjectSlider::remove(size_t index) {
    if (index >= sliders.size()) {
        throw std::out_of_range("Index out of range");
    }
    sliders.erase(sliders.begin() + index);
}
