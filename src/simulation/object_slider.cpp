#include "simulation/object_slider.h"

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

void ObjectSlider::apply_q(const std::vector<float>& q) {
    for (size_t i = 0; i < sliders.size(); ++i) {
        sliders[i]->q = {q[i * 3], q[i * 3 + 1], q[i * 3 + 2]};
    }
}

void ObjectSlider::apply_v(const std::vector<float>& v) {
    for (size_t i = 0; i < sliders.size(); ++i) {
        sliders[i]->v = {v[i * 3], v[i * 3 + 1], v[i * 3 + 2]};
    }
}

std::vector<float> ObjectSlider::get_q() const {
    std::vector<float> result;
    result.reserve(sliders.size() * 3);
    for (const auto& slider : sliders) {
        result.push_back(slider->q[0]);
        result.push_back(slider->q[1]);
        result.push_back(slider->q[2]);
    }
    return result;
}

std::vector<float> ObjectSlider::get_v() const {
    std::vector<float> result;
    result.reserve(sliders.size() * 3);
    for (const auto& slider : sliders) {
        result.push_back(slider->v[0]);
        result.push_back(slider->v[1]);
        result.push_back(slider->v[2]);
    }
    return result;
}

std::vector<float> ObjectSlider::get_radius() const {
    std::vector<float> result;
    result.reserve(sliders.size());
    for (const auto& slider : sliders) {
        result.push_back(slider->radius);
    }
    return result;
}

void ObjectSlider::remove(size_t index) {
    if (index >= sliders.size()) {
        throw std::out_of_range("Index out of range");
    }
    sliders.erase(sliders.begin() + index);
}

void ObjectSlider::clear() {
    sliders.clear();
}