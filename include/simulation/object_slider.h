#ifndef OBJECT_SLIDER_H
#define OBJECT_SLIDER_H

#include <vector>
#include <memory>
#include <stdexcept>
#include <Eigen/Dense>
#include "diagram/diagram_all.h"

class ObjectSlider {
public:
    // Constructor
    ObjectSlider() = default;

    // Diagram list
    std::vector<std::unique_ptr<Diagram>> sliders;

    // Append a Diagram to the slider list
    void add(std::unique_ptr<Diagram> item);

    // Append a raw Diagram object (overload for convenience)
    void add(Diagram* item);

    // Get every slider as a vector
    const std::vector<std::unique_ptr<Diagram>>& get_sliders() const;

    // Get the number of sliders
    size_t size() const;

    // Access a specific slider by index
    Diagram* operator[](size_t i);

    // Iterator support
    auto begin() -> decltype(sliders.begin()) {
        return sliders.begin();
    }

    auto end() -> decltype(sliders.end()) {
        return sliders.end();
    }

    auto begin() const -> decltype(sliders.begin()) {
        return sliders.begin();
    }

    auto end() const -> decltype(sliders.end()) {
        return sliders.end();
    }

    // Apply q to all sliders
    void apply_q(const Eigen::VectorXf& q);

    // Apply v to all sliders
    void apply_v(const Eigen::VectorXf& v);

    // Get combined q of all sliders
    Eigen::VectorXf get_q() const;

    // Get combined v of all sliders
    Eigen::VectorXf get_v() const;

    // Get radii of all sliders
    Eigen::VectorXf get_radius() const;

    // Remove a slider by index
    void remove(size_t index);

    // Destructor
    ~ObjectSlider() = default;

private:
};

#endif // OBJECT_SLIDER_H
