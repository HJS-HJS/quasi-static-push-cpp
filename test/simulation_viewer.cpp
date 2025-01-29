#include "diagram/diagram_all.h"
#include "viewer/viewer.h"
#include "simulation/object_slider.h"
#include "simulation/object_pusher.h"
#include "simulation/param_function.h"
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <chrono>
#include <Eigen/Dense>

Eigen::VectorXf add_vectors(const Eigen::VectorXf& a, const Eigen::VectorXf& b) {
    if (a.size() != b.size()) {
        throw std::invalid_argument("Vector sizes must match for addition.");
    }
    return a + b;
}

float angle(const Eigen::VectorXf& v) {
    return std::atan2(v[1], v[0]);
}


int main() {
    try {
        // Initialize random seed
        std::srand(static_cast<unsigned>(std::time(0)));

        // Create viewer
        SimulationViewer viewer(1600, 1600, 1600.0f, true, true);
        viewer.setGridSpacing(0.1f);

        // Initialize sliders
        ObjectSlider sliders;
        sliders.add(std::make_unique<Circle>(0.5, 0.5, 0.0, 0.25));
        sliders.add(std::make_unique<SmoothRPolygon>(-0.5, 0.5, 0.0, 0.25, 5));
        sliders.add(std::make_unique<SuperEllipse>(-0.5, -0.5, 0, 0.25, 0.1, 7));
        sliders.add(std::make_unique<Ellipse>(0.5f, -0.5f, 1.0f, 0.25f, 0.125f));

        // Initialize pushers
        ObjectPusher pushers(3, 120.0f, "superellipse", {{"a", 0.015f}, {"b", 0.03f}, {"n", 7}}, 0.15f, 0.17f, 0.04f, 0.0f, 0.0f, 0.0f);

        ObjectSlider empty;
        ParamFunction param(sliders, pushers, empty);

        Eigen::VectorXf move(12);
        move << 0.0f, 0.0f, 0.002f,
                0.0f, 0.0f, -0.001f,
                0.0f, 0.0f, -0.005f,
                0.0f, 0.0f, 0.005f;

        Eigen::Vector4f move_pusher(0.0f, 0.0f, 0.0f, 0.05f);

        // Add diagrams to viewer
        viewer.addDiagram(pushers.get_pushers(), "red");
        viewer.addDiagram(sliders.get_sliders(), "blue");

        // Main loop
        SDL_Event event;
        bool running = true;

        while (running) {
            auto start = std::chrono::high_resolution_clock::now();
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
                    running = false;
                }
                if (event.type == SDL_KEYDOWN) {
                    switch (event.key.keysym.sym) {
                        case SDLK_w: move_pusher[1] = 0.01f; break;  // Move up (y-axis)
                        case SDLK_s: move_pusher[1] = -0.01f; break; // Move down (y-axis)
                        case SDLK_a: move_pusher[0] = -0.01f; break; // Move left (x-axis)
                        case SDLK_d: move_pusher[0] = 0.01f; break;  // Move right (x-axis)
                        case SDLK_q: move_pusher[2] = 0.01f; break;  // Rotate counterclockwise
                        case SDLK_e: move_pusher[2] = -0.01f; break; // Rotate clockwise
                    }
                } else if (event.type == SDL_KEYUP) {
                    switch (event.key.keysym.sym) {
                        case SDLK_w:
                        case SDLK_s: move_pusher[1] = 0.0f; break;
                        case SDLK_a:
                        case SDLK_d: move_pusher[0] = 0.0f; break;
                        case SDLK_q:
                        case SDLK_e: move_pusher[2] = 0.0f; break;
                    }
                }
            }

            auto end1 = std::chrono::high_resolution_clock::now();
            param.update_param();
            auto end = std::chrono::high_resolution_clock::now();
            // Update positions
            pushers.apply_q(add_vectors(move_pusher, pushers.q));
            sliders.apply_q(add_vectors(move, sliders.get_q()));

            auto end2 = std::chrono::high_resolution_clock::now();
            // Calculate collision data and render points/arrows
            std::vector<std::vector<float>> points;
            std::vector<std::tuple<float, float, float, float>> arrows;

            for (const auto& pusher : pushers) {
                for (const auto& slider : sliders) {
                    auto collision_data = pusher->cal_collision_data(*slider);
                    points.push_back({pusher->point(collision_data[0])[0], pusher->point(collision_data[0])[1]});
                    points.push_back({slider->point(collision_data[1])[0], slider->point(collision_data[1])[1]});

                    arrows.push_back({pusher->point(collision_data[0])[0], pusher->point(collision_data[0])[1], angle(pusher->tangentVector(collision_data[0])), 0.03f});
                    arrows.push_back({pusher->point(collision_data[0])[0], pusher->point(collision_data[0])[1], angle(pusher->normalVector(collision_data[0])),  0.03f});
                    arrows.push_back({slider->point(collision_data[1])[0], slider->point(collision_data[1])[1], angle(slider->tangentVector(collision_data[1])), 0.03f});
                    arrows.push_back({slider->point(collision_data[1])[0], slider->point(collision_data[1])[1], angle(slider->normalVector(collision_data[1])),  0.03f});

                    // std::cout << "Collision between Pusher and Slider: "
                    //         << "Angle1 = " << collision_data[0] * 180 / M_PI << ", "
                    //         << "Angle2 = " << collision_data[1] * 180 / M_PI << ", "
                    //         << "Distance = " << collision_data[2] << std::endl;
                }
            }
            auto end3 = std::chrono::high_resolution_clock::now();
            // Render diagrams
            viewer.render(points, arrows);

            auto end4 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed1 = end1 - start;
            std::chrono::duration<double> elapsed2 = end - end1;
            std::chrono::duration<double> elapsed3 = end2 - end;
            std::chrono::duration<double> elapsed4 = end3 - end2;
            std::chrono::duration<double> elapsed5 = end4 - end3;
            std::cout << "\n\tTime spent: " << elapsed1.count() << "s" << std::endl;
            std::cout << "\tTime spent: " << elapsed2.count() << "s" << std::endl;
            std::cout << "\tTime spent: " << elapsed3.count() << "s" << std::endl;
            std::cout << "\tTime spent: " << elapsed4.count() << "s" << std::endl;
            std::cout << "\tTime spent: " << elapsed5.count() << "s" << std::endl;

            // Delay for smooth animation
            SDL_Delay(100);
        }

        std::cout << "Simulation completed." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
