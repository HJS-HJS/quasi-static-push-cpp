#include "diagram/diagram_all.h"
#include "viewer/viewer.h"
#include "simulation/object_slider.h"
#include "simulation/object_pusher.h"
// #include "simulation/param_function.h"
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <chrono>
#include <Eigen/Dense>

std::vector<float> add_vectors(const std::vector<float>& a, const std::vector<float>& b) {
    if (a.size() != b.size()) {
        throw std::invalid_argument("Vector sizes must match for addition.");
    }

    std::vector<float> result(a.size());
    std::transform(a.begin(), a.end(), b.begin(), result.begin(), std::plus<float>());
    return result;
}

std::array<float, 4> add_vector_array(const std::vector<float>& vec, const std::array<float, 4>& arr) {
    if (vec.size() != 4) {
        throw std::invalid_argument("Vector and array must have the same size");
    }
    std::array<float, 4> result;
    for (size_t i = 0; i < 4; ++i) {
        result[i] = vec[i] + arr[i];
    }
    return result;
}

float angle(const std::array<float, 2>& v) {
    return std::atan2(v[1], v[0]);
}


int main() {
    try {
        // Initialize random seed
        std::srand(static_cast<unsigned>(std::time(0)));

        // Create viewer
        SimulationViewer viewer(1600, 1600, 400.0f, true, true);
        viewer.setGridSpacing(0.1f);

        // Initialize sliders
        ObjectSlider sliders;

        sliders.add(std::make_unique<Circle>(0.0, -0.5, 0.0, 0.45));
        sliders.add(std::make_unique<Circle>(0.5, 0.3, 0.0, 0.4));
        sliders.add(std::make_unique<Circle>(-0.5, 0.3, 0.0, 0.45));
        sliders.add(std::make_unique<Circle>(0.0, 1.1, 0.0, 0.4));
        sliders.add(std::make_unique<Circle>(1.0, 1.1, 0.0, 0.45));
        sliders.add(std::make_unique<Circle>(-1.0, 1.1, 0.0, 0.45));

        // Initialize pushers
        // ObjectPusher pushers(3, 120.0f, "superellipse", {{"a", 0.015f}, {"b", 0.03f}, {"n", 10}}, 0.10f, 0.185f, 0.04f, 0.0f, -1.2f, 3.141592f);
        ObjectPusher pushers(3, 120.0f, "superellipse", {{"a", 0.015f}, {"b", 0.03f}, {"n", 10}}, 0.10f, 0.185f, 0.04f, 0.0f, -1.2f, 0.0f);

        ObjectSlider empty;
        // ParamFunction param(sliders, pushers, empty);

        // std::vector<float> move = {
        //         0.0f, 0.0f, 0.002f,
        //         0.0f, 0.0f, -0.001f,
        //         0.0f, 0.0f, -0.005f,
        //         0.0f, 0.0f, 0.005f,
        // };

        std::vector<float> move_pusher = {0.0f, 0.0f, 0.0f, 0.01f};

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
            // param.update_param();
            auto end = std::chrono::high_resolution_clock::now();
            // Update positions
            pushers.apply_q(add_vector_array(move_pusher, pushers.q));
            // sliders.apply_q(add_vectors(move, sliders.get_q()));

            auto end2 = std::chrono::high_resolution_clock::now();
            // Calculate collision data and render points/arrows
            std::vector<std::vector<float>> points;
            std::vector<std::tuple<float, float, float, float>> arrows;

            for (const auto& pusher : pushers) {
                for (const auto& slider : sliders) {
                    auto collision_data = pusher->cal_collision_data(*slider);
                    points.push_back({pusher->point(collision_data[0])[0], pusher->point(collision_data[0])[1]});
                    points.push_back({slider->point(collision_data[1])[0], slider->point(collision_data[1])[1]});

                    arrows.push_back({pusher->point(collision_data[0])[0], pusher->point(collision_data[0])[1], angle(pusher->tangentVector(collision_data[0])), 0.3f});
                    arrows.push_back({pusher->point(collision_data[0])[0], pusher->point(collision_data[0])[1], angle(pusher->normalVector(collision_data[0])),  0.3f});
                    arrows.push_back({slider->point(collision_data[1])[0], slider->point(collision_data[1])[1], angle(slider->tangentVector(collision_data[1])), 0.3f});
                    arrows.push_back({slider->point(collision_data[1])[0], slider->point(collision_data[1])[1], angle(slider->normalVector(collision_data[1])),  0.3f});

                    // std::cout << "Collision between Pusher and Slider: "
                    //         << "Angle1 = " << collision_data[0] * 180 / M_PI << ", "
                    //         << "Angle2 = " << collision_data[1] * 180 / M_PI << ", "
                    //         << "Distance = " << collision_data[2] << std::endl;
                }
            }
            auto end3 = std::chrono::high_resolution_clock::now();
            // Render diagrams
            // viewer.render();
            viewer.render(points, arrows);

            auto dt = pushers.pusher_dv(0.0001);
            for(auto vec : dt){
                for(auto vec2 : vec){
                    std::cout << vec2[0] << " " << vec2[1] << " " << vec2[2] << std::endl;
                }
                std::cout << "\n";
            }

            auto end4 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end4 - start;
            std::chrono::duration<double> elapsed1 = end1 - start;
            std::chrono::duration<double> elapsed2 = end - end1;
            std::chrono::duration<double> elapsed3 = end2 - end;
            std::chrono::duration<double> elapsed4 = end3 - end2;
            std::chrono::duration<double> elapsed5 = end4 - end3;
            std::cout << "Total time spent: " << 1/elapsed.count() << "s" << std::endl;
            std::cout << "\tTime spent: " << elapsed1.count() << "s" << std::endl;
            std::cout << "\tTime spent: " << elapsed2.count() << "s" << std::endl;
            std::cout << "\tTime spent: " << elapsed3.count() << "s" << std::endl;
            std::cout << "\tTime spent: " << elapsed4.count() << "s\tmain" << std::endl;
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
