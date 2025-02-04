#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "viewer/viewer.h"
#include "simulation/quasi_state_sim.h"
#include "simulation/object_slider.h"
#include "simulation/object_pusher.h"
#include "simulation/param_function.h"
#include "diagram/diagram_all.h"


#include <thread>

namespace py = pybind11;

class PySimulationViewer {
public:
    PySimulationViewer(
        int width = 1600, 
        int height = 1600, 
        float scale = 400.0f,
        bool grid = true,
        bool visualise = true,
        bool move_to_target = true,
        bool show_closest_point = true
        ) : viewer(width, height, scale, grid, visualise),
            pushers(3, 120.0f, "superellipse", { {"a", 0.015f}, {"b", 0.03f}, {"n", 10} }, 0.10f, 0.185f, 0.04f, 0.0f, -1.2f, 0.0f),
            param(std::make_shared<ParamFunction>(sliders, pushers, obstacles)),
            visualise(visualise), 
            show_closest_point(show_closest_point),
            move_to_target(move_to_target) {
        viewer.setGridSpacing(0.1f);
        reset();
    }

    void reset() {
        reset(
            {
                std::make_tuple("circle", std::vector<float>{0.0, -0.5, 0.0, 0.45}),
                std::make_tuple("circle", std::vector<float>{0.5, 0.3, 0.0, 0.45}),
                std::make_tuple("circle", std::vector<float>{-0.5, 0.3, 0.0, 0.45}),
                std::make_tuple("circle", std::vector<float>{0.0, 1.1, 0.0, 0.45}),
                std::make_tuple("circle", std::vector<float>{1.0, 1.1, 0.0, 0.45}),
                std::make_tuple("circle", std::vector<float>{-1.0, 1.1, 0.0, 0.45})
            },
            {
                3, 120.0f, "superellipse", {{"a", 0.015f}, {"b", 0.03f}, {"n", 10}}, 0.10f, 0.185f, 0.04f, 0.0f, -1.2f, 0.0f
            }
        );
    }

    void reset(
        std::vector<std::tuple<std::string, std::vector<float>>> slider_inputs,
        std::tuple<int, float, std::string, std::map<std::string, float>, float, float, float, float, float, float> pusher_input
    ) {
        sliders.clear();
        pushers.clear();
        param.reset();

        for (const auto& slider : slider_inputs) {
            add_slider(std::get<0>(slider), std::get<1>(slider));
        }

        pushers = ObjectPusher(
            std::get<0>(pusher_input), std::get<1>(pusher_input), std::get<2>(pusher_input),
            std::get<3>(pusher_input), std::get<4>(pusher_input), std::get<5>(pusher_input), 
            std::get<6>(pusher_input), std::get<7>(pusher_input), std::get<8>(pusher_input), std::get<9>(pusher_input)
        );

        param = std::make_shared<ParamFunction>(sliders, pushers, obstacles);
        
        // Initialize QuasiStateSim
        sim = std::make_unique<QuasiStateSim>(
            100,
            0.05,
            param,
            false
        );

        viewer.addDiagram(pushers.get_pushers(), "red");
        viewer.addDiagram(sliders.get_sliders(), "blue");
    }

    void add_slider(std::string type, std::vector<float> param) {
        if (type == "circle") sliders.add(std::make_unique<Circle>(param[0], param[1], param[2], param[3]));
        else if (type == "ellipse") sliders.add(std::make_unique<Ellipse>(param[0], param[1], param[2], param[3], param[4]));
        else if (type == "superellipse") sliders.add(std::make_unique<SuperEllipse>(param[0], param[1], param[2], param[3], param[4], param[5]));
    }

    void run(const std::vector<float>& u_input) {

        param->update_param();

        if (!sim) {
            throw std::runtime_error("Simulation not initialized. Call reset() first.");
        }
        Eigen::Matrix2f _rot;
        if (move_to_target) {
            float cos_r = std::cos(pushers.get_rot());
            float sin_r = std::sin(pushers.get_rot());

            _rot << -sin_r, -cos_r,
                    cos_r, -sin_r;
        } else {
            _rot << 1, 0,
                    0, 1;
        }

        // 입력 벡터 u_input 변환
        Eigen::VectorXf u = Eigen::Map<const Eigen::VectorXf>(u_input.data(), u_input.size());

        Eigen::VectorXf transformed_u(4);
        transformed_u.head<2>() = _rot * u.head<2>();  // 회전 적용
        transformed_u(2) = u(2);
        transformed_u(3) = u(3);

        // 시뮬레이션 실행
        auto ans = sim->run(transformed_u);

        // 결과 가져오기
        std::vector<float> qs = std::get<0>(ans);
        std::vector<float> qp = std::get<1>(ans);

        // 이전 상태 저장 후 업데이트
        std::vector<float> qs_diff = substract_vectors(qs, sliders.get_q());
        std::vector<float> qp_diff = substract_vectors(qp, pushers.q);

        // 새로운 상태 적용
        sliders.apply_v(qs_diff);
        sliders.apply_q(qs);
        pushers.apply_v(qp_diff);
        pushers.apply_q(qp);
    }

    void render() {
        if (!show_closest_point){   
            viewer.render();
        }
        else{
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
                }
            }
            viewer.render(points, arrows);
        }
    }

    void get_state() {
    }

    float angle(const std::array<float, 2>& v) {
        return std::atan2(v[1], v[0]);
    }

    std::vector<float> substract_vectors(const std::vector<float>& a, const std::vector<float>& b) {
        if (a.size() != b.size()) {
            throw std::invalid_argument("Vector sizes must match for addition.");
        }

        std::vector<float> result(a.size());
        std::transform(a.begin(), a.end(), b.begin(), result.begin(), std::minus<float>());
        return result;
    }

private:
    SimulationViewer viewer;
    ObjectSlider sliders;
    ObjectPusher pushers;
    ObjectSlider obstacles;
    std::shared_ptr<ParamFunction> param;
    std::unique_ptr<QuasiStateSim> sim;
    bool visualise;
    bool show_closest_point;
    bool move_to_target;

};

PYBIND11_MODULE(quasi_static_push, m) {
    m.doc() = "Quasi-static push simulation module";

    py::class_<PySimulationViewer>(m, "SimulationViewer")
        .def(py::init<int, int, float, bool, bool, bool, bool>(),
             py::arg("width") = 1600,
             py::arg("height") = 1600,
             py::arg("scale") = 400.0f,
             py::arg("grid") = true,
             py::arg("visualise") = true,
             py::arg("move_to_target") = true,
             py::arg("show_closest_point") = true)

        .def("reset", py::overload_cast<>(&PySimulationViewer::reset))
        .def("reset", py::overload_cast<std::vector<std::tuple<std::string, std::vector<float>>>, std::tuple<int, float, std::string, std::map<std::string, float>, float, float, float, float, float, float>>(&PySimulationViewer::reset))
        .def("run", &PySimulationViewer::run)
        .def("render", &PySimulationViewer::render);
}
