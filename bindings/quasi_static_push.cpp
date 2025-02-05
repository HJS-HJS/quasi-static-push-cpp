#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "viewer/viewer.h"
#include "viewer/recorder.h"
#include "simulation/quasi_state_sim.h"
#include "simulation/object_slider.h"
#include "simulation/object_pusher.h"
#include "simulation/param_function.h"
#include "diagram/diagram_all.h"

namespace py = pybind11;

class PySimulationViewer {
public:
    PySimulationViewer(
        int window_width = 1600, 
        int window_height = 1600, 
        float scale = 400.0f,
        float tableWidth = 2.0,
        float tableHeight = 2.0,
        float frame_rate = 100,
        int frame_skip = 10,
        bool grid = true,
        bool visualise = true,
        bool move_to_target = true,
        bool show_closest_point = true,
        std::string state = "Image",
        bool recording_enabled = false,
        std::string recording_path = "recordings"
        ) : viewer(window_width, window_height, scale, tableWidth, tableHeight, grid, visualise),
            pushers(3, 120.0f, "superellipse", { {"a", 0.015f}, {"b", 0.03f}, {"n", 10} }, 0.10f, 0.185f, 0.04f, 0.0f, -1.2f, 0.0f),
            param(std::make_shared<ParamFunction>(sliders, pushers, obstacles)),
            table_limit(std::array<float, 2>{tableWidth/2, tableHeight/2}),
            frame_rate(frame_rate),
            frame_skip(frame_skip),
            visualise(visualise), 
            show_closest_point(show_closest_point),
            move_to_target(move_to_target),
            state(state),
            recording_enabled(recording_enabled),
            recording_path(recording_path) {
        viewer.setGridSpacing(0.1f);
        // reset();
        if (recording_enabled) {
            recorder = std::make_unique<Recorder>(recording_path, 1.0 / frame_rate * frame_skip, window_width, window_height);
        }
    }

    ~PySimulationViewer() {
        if (recording_enabled && recorder) {
            recorder->stopRecording();
        }
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
            },
            2.0,
            2.0
        );
    }

    void reset(
        std::vector<std::tuple<std::string, std::vector<float>>> slider_inputs,
        std::tuple<int, float, std::string, std::map<std::string, float>, float, float, float, float, float, float> pusher_input,
        float newtableWidth,
        float newtableHeight
    ) {
        mode = -1;
        sliders.clear();
        pushers.clear();
        param.reset();

        for (const auto& slider : slider_inputs) {
            addSlider_(std::get<0>(slider), std::get<1>(slider));
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
            param
        );

        viewer.reset(newtableWidth, newtableHeight, true);
        viewer.addDiagram(pushers.get_pushers(), "pink");
        viewer.addDiagram(sliders.get_sliders(), "blue");
        viewer.changeDiagramColor(sliders.get_sliders().begin()->get(), "green");
        
        table_limit = std::array<float, 2>{newtableWidth / 2, newtableHeight / 2};

        stopRecording();
        startRecording();
    }

    py::tuple run(const std::vector<float>& u_input) {
        simulate_(u_input);

        bool condition = isDishOut_();  // 특정 조건을 확인하는 함수
        py::object image_state_ = getImageState();
        py::object linear_state_ = getLinearState();

        if (recording_enabled && recorder) {
            recorder->saveFrame(SDL_SurfaceToMat(viewer.getRenderedImage()), {1.0f, 2.0f}, {3.0f, 4.0f, 5.0f}, {6.0f, 7.0f, 8.0f});
            std::cout<<"record frame to video" <<std::endl;
        }


        if (state == "Image") {
            return py::make_tuple(condition, image_state_);
        } 
        else if (state == "Linear") {
            return py::make_tuple(condition, linear_state_);
        }
        throw std::runtime_error("Invalid state type");
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

                    arrows.push_back({pusher->point(collision_data[0])[0], pusher->point(collision_data[0])[1], calculateAngle_(pusher->tangentVector(collision_data[0])), 0.3f});
                    arrows.push_back({pusher->point(collision_data[0])[0], pusher->point(collision_data[0])[1], calculateAngle_(pusher->normalVector(collision_data[0])),  0.3f});
                    arrows.push_back({slider->point(collision_data[1])[0], slider->point(collision_data[1])[1], calculateAngle_(slider->tangentVector(collision_data[1])), 0.3f});
                    arrows.push_back({slider->point(collision_data[1])[0], slider->point(collision_data[1])[1], calculateAngle_(slider->normalVector(collision_data[1])),  0.3f});
                }
            }
            viewer.render(points, arrows);
        }
    }

private:
    SimulationViewer viewer;
    ObjectSlider sliders;
    ObjectPusher pushers;
    ObjectSlider obstacles;
    std::shared_ptr<ParamFunction> param;
    std::unique_ptr<QuasiStateSim> sim;
    std::array<float, 2> table_limit;
    float frame_rate;
    int frame_skip;
    bool visualise;
    bool show_closest_point;
    bool move_to_target;
    std::string state;
    int mode;

    // Recorder
    bool recording_enabled;
    std::string recording_path;
    std::unique_ptr<Recorder> recorder;

    bool isDishOut_() {
    return std::any_of(sliders.begin(), sliders.end(), [this](const auto& dish) {
        return std::abs(dish->q[0]) > table_limit[0] || std::abs(dish->q[1]) > table_limit[1];
        });
    }

    bool simulate_(const std::vector<float>& u_input) {
        if (!sim) {
            throw std::runtime_error("Simulation not initialized. Call reset() first.");
        }

        bool success = true;

        if(u_input.back() > 0.5f && mode < 0){
            param->update_param();
            if ((param->phi.array() < 0).any()){
                success = false;
            }
            else{
                mode = 0;
                viewer.changeDiagramColor(pushers.get_pushers(), "red");
            }
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

        for (int i = 0; i < frame_skip; i++){
            // 시뮬레이션 준비
            param->update_param();
            // 시뮬레이션 실행

            if(mode == -1){
                auto ans = sim->run(transformed_u / frame_rate, true);

                // 결과 가져오기
                std::vector<float> qp = std::get<1>(ans);

                // 이전 상태 저장 후 업데이트
                std::vector<float> qp_diff = substractVectors_(qp, pushers.q);

                // 새로운 상태 적용
                pushers.apply_v(qp_diff);
                pushers.apply_q(qp);    
            }
            else{
                auto ans = sim->run(transformed_u / frame_rate, false);

                // 결과 가져오기
                std::vector<float> qs = std::get<0>(ans);
                std::vector<float> qp = std::get<1>(ans);

                // 이전 상태 저장 후 업데이트
                std::vector<float> qs_diff = substractVectors_(qs, sliders.get_q());
                std::vector<float> qp_diff = substractVectors_(qp, pushers.q);

                // 새로운 상태 적용
                sliders.apply_v(qs_diff);
                sliders.apply_q(qs);
                pushers.apply_v(qp_diff);
                pushers.apply_q(qp);    
            }
        }

        return success;
    }

    py::object getImageState() {
        auto surface = viewer.getRenderedImage();
        return py::array_t<uint8_t>(
            {surface->h, surface->w, 4}, 
            {surface->pitch, 4, 1},
            static_cast<uint8_t*>(surface->pixels), 
            py::capsule(surface, [](void *p) { SDL_FreeSurface(static_cast<SDL_Surface*>(p)); })
        );
    } 

    py::object getLinearState() {
        std::vector<float> linear_state = {0.1f, 0.2f, 0.3f, 0.4f};  // 예제 데이터
        return py::array_t<float>(
            {linear_state.size()},  // Shape (1D array)
            {sizeof(float)},        // Strides
            linear_state.data()     // Data pointer
        );
    }

    void addSlider_(std::string type, std::vector<float> param) {
        if (type == "circle") sliders.add(std::make_unique<Circle>(param[0], param[1], param[2], param[3]));
        else if (type == "ellipse") sliders.add(std::make_unique<Ellipse>(param[0], param[1], param[2], param[3], param[4]));
        else if (type == "superellipse") sliders.add(std::make_unique<SuperEllipse>(param[0], param[1], param[2], param[3], param[4], param[5]));
    }

    float calculateAngle_(const std::array<float, 2>& v) {
        return std::atan2(v[1], v[0]);
    }

    std::vector<float> substractVectors_(const std::vector<float>& a, const std::vector<float>& b) {
        if (a.size() != b.size()) {
            throw std::invalid_argument("Vector sizes must match for addition.");
        }

        std::vector<float> result(a.size());
        std::transform(a.begin(), a.end(), b.begin(), result.begin(), std::minus<float>());
        return result;
    }

    cv::Mat SDL_SurfaceToMat(SDL_Surface* surface) {
        if (!surface) {
            throw std::runtime_error("SDL_Surface is null!");
        }

        // SDL_Surface에서 픽셀 데이터를 가져오기
        int width = surface->w;
        int height = surface->h;
        int channels = surface->format->BytesPerPixel;

        // OpenCV의 데이터 타입 결정 (RGB or RGBA)
        int type = (channels == 4) ? CV_8UC4 : CV_8UC3;

        // SDL_Surface의 픽셀 데이터를 OpenCV의 cv::Mat으로 복사
        cv::Mat mat(height, width, type, surface->pixels);

        // SDL은 픽셀 데이터를 아래에서 위로 저장하므로, OpenCV에서 뒤집기 필요
        cv::Mat flipped;
        cv::flip(mat, flipped, 0);

        return flipped;
    }

    void stopRecording() {
        if (recording_enabled && recorder) {
            recorder->stopRecording();
        }
    }

    void startRecording() {
        if (recording_enabled) {
            recorder->startRecording();
        }
    }

};

PYBIND11_MODULE(quasi_static_push, m) {
    m.doc() = "Quasi-static push simulation module";

    py::class_<PySimulationViewer>(m, "SimulationViewer",
        R"pbdoc(
            SimulationViewer for quasi-static push simulation.

            Parameters:
            - window_width (int): Width of the simulation window (default: 1600).
            - window_height (int): Height of the simulation window (default: 1600).
            - scale (float): Scale factor for visualization (default: 400.0).
            - tableWidth (float): Table width in meters (default: 2.0).
            - tableHeight (float): Table height in meters (default: 2.0).
            - frame_rate (float): frame rate of the simulation [hz] (default: 100.0).
            - frame_skip (float): continous frame of the simulation [frame] (default: 10.0).
            - grid (bool): Show grid in visualization (default: True).
            - visualise (bool): Enable visualization (default: True).
            - move_to_target (bool): Move to target position (default: True).
            - show_closest_point (bool): Highlight closest points in visualization (default: True).
            - state (str): Type of state to return from `get_state()` and `run()`. Options:
                - "Image": Returns an (H, W, 4) RGBA NumPy array.
                - "Linear": Returns a 1D NumPy array of floats.
                - "Gray": Returns an (H, W) grayscale NumPy array.
            
            Methods:
            --------
            reset()
                Reset the simulation to the initial state.

            reset(slider_inputs, pusher_input, newtableWidth, newtableHeight)
                Reset the simulation with custom parameters.

                Parameters:
                - slider_inputs (List[Tuple[str, List[float]]]): List of sliders with their types and parameters.
                - pusher_input (Tuple[int, float, str, Dict[str, float], float, float, float, float, float, float]): 
                Configuration of the pusher.
                - newtableWidth (float): New table width.
                - newtableHeight (float): New table height.

            run(u_input: List[float]) -> Tuple[bool, Union[numpy.ndarray, List[float]]]
                Run the simulation and return a tuple `(bool, state)`.

                - The first element (bool) indicates whether a specific condition is met (e.g., object out of bounds).
                - The second element (state) depends on the `state` type:
                    - "Image": (H, W, 4) RGBA NumPy array.
                    - "Linear": 1D NumPy array of floats.
                    - "Gray": (H, W) grayscale NumPy array.

            get_state() -> Union[numpy.ndarray, List[float]]
                Get the simulation state based on the selected `state` type.

                - "Image": Returns an (H, W, 4) RGBA NumPy array.
                - "Linear": Returns a 1D NumPy array of floats.
                - "Gray": Returns an (H, W) grayscale NumPy array.
        )pbdoc")
        .def(py::init<int, int, float, float, float, float, int, bool, bool, bool, bool, std::string, bool, std::string>(),
             py::arg("window_width") = 1600,
             py::arg("window_height") = 1600,
             py::arg("scale") = 400.0f,
             py::arg("tableWidth") = 2.0f,
             py::arg("tableHeight") = 2.0f,
             py::arg("frame_rate") = 100.0f,
             py::arg("frame_skip") = 10,
             py::arg("grid") = true,
             py::arg("visualise") = true,
             py::arg("move_to_target") = true,
             py::arg("show_closest_point") = true,
             py::arg("state") = "Image",
             py::arg("recording_enabled") = false,
             py::arg("recording_path") = "recordings")

        .def("reset", py::overload_cast<>(&PySimulationViewer::reset))
        .def("reset", py::overload_cast<
                std::vector<std::tuple<std::string, std::vector<float>>>,
                std::tuple<int, float, std::string, std::map<std::string, float>, float, float, float, float, float, float>,
                float,
                float
            >(&PySimulationViewer::reset),
            py::arg("slider_inputs") = std::vector<std::tuple<std::string, std::vector<float>>>{
                {"circle", {0.0, -0.5, 0.0, 0.45}},
                {"circle", {0.5, 0.3, 0.0, 0.45}},
                {"circle", {-0.5, 0.3, 0.0, 0.45}},
                {"circle", {0.0, 1.1, 0.0, 0.45}},
                {"circle", {1.0, 1.1, 0.0, 0.45}},
                {"circle", {-1.0, 1.1, 0.0, 0.45}}
            },
            py::arg("pusher_input") = std::tuple<int, float, std::string, std::map<std::string, float>, float, float, float, float, float, float>{
                3, 120.0f, "superellipse", {{"a", 0.015f}, {"b", 0.03f}, {"n", 10}}, 
                0.10f, 0.185f, 0.04f, 0.0f, -1.2f, 0.0f
            },
            py::arg("newtableWidth") = 2.0f,
            py::arg("newtableHeight") = 2.0f
        )
        .def("render", &PySimulationViewer::render)
        .def("run", &PySimulationViewer::run, R"pbdoc(
            Run the simulation and return a tuple (bool, state).

            - The first element (bool) indicates a certain condition (e.g., if an object is out of bounds).
            - The second element (state) depends on the `state` type:
                - "Image": (H, W, 4) RGBA NumPy array.
                - "Linear": NumPy array of floats.
        )pbdoc");
}
