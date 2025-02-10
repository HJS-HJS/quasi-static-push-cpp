#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "utils/viewer.h"
#include "utils/recorder.h"
#include "utils/player.h"
#include "simulation/quasi_state_sim.h"
#include "simulation/object_slider.h"
#include "simulation/object_pusher.h"
#include "simulation/param_function.h"
#include "diagram/diagram_all.h"

namespace py = pybind11;

/**
 * @file quasi_static_push.cpp
 * @brief Python bindings for the quasi-static push simulation using pybind11.
 *
 * This module integrates a physics-based simulation with a visualization engine.
 * It allows users to configure the simulation, execute a step, and retrieve the state.
 * The main simulation class, `PySimulationViewer`, manages interactions with objects
 * such as pushers and sliders, tracks simulation progress, and handles visualization.
 *
 * Features:
 * - Object pushing and grasping simulation.
 * - Detection of simulation termination conditions.
 * - Image rendering and state retrieval.
 * - Support for recording simulation frames.
 */

// Enum representing the reasons why a simulation might end
enum SimulationDoneReason {
    DONE_NONE = 0,          // Default: No termination condition met
    DONE_FALL_OUT = 1,      // The object has fallen outside the table bounds
    DONE_GRASP_SUCCESS = 2, // Successful grasp detected
    DONE_GRASP_FAILED = 4   // Grasp failed due to insufficient grip
};

enum GripperMotion {
    MOVE_XY = 0,          // Default: No termination condition met
    MOVE_TO_TARGET = 1,   // The object has fallen outside the table bounds
    MOVE_FORWARD = 2,     // Gripper move its forward direction
};

// Structure to store the simulation results
struct __attribute__ ((visibility("hidden"))) SimulationResult {
    int done;                       // Bitwise OR of SimulationDoneReason values
    std::vector<std::string> reasons; // List of reasons why the simulation ended
    int mode;                       // Current simulation mode
    py::object image_state;         // Rendered image of the simulation
    py::object pusher_state;        // State of the pusher object
    py::object slider_state;        // State of the slider object
};


// Main simulation class that integrates viewer, physics simulation, and interaction
class PySimulationViewer {
    /**
     * @brief Constructor for the PySimulationViewer.
     *
     * @param window_width Width of the simulation window.
     * @param window_height Height of the simulation window.
     * @param scale Visualization scale factor.
     * @param tableWidth Width of the simulation table.
     * @param tableHeight Height of the simulation table.
     * @param frame_rate Frame rate of the simulation.
     * @param frame_skip Number of frames to skip during simulation.
     * @param grid Enable or disable grid visualization.
     * @param headless Enable or disable rendering.
     * @param gripper_movement Enable automatic movement to a target.
     * @param show_closest_point Highlight closest contact points in visualization.
     * @param recording_enabled Enable recording of simulation frames.
     * @param recording_path Path to save recorded frames.
     */
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
        float grid_space = 0.1f,
        bool headless = false,
        int gripper_movement = 0,
        bool show_closest_point = false,
        bool recording_enabled = false,
        std::string recording_path = "recordings"
        ) : viewer(window_width, window_height, scale, tableWidth, tableHeight, grid, grid_space, !headless),
            pushers(3, 120.0f, "superellipse", { {"a", 0.015f}, {"b", 0.03f}, {"n", 10} }, 0.10f, 0.185f, 0.04f, 0.0f, -1.2f, 0.0f),
            param(std::make_shared<ParamFunction>(sliders, pushers, obstacles)),
            table_limit(std::array<float, 2>{tableWidth/2, tableHeight/2}),
            frame_rate(frame_rate),
            frame_skip(frame_skip),
            show_closest_point(show_closest_point),
            gripper_movement(gripper_movement),
            recording_enabled(recording_enabled),
            recording_path(recording_path) {
        if (recording_enabled) {
            recorder = std::make_unique<Recorder>(recording_path, 1.0 / frame_rate * frame_skip, window_width, window_height);
            startRecording();
        }
    }

    ~PySimulationViewer() {
        if (recording_enabled && recorder) {
            recorder->stopRecording();
        }
    }

    // Reset the simulation with default parameters
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
        viewer.changeDiagramColor((pushers.get_pushers().end() - 1)->get(), "lightpurple");
        viewer.changeDiagramColor(sliders.get_sliders().begin()->get(), "green");
        
        table_limit = std::array<float, 2>{newtableWidth / 2, newtableHeight / 2};

        stopRecording();
        startRecording();
    }

    /**
     * @brief Runs the simulation for one step.
     *
     * @param u_input Input control vector for the simulation step.
     * @return SimulationResult containing simulation status, rendered image, and state information.
     */
    SimulationResult run(const std::vector<float>& u_input) {

        if(u_input.back() > 0.5f && mode < 0){
            param->update_param();
            if (!(param->phi.array() < 0).any()){
                mode = 0;
                viewer.changeDiagramColor(pushers.get_pushers(), "red");
                viewer.changeDiagramColor((pushers.get_pushers().end() - 1)->get(), "purple");
            }
        }

        Eigen::VectorXf transformed_u = applyGripperMovement(u_input);

        simulate_(transformed_u);
        int condition = isDishOut_(); 
        int grasp = grasp_();

        int done = DONE_NONE;
        std::vector<std::string> reasons;

        if (condition >= 0) {
            done |= DONE_FALL_OUT;
            reasons.push_back("DONE_FALL_OUT");
            // viewer.removeDiagram(sliders[condition]);
            sliders.remove(condition);
        }
        if (grasp > 0) {
            done |= DONE_GRASP_SUCCESS;
            reasons.push_back("DONE_GRASP_SUCCESS");
            // viewer.removeDiagram(sliders[0]);
            sliders.remove(0);
        }
        else if (grasp < 0) {
            done |= DONE_GRASP_FAILED;
            reasons.push_back("DONE_GRASP_FAILED");
        }

        renderViewer_();

        // Record image and data
        if (recording_enabled && recorder) {
            recorder->saveFrame(SDL_SurfaceToMat(viewer.getRenderedImage()), done, reasons, mode, pushers.q, sliders.get_status(), std::vector<float>(transformed_u.data(), transformed_u.data() + transformed_u.size()));
        }
        
        return {
            done,
            reasons,
            mode,
            getImageState(),
            getPusherState(pushers.q),
            getSliderState(sliders.get_status())
        };
    }

    std::tuple<std::array<float, 5>, bool, bool> keyboard_input(){
        return viewer.getKeyboardInput();
    }

    static py::object getPusherState(const std::vector<float>& linear_state) {
        return py::array_t<float>(
            {linear_state.size()},  // Shape (1D array)
            {sizeof(float)},        // Strides
            linear_state.data()     // Data pointer
        );
    }

    static py::object getSliderState(const std::vector<std::vector<float>>& linear_state) {
        py::list py_list;
        for (const auto& row : linear_state) {
            py::list py_row;
            for (float val : row) {
                py_row.append(val);
            }
            py_list.append(py_row);
        }
        return py_list;  // Python 리스트로 반환
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
    bool show_closest_point;
    int gripper_movement;
    int mode;

    // Recorder
    bool recording_enabled;
    std::string recording_path;
    std::unique_ptr<Recorder> recorder;

    int isDishOut_() {
        auto it = std::find_if(sliders.begin(), sliders.end(), [this](const auto& dish) {
            return std::abs(dish->q[0]) > table_limit[0] || std::abs(dish->q[1]) > table_limit[1];
        });

        if (it != sliders.end()) {
            return it - sliders.begin();
        }
        else{
            return -1;
        }
    }

    Eigen::MatrixXf applyGripperMovement(const std::vector<float>& u_input){
        Eigen::Matrix2f rot_;
        if (gripper_movement == MOVE_XY) {
            rot_ << 0, -1,
                    1,  0;
        }
        else if (gripper_movement == MOVE_TO_TARGET) {
            std::vector<float> v_(2);
            std::transform(pushers.q.begin(), pushers.q.begin()+2, sliders[0]->q.begin(), v_.begin(), std::minus<float>());

            float angle_ = std::atan2(v_[1], v_[0]) + M_PI / 2;
            float cos_r = std::cos(angle_);
            float sin_r = std::sin(angle_);

            rot_ << -sin_r, -cos_r,
                     cos_r, -sin_r;
        }
        else if (gripper_movement == MOVE_FORWARD) {
            float cos_r = std::cos(pushers.get_rot());
            float sin_r = std::sin(pushers.get_rot());

            rot_ << -sin_r, -cos_r,
                     cos_r, -sin_r;
        }
        else{
            throw std::invalid_argument("Gripper Movement is not defined");
        }
        
        // 입력 벡터 u_input 변환
        Eigen::VectorXf u = Eigen::Map<const Eigen::VectorXf>(u_input.data(), u_input.size());

        Eigen::VectorXf transformed_u(4);
        transformed_u.head<2>() = rot_ * u.head<2>();  // 회전 적용
        transformed_u(2) = u(2);
        transformed_u(3) = u(3);

        return transformed_u;
    }

    void simulate_(Eigen::VectorXf& u_input) {
        if (!sim) {
            throw std::runtime_error("Simulation not initialized. Call reset() first.");
        }

        for (int i = 0; i < frame_skip; i++){
            // 시뮬레이션 준비
            param->update_param();
            // 시뮬레이션 실행

            if(mode == -1){
                auto ans = sim->run(u_input / frame_rate, true);

                // 결과 가져오기
                std::vector<float> qp = std::get<1>(ans);

                // 이전 상태 저장 후 업데이트
                std::vector<float> qp_diff = substractVectors_(qp, pushers.q);

                // 새로운 상태 적용
                pushers.apply_v(qp_diff);
                pushers.apply_q(qp);    
            }
            else{
                auto ans = sim->run(u_input / frame_rate, false);

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

        return;
    }

    int grasp_() {
        if (mode == 0 && isGraspReady()){
            mode = 1;
            Eigen::VectorXf transformed_u_(4);
            transformed_u_ << 0.0f, 0.0f, 0.0f, -0.5f;
            float width_ = pushers.q[3];
            int finger = pushers.size();
            while (true) {
                param->update_param();

                auto ans = sim->run(transformed_u_ / frame_rate, false);

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

                if ((param->phi.head(finger).array() < 0.015).any() && (param->phi.head(finger).array() != 0).all()){
                    return 1;
                } 
                else if ((width_ - pushers.q[3]) < 0.5 / frame_rate * 0.9){
                    return -1;
                }
                width_ = pushers.q[3];
                renderViewer_();
            }
        }
        return 0;
    }

    bool isGraspReady(){
        if (std::hypot(pushers.q[0] - sliders[0]->q[0], pushers.q[1] - sliders[0]->q[1]) < 0.015){
            return true;
        }
        else{
            return false;
        }
    }

    py::object getImageState() {
        auto surface = viewer.getRenderedImage();
        if (!surface) {
            throw std::runtime_error("Failed to get rendered image!");
        }

        // SDL_Surface → cv::Mat (4채널 RGBA 또는 BGRA)
        cv::Mat frame = SDL_SurfaceToMat(surface);

        std::vector<py::ssize_t> shape = {frame.rows, frame.cols, frame.channels()};
        std::vector<py::ssize_t> strides = {static_cast<py::ssize_t>(frame.step[0]), static_cast<py::ssize_t>(frame.elemSize()), 1};

        return py::array_t<uint8_t>(
            shape,      // shape 정보 전달 (H, W, C)
            strides,    // strides 정보 전달 (row stride, column stride, element stride)
            frame.data  // 데이터 포인터
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

        cv::Mat rgbImage;
        // OpenCV의 데이터 타입 결정 (RGB or RGBA)
        if (channels == 4){
            // SDL_Surface의 픽셀 데이터를 OpenCV의 cv::Mat으로 복사
            cv::Mat mat(height, width, CV_8UC4, surface->pixels);
            cv::cvtColor(mat, rgbImage, cv::COLOR_BGRA2RGB);
        }
        else{
            // SDL_Surface의 픽셀 데이터를 OpenCV의 cv::Mat으로 복사
            cv::Mat mat(height, width, CV_8UC3, surface->pixels);
            cv::cvtColor(mat, rgbImage, cv::COLOR_BGRA2RGB);
        }

        return rgbImage;
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

    void renderViewer_() {
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

                    arrows.push_back({pusher->point(collision_data[0])[0], pusher->point(collision_data[0])[1], calculateAngle_(pusher->tangentVector(collision_data[0])), 0.1f});
                    arrows.push_back({pusher->point(collision_data[0])[0], pusher->point(collision_data[0])[1], calculateAngle_(pusher->normalVector(collision_data[0])),  0.1f});
                    arrows.push_back({slider->point(collision_data[1])[0], slider->point(collision_data[1])[1], calculateAngle_(slider->tangentVector(collision_data[1])), 0.1f});
                    arrows.push_back({slider->point(collision_data[1])[0], slider->point(collision_data[1])[1], calculateAngle_(slider->normalVector(collision_data[1])),  0.1f});
                }
            }
            viewer.render(points, arrows);
        }
    }

};

class PyPlayerIterator {
public:
    PyPlayerIterator(Player& player) : player(player), iter(player.begin()), end_iter(player.end()) {}

    std::tuple<bool, SimulationResult, std::vector<float>> next() {
        if (iter == end_iter) throw py::stop_iteration();

        auto [frame, metadata] = *iter;

        if (frame.empty()) throw py::stop_iteration();  // ✅ 영상이 끝나면 정상 종료

        // ✅ OpenCV Mat → NumPy 변환 (RGB 형식)
        std::vector<py::ssize_t> shape = {frame.rows, frame.cols, frame.channels()};
        std::vector<py::ssize_t> strides = {static_cast<py::ssize_t>(frame.step[0]), static_cast<py::ssize_t>(frame.elemSize()), 1};
        
        py::array_t<uint8_t> numpy_frame(shape, strides, frame.data);

        // ✅ Metadata에서 Simulation 상태 정보 가져오기
        bool new_video = metadata.contains("frame") ? !metadata["frame"].get<int>() : DONE_NONE;
        int done = metadata.contains("done") ? metadata["done"].get<int>() : DONE_NONE;
        std::vector<std::string> reasons;
        if (metadata.contains("reasons") && metadata["reasons"].is_array()) {
            for (const auto& reason : metadata["reasons"]) {
                reasons.push_back(reason.get<std::string>());
            }
        }
        int mode = metadata.contains("mode") ? metadata["mode"].get<int>() : 0;

        // ✅ pusher_state 변환 (없으면 py::none())
        py::object pusher_state = py::none();
        std::vector<float> pusher_data;
        if (metadata.contains("pusher") && metadata["pusher"].is_array()) {
            for (const auto& val : metadata["pusher"]) {
                pusher_data.push_back(val.get<float>());
            }
            pusher_state = PySimulationViewer::getPusherState(pusher_data);
        }

        // ✅ slider_state 변환 (없으면 py::none())
        py::object slider_state = py::none();
        std::vector<std::vector<float>> slider_data;
        if (metadata.contains("sliders") && metadata["sliders"].is_array()) {
            for (const auto& row : metadata["sliders"]) {
                if (row.is_array()) {
                    std::vector<float> slider_row;
                    for (const auto& val : row) {
                        slider_row.push_back(val.get<float>());
                    }
                    slider_data.push_back(slider_row);
                }
            }
            slider_state = PySimulationViewer::getSliderState(slider_data);
        }

        // ✅ action 데이터 가져오기
        Eigen::VectorXf action_;
        if (metadata.contains("action") && metadata["action"].is_array()) {
            std::vector<float> action_vec = metadata["action"].get<std::vector<float>>();
            action_ = Eigen::Map<const Eigen::VectorXf>(action_vec.data(), action_vec.size());
        }

        // Convert action data to propriate move mode
        Eigen::Matrix2f rot_;
        if (player.gripperMovement == MOVE_XY) {
            rot_ <<  0, 1,
                    -1, 0;
        }
        else if (player.gripperMovement == MOVE_TO_TARGET) {
            std::vector<float> v_(2);
            std::transform(pusher_data.begin(), pusher_data.begin() + 2, slider_data[0].begin(), v_.begin(), std::minus<float>());

            float angle_ = -(std::atan2(v_[1], v_[0]) + M_PI / 2);
            float cos_r = std::cos(angle_);
            float sin_r = std::sin(angle_);

            rot_ <<  sin_r, cos_r,
                    -cos_r, sin_r;
        }
        else if (player.gripperMovement == MOVE_FORWARD) {
            float cos_r = std::cos(pusher_data[2]);
            float sin_r = std::sin(pusher_data[2]);

            rot_ << -sin_r, -cos_r,
                     cos_r, -sin_r;
        }
        else
        {
            throw std::invalid_argument("Gripper Movement is not defined");
        }

        action_.head<2>() = rot_ * action_.head<2>();  // 회전 적용

        std::vector<float> action(action_.data(), action_.data() + action_.size());

        // ✅ SimulationResult 생성
        SimulationResult result = {
            done,           // Simulation 종료 상태
            reasons,        // 종료 이유 리스트
            mode,           // Simulation mode
            numpy_frame,    // 이미지 상태
            pusher_state,   // Pusher 상태
            slider_state    // Slider 상태
        };

        // ✅ SimulationResult와 action을 함께 반환
        return std::make_tuple(new_video, result, action);
    }



private:
    Player& player;
    Player::Iterator iter, end_iter;

    py::object jsonToPyDict(const json& j) {
        if (j.is_object()) {
            py::dict pyDict;
            for (auto it = j.begin(); it != j.end(); ++it) {
                pyDict[py::str(it.key())] = jsonToPyDict(it.value());
            }
            return pyDict;
        } else if (j.is_array()) {
            py::list pyList;
            for (const auto& element : j) {
                pyList.append(jsonToPyDict(element));
            }
            return pyList;
        } else if (j.is_string()) {
            return py::str(j.get<std::string>());
        } else if (j.is_boolean()) {
            return py::bool_(j.get<bool>());
        } else if (j.is_number_integer()) {
            return py::int_(j.get<int>());
        } else if (j.is_number_unsigned()) {
            return py::int_(j.get<unsigned int>());
        } else if (j.is_number_float()) {
            return py::float_(j.get<double>());
        } else {
            return py::none();
        }
    }
};

PYBIND11_MODULE(quasi_static_push, m) {
    m.doc() = R"pbdoc(
        Quasi-Static Push Simulation Module using pybind11.

        This module provides an interface for simulating quasi-static object pushing
        and grasping in a 2D physics-based environment. It integrates a simulation engine
        with a visualization module, allowing users to interact with pushers, sliders,
        and obstacles. The module supports step-by-step execution, simulation state retrieval,
        and visualization.

        Features:
        ----------
        - Quasi-static object pushing and grasping simulation.
        - Configurable simulation environment with different object types.
        - Collision detection and tracking of closest contact points.
        - Real-time image rendering and state retrieval.
        - Automatic movement of the pusher towards the target.
        - Recording and playback of simulation frames.
        - Iterative playback of recorded videos with metadata.

        Classes:
        --------
        - `SimulationViewer`: Core simulation environment that manages objects and interactions.
        - `Player`: Loads and replays recorded simulation data.
        - `PyPlayerIterator`: Iterator for accessing frames from the recorded simulation.

        Enumerations:
        -------------
        - `SimulationDoneReason`: Defines different conditions that can terminate a simulation.
        - `GripperMotion`: Specifies gripper movement modes.

        Simulation Workflow:
        --------------------
        1. Create a `SimulationViewer` instance with the desired environment settings.
        2. Add objects (pushers, sliders) and configure simulation parameters.
        3. Run the simulation step-by-step using the `run()` method.
        4. Retrieve the current state, rendered image, and simulation results.
        5. Optionally, enable recording to save simulation frames for later replay.
        6. Use the `Player` class to iterate through recorded frames.

        Example Usage:
        --------------
        ```python
        from quasi_static_push import SimulationViewer, SimulationDoneReason, Player

        # Initialize the simulation viewer
        sim = SimulationViewer(window_width=1600, window_height=1600, scale=400.0, frame_rate=100)

        # Define object parameters
        slider_inputs = [("circle", [0.0, -0.5, 0.0, 0.45])]
        pusher_input = (3, 120.0, "superellipse", {"a": 0.015, "b": 0.03, "n": 10}, 0.10, 0.185, 0.04, 0.0, -1.2, 0.0)

        # Reset the environment with new objects
        sim.reset(slider_inputs, pusher_input, newtableWidth=2.0, newtableHeight=2.0)

        # Run simulation steps
        for _ in range(100):
            result = sim.run([0.1, 0.0, 0.0, 0.0])  # Example action
            if result.done:
                print("Simulation ended with reason:", result.reasons)
                break
        ```

        Example: Playing Recorded Simulation
        ------------------------------------
        ```python
        from quasi_static_push import Player

        player = Player("recordings")

        for new_video, result, action in player:
            print(f"Frame: {result.image_state.shape}, Mode: {result.mode}, Done: {result.done}")
            print(f"Action Taken: {action}")
        ```
    )pbdoc";

    py::enum_<SimulationDoneReason>(m, "SimulationDoneReason", R"pbdoc(
        Enum representing the reasons why a simulation might end.

        Values:
        -------
        - NONE (0): No termination condition met.
        - DONE_FALL_OUT (1): The object has fallen outside the table bounds.
        - DONE_GRASP_SUCCESS (2): Successful grasp detected.
        - DONE_GRASP_FAILED (4): Grasp failed due to insufficient grip.
    )pbdoc")
        .value("NONE", DONE_NONE)
        .value("DONE_FALL_OUT", DONE_FALL_OUT)
        .value("DONE_GRASP_SUCCESS", DONE_GRASP_SUCCESS)
        .value("DONE_GRASP_FAILED", DONE_GRASP_FAILED)
        .export_values();

    py::enum_<GripperMotion>(m, "GripperMotion", R"pbdoc(
        Enum representing gripper movement modes.

        Values:
        -------
        - MOVE_XY (0): Move in XY-plane without rotation.
        - MOVE_TO_TARGET (1): Rotate and move towards a specified target.
    )pbdoc")
        .value("MOVE_XY", MOVE_XY)
        .value("MOVE_TO_TARGET", MOVE_TO_TARGET)
        .export_values();

    py::class_<SimulationResult>(m, "SimulationResult", R"pbdoc(
        Structure to store the simulation results.

        Attributes:
        -----------
        - done (int): Bitwise OR of `SimulationDoneReason` values.
        - reasons (List[str]): List of reasons why the simulation ended.
        - mode (int): Current simulation mode.
        - image_state (numpy.ndarray): Rendered image of the simulation (H, W, 3 format).
        - pusher_state (numpy.ndarray): State of the pusher object.
        - slider_state (numpy.ndarray): State of the slider object.
    )pbdoc")
        .def_readonly("done", &SimulationResult::done)
        .def_readonly("reasons", &SimulationResult::reasons)
        .def_readonly("mode", &SimulationResult::mode)
        .def_readonly("image_state", &SimulationResult::image_state)
        .def_readonly("pusher_state", &SimulationResult::pusher_state)
        .def_readonly("slider_state", &SimulationResult::slider_state);

    py::class_<PySimulationViewer>(m, "SimulationViewer", R"pbdoc(
        Simulation viewer for quasi-static push simulation.

        This class integrates visualization, physics-based simulation, and user interaction.
        It manages objects such as pushers, sliders, and obstacles, tracking simulation progress.

        Parameters:
        -----------
        - window_width (int, default=1600): Width of the simulation window in pixels.
        - window_height (int, default=1600): Height of the simulation window in pixels.
        - scale (float, default=400.0): Scale factor for visualization.
        - tableWidth (float, default=2.0): Width of the simulation table in meters.
        - tableHeight (float, default=2.0): Height of the simulation table in meters.
        - frame_rate (float, default=100.0): Frame rate of the simulation (Hz).
        - frame_skip (int, default=10): Number of frames to skip during simulation.
        - grid (bool, default=True): Enable or disable grid visualization.
        - grid_space (float, default=0.1): Grid spacing in meters.
        - headless (bool, default=False): Enable or disable rendering.
        - gripper_movement (int, default=0): Enable automatic movement to a target.
        - show_closest_point (bool, default=True): Highlight closest contact points in visualization.
        - recording_enabled (bool, default=False): Enable recording of simulation frames.
        - recording_path (str, default="recordings"): Path to save recorded frames.

        Methods:
        --------
        reset(slider_inputs, pusher_input, newtableWidth, newtableHeight)
            Reset the simulation with new parameters.

            Parameters:
            - slider_inputs (List[Tuple[str, List[float]]]): List of sliders with their types and parameters.
            - pusher_input (Tuple[int, float, str, Dict[str, float], float, float, float, float, float, float]): 
              Configuration of the pusher.
            - newtableWidth (float): New table width in meters.
            - newtableHeight (float): New table height in meters.

        run(u_input: List[float]) -> SimulationResult
            Run the simulation for one step.

            Parameters:
            - u_input (List[float]): Input control vector for the simulation step.

            Returns:
            - SimulationResult: A structure containing simulation status, rendered image, and object states.

        keyboard_input() -> Tuple[np.ndarray, bool, bool]
            Get keyboard input from the simulation window.

            Returns:
            - Tuple[np.ndarray, bool, bool]: 
              - First element: An array of floats representing input directions.
              - Second element: Boolean indicating whether any key was pressed.
              - Third element: Boolean indicating whether input changed since the last call.
    )pbdoc")
        .def(py::init<int, int, float, float, float, float, int, bool, float, bool, int, bool, bool, std::string>(),
             py::arg("window_width") = 1600,
             py::arg("window_height") = 1600,
             py::arg("scale") = 400.0f,
             py::arg("tableWidth") = 2.0f,
             py::arg("tableHeight") = 2.0f,
             py::arg("frame_rate") = 100.0f,
             py::arg("frame_skip") = 10,
             py::arg("grid") = true,
             py::arg("grid_space") = 0.1f,
             py::arg("headless") = false,
             py::arg("gripper_movement") = 0,
             py::arg("show_closest_point") = true,
             py::arg("recording_enabled") = false,
             py::arg("recording_path") = "recordings")
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
            py::arg("newtableHeight") = 2.0f)
        .def("run", &PySimulationViewer::run)
        .def("keyboard_input", &PySimulationViewer::keyboard_input);

    py::class_<Player>(m, "Player", R"pbdoc(
        Player class for replaying recorded simulation data.

        This class loads simulation recordings and provides an iterator to retrieve 
        simulation states (`SimulationResult`) and corresponding actions.

        Parameters:
        -----------
        - directory (str, default="recordings"): Path to the recorded simulation data.
        - gripper_movement (GripperMotion, default=GripperMotion.MOVE_XY): Defines the movement mode of the gripper.

        Methods:
        --------
        __iter__()
            Returns an iterator that yields `SimulationResult` and `action`.

        Example:
        --------
        ```python
        from quasi_static_push import Player, GripperMotion

        player = Player("recordings", GripperMotion.MOVE_TO_TARGET)

        for sim_result, action in player:
            print(f"Simulation Mode: {sim_result.mode}, Done: {sim_result.done}")
            print(f"Action Taken: {action}")
        ```
    )pbdoc")
        .def(py::init<std::string, GripperMotion>(), py::arg("directory") = "recordings", py::arg("gripper_movement") = GripperMotion::MOVE_XY)
        .def("__iter__", [](Player& self) { return PyPlayerIterator(self); });

    py::class_<PyPlayerIterator>(m, "PyPlayerIterator", R"pbdoc(
        Iterator for processing frames from recorded simulation data.

        This iterator retrieves simulation states (`SimulationResult`) and corresponding actions (`action`)
        for each recorded frame.

        Methods:
        --------
        __next__()
            Retrieves the next simulation frame and its corresponding action.

            Returns:
            --------
            Tuple[bool, SimulationResult, List[float]]:
                - `is_new_video`: Return true on the very first frame of the video.
                - `SimulationResult`: Contains simulation state information.
                - `action` (List[float]): Control inputs applied in the simulation step.

            Raises:
            -------
            - `StopIteration`: When all frames are exhausted.
    )pbdoc")
        .def("__iter__", [](PyPlayerIterator& self) -> PyPlayerIterator& { return self; })
        .def("__next__", &PyPlayerIterator::next);

}
