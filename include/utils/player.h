#ifndef PLAYER_HPP
#define PLAYER_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace fs = std::filesystem;
using json = nlohmann::json;

class Player {
public:
    explicit Player(const std::string& directory, const int gripper_movement);
    ~Player();

    bool loadNextVideo();
    bool hasNextFrame();
    std::pair<cv::Mat, json> getNextFrame();

    // ✅ C++ 표준 이터레이터 패턴 구현
    class Iterator {
    public:
        Iterator(Player* player, bool is_end = false);
        Iterator& operator++();
        bool operator!=(const Iterator& other) const;
        bool operator==(const Iterator& other) const;

        std::pair<cv::Mat, json> operator*();

    private:
        Player* player;
        bool is_end;
    };

    Iterator begin();
    Iterator end();
    
    int gripperMovement;

private:
    std::string directory;
    std::vector<std::string> videoFiles;
    std::vector<std::string> metadataFiles;
    size_t currentVideoIndex;

    cv::VideoCapture videoCapture;
    std::ifstream metadataFile;
    json metadataJson;
    size_t currentFrameIndex;
};

#endif // PLAYER_HPP
