#ifndef RECORDER_H
#define RECORDER_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <nlohmann/json.hpp>
#include <iostream>
#include <filesystem>
#include <sstream>
#include <iomanip>

using json = nlohmann::json;

class Recorder {
public:
    Recorder(const std::string& saveDirectory, double frameTimeSec);
    ~Recorder();

    void startRecording(int fps = 30);
    void saveFrame(const cv::Mat& frame, const std::vector<float>& pusher, 
                   const std::vector<float>& sliders, const std::vector<float>& action);
    void stopRecording();

    std::function<bool(cv::Mat&, std::vector<float>&, std::vector<float>&, std::vector<float>&)> play();

private:
    std::string saveDirectory;
    double frameTimeSec;
    int frameCount;
    int videoIndex;

    cv::VideoWriter videoWriter;
    bool recording;

    std::ofstream metadataFile;

    void ensureDirectoryExists();
    std::string generateVideoFileName();
    std::string getMetadataFileName();
    void saveMetadata(int frameIndex, const std::vector<float>& pusher, 
                      const std::vector<float>& sliders, const std::vector<float>& action);
};

#endif // RECORDER_H
