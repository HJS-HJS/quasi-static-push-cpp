#ifndef RECORDER_H
#define RECORDER_H

#include <opencv2/opencv.hpp>
#include <string>
#include <regex>
#include <vector>
#include <fstream>
#include <nlohmann/json.hpp>
#include <iostream>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <thread>

using json = nlohmann::json;

class Recorder {
public:
    Recorder(const std::string& saveDirectory, double frameTimeSec, int width = 800, int height = 600);
    ~Recorder();

    void startRecording();
    void saveFrame(const cv::Mat& frame, const std::vector<float>& pusher, 
                   const std::vector<std::vector<float>>& sliders, const std::vector<float>& action);
    void saveFrame(const cv::Mat& frame, 
                   const int done, 
                   const std::vector<std::string> reasons, 
                   const int mode,
                   const std::vector<float>& pusher, 
                   const std::vector<std::vector<float>>& sliders,
                   const std::vector<float>& action
                   );
    void stopRecording();
    bool renameSavedFiles(const std::string& currentBaseName,
                          const std::string& newBaseName,
                          bool appendOldNameToNew);

private:
    std::string saveDirectory;
    double frameTimeSec;
    int frameCount;
    bool recording;
    int width;
    int height;

    cv::VideoWriter videoWriter;
    std::ofstream metadataFile;
    std::string videoFileName;
    std::string metadataFileName;

    void ensureDirectoryExists();
    std::string generateVideoFileName();
    std::string getVideoFileName();
    void saveMetadata(int frameIndex, 
                      const int done, 
                      const std::vector<std::string> reasons, 
                      const int mode,
                      const std::vector<float>& pusher, 
                      const std::vector<std::vector<float>>& sliders,
                      const std::vector<float>& action
                      );
};

#endif // RECORDER_H
