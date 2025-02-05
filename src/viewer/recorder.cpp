#include "viewer/recorder.h"

namespace fs = std::filesystem;

Recorder::Recorder(const std::string& saveDirectory, double frameTimeSec)
    : saveDirectory(saveDirectory), frameTimeSec(frameTimeSec),
      frameCount(0), videoIndex(1), recording(false) {
    ensureDirectoryExists();
}

Recorder::~Recorder() {
    stopRecording();
}

void Recorder::ensureDirectoryExists() {
    if (!fs::exists(saveDirectory)) {
        fs::create_directories(saveDirectory);
    }
}

std::string Recorder::generateVideoFileName() {
    ensureDirectoryExists();
    int count = 0;
    for (const auto& entry : fs::directory_iterator(saveDirectory)) {
        if (entry.path().extension() == ".mp4") {
            count++;
        }
    }
    std::ostringstream ss;
    ss << saveDirectory << "/" << std::setw(3) << std::setfill('0') << (count + 1) << ".mp4";
    return ss.str();
}

std::string Recorder::getMetadataFileName() {
    return saveDirectory + "/metadata.json";
}

void Recorder::startRecording(int fps) {
    if (recording) return;
    std::string filename = generateVideoFileName();
    int fourcc = cv::VideoWriter::fourcc('H', '2', '6', '4');
    videoWriter.open(filename, fourcc, fps, cv::Size(800, 600), true);
    if (!videoWriter.isOpened()) {
        throw std::runtime_error("Failed to open video file for recording");
    }
    metadataFile.open(getMetadataFileName(), std::ios::app);
    recording = true;
}

void Recorder::saveFrame(const cv::Mat& frame, const std::vector<float>& pusher, 
                         const std::vector<float>& sliders, const std::vector<float>& action) {
    if (!recording) return;
    videoWriter.write(frame);
    saveMetadata(frameCount, pusher, sliders, action);
    frameCount++;
}

void Recorder::saveMetadata(int frameIndex, const std::vector<float>& pusher, 
                            const std::vector<float>& sliders, const std::vector<float>& action) {
    json jsonData;
    jsonData["frame"] = frameIndex;
    jsonData["time"] = frameIndex * frameTimeSec;
    jsonData["pusher"] = pusher;
    jsonData["sliders"] = sliders;
    jsonData["action"] = action;
    metadataFile << jsonData.dump() << std::endl;
}

void Recorder::stopRecording() {
    if (recording) {
        videoWriter.release();
        metadataFile.close();
        recording = false;
        videoIndex++;
    }
}
