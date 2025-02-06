#include "utils/recorder.h"

namespace fs = std::filesystem;

Recorder::Recorder(const std::string& saveDirectory, double frameTimeSec, int width, int height)
    : saveDirectory(saveDirectory), frameTimeSec(frameTimeSec), frameCount(0), recording(false),
      width(width), height(height) {
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

void Recorder::startRecording() {
    if (recording) return;

    videoFileName = generateVideoFileName();
    metadataFileName = videoFileName.substr(0, videoFileName.find_last_of(".")) + ".json";

    metadataFile.open(metadataFileName, std::ios::out);
    metadataFile << "[";

    int fps = static_cast<int>(1.0 / frameTimeSec);
    // int fourcc = cv::VideoWriter::fourcc('H', '2', '6', '4');  // H.264 코덱 사용
    int fourcc = cv::VideoWriter::fourcc('a', 'v', 'c', '1');  // H.264 코덱 사용
    videoWriter.open(videoFileName, fourcc, fps, cv::Size(width, height), true);

    if (!videoWriter.isOpened()) {
        // std::cerr << "[Recorder] ❌ Failed to open video file for recording!" << std::endl;
        return;
    }

    recording = true;
    frameCount = 0;
    // std::cout << "[Recorder] 🎥 Recording started: " << videoFileName << std::endl;
}

void Recorder::saveFrame(const cv::Mat& frame, const std::vector<float>& pusher, 
                         const std::vector<std::vector<float>>& sliders, const std::vector<float>& action) {
    if (!recording || !videoWriter.isOpened()) return;

    if (frame.empty()) {
        std::cerr << "[Recorder] ❌ Empty frame detected at frame " << frameCount << std::endl;
        return;
    }

    videoWriter.write(frame);

    if (frameCount > 0) {
        metadataFile << ",\n";
    }

    saveMetadata(frameCount, pusher, sliders, action);
    frameCount++;

    // std::cout << "[Recorder] 📸 Frame " << frameCount << " saved." << std::endl;
}

void Recorder::saveMetadata(int frameIndex, const std::vector<float>& pusher, 
                            const std::vector<std::vector<float>>& sliders, const std::vector<float>& action) {
    json jsonData;
    jsonData["frame"] = frameIndex;
    jsonData["time"] = frameIndex * frameTimeSec;
    jsonData["pusher"] = pusher;
    jsonData["sliders"] = sliders;
    jsonData["action"] = action;
    
    metadataFile << jsonData.dump(4);
}

void Recorder::stopRecording() {
    if (!recording) return;

    // std::cout << "[Recorder] 🛑 Stopping recording. Total frames recorded: " << frameCount << std::endl;

    videoWriter.release();
    metadataFile << "\n]";
    metadataFile.close();
    recording = false;

    if (frameCount == 0) {
        // std::cout << "[Recorder] ⚠ No frames recorded. Deleting video & metadata files." << std::endl;
        if (fs::exists(videoFileName)) fs::remove(videoFileName);
        if (fs::exists(metadataFileName)) fs::remove(metadataFileName);
        return;
    }

    // std::cout << "[Recorder] ✅ Video file saved: " << videoFileName << std::endl;
    // checkVideoIntegrity();
}

void Recorder::checkVideoIntegrity() {
    if (!fs::exists(videoFileName)) {
        // std::cerr << "[Recorder] ❌ Error: Video file does not exist!" << std::endl;
        return;
    }

    long fileSize = fs::file_size(videoFileName);
    std::cout << "[Recorder] ✅ Final video file size: " << fileSize << " bytes" << std::endl;

    std::string command = "ffmpeg -i " + videoFileName + " -hide_banner";
    std::cout << "[Recorder] Running FFmpeg check:\n" << command << std::endl;
    std::system(command.c_str());
}
