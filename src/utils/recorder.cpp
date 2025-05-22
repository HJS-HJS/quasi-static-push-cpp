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
    ss << saveDirectory << "/" << std::setw(4) << std::setfill('0') << (count + 1) << ".mp4";
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
        // std::cerr << "[Recorder] Failed to open video file for recording!" << std::endl;
        return;
    }

    recording = true;
    frameCount = 0;
    // std::cout << "[Recorder] Recording started: " << videoFileName << std::endl;
}

void Recorder::saveFrame(const cv::Mat& frame, 
                         const int done, 
                         const std::vector<std::string> reasons, 
                         const int mode,
                         const std::vector<float>& pusher, 
                         const std::vector<std::vector<float>>& sliders,
                         const std::vector<float>& action) {
    if (!recording || !videoWriter.isOpened()) return;

    if (frame.empty()) {
        std::cerr << "[Recorder] Empty frame detected at frame " << frameCount << std::endl;
        return;
    }

    videoWriter.write(frame);

    if (frameCount > 0) {
        metadataFile << ",\n";
    }

    saveMetadata(frameCount, done, reasons, mode, pusher, sliders, action);
    frameCount++;

    // std::cout << "[Recorder] Frame " << frameCount << " saved." << std::endl;
}

void Recorder::saveMetadata(int frameIndex, 
                            const int done, 
                            const std::vector<std::string> reasons, 
                            const int mode,
                            const std::vector<float>& pusher, 
                            const std::vector<std::vector<float>>& sliders,
                            const std::vector<float>& action) {
    json jsonData;
    jsonData["frame"] = frameIndex;
    jsonData["time"] = frameIndex * frameTimeSec;
    jsonData["done"] = done;
    jsonData["reasons"] = reasons;
    jsonData["mode"] = mode;
    jsonData["pusher"] = pusher;
    jsonData["sliders"] = sliders;
    jsonData["action"] = action;
    
    metadataFile << jsonData.dump(4);
}

void Recorder::stopRecording() {
    if (!recording) return;

    videoWriter.release();
    metadataFile << "\n]";
    metadataFile.close();
    recording = false;

    if (frameCount <= 1) {
        if (fs::exists(videoFileName)) fs::remove(videoFileName);
        if (fs::exists(metadataFileName)) fs::remove(metadataFileName);
        return;
    }
}

bool Recorder::renameSavedFiles(const std::string& currentBaseName,
                                const std::string& newBaseName,
                                bool appendOldNameToNew) {
    std::string baseName = currentBaseName;

    // 🔍 currentBaseName이 비어 있으면, 가장 최근에 저장된 .mp4 파일 이름을 찾는다
    std::string currentVideoFile;
    std::string currentJsonFile;
    if (baseName.empty()) {
        stopRecording();
        currentVideoFile = videoFileName;
        currentJsonFile = metadataFileName;
        baseName = fs::path(videoFileName).stem().string();
    }
    else{
        currentVideoFile = saveDirectory + "/" + baseName + ".mp4";
        currentJsonFile = saveDirectory + "/" + baseName + ".json";
    }


    int waited = 0;
    while ((!fs::exists(videoFileName) || !fs::exists(metadataFileName)) && waited < 1000) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        waited += 10;
    }

    if (!fs::exists(currentVideoFile) || !fs::exists(currentJsonFile)) {
        std::cerr << "[Recorder] ⚠ File(s) not found: "
                  << currentVideoFile << " or " << currentJsonFile << std::endl;
        return false;
    }

    std::string finalBaseName = appendOldNameToNew
        ? newBaseName + "_" + baseName
        : newBaseName;

    std::string newVideoFile = saveDirectory + "/" + finalBaseName + ".mp4";
    std::string newJsonFile = saveDirectory + "/" + finalBaseName + ".json";

    try {
        fs::rename(currentVideoFile, newVideoFile);
        fs::rename(currentJsonFile, newJsonFile);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[Recorder] Rename failed: " << e.what() << std::endl;
        return false;
    }
}
