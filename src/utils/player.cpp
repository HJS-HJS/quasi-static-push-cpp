#include "utils/player.h"

Player::Player(const std::string& directory) : directory(directory), currentVideoIndex(0), currentFrameIndex(0) {
    // ✅ 폴더 존재 여부 확인
    if (!std::filesystem::exists(directory)) {
        // std::cerr << "[Player] ❌ Directory not found: " << directory << std::endl;
        // std::cerr << "[Player] 🔧 Creating directory: " << directory << std::endl;
        try {
            std::filesystem::create_directories(directory);
            // std::cout << "[Player] ✅ Directory created successfully: " << directory << std::endl;
        } catch (const std::filesystem::filesystem_error& e) {
            // std::cerr << "[Player] ❌ Failed to create directory: " << e.what() << std::endl;
            return;
        }
    }

    // ✅ 폴더가 존재하지만 비어 있는 경우 처리
    bool hasVideo = false;
    bool hasMetadata = false;

    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.path().extension() == ".mp4") {
            videoFiles.push_back(entry.path().string());
            hasVideo = true;
        } else if (entry.path().extension() == ".json") {
            metadataFiles.push_back(entry.path().string());
            hasMetadata = true;
        }
    }

    std::sort(videoFiles.begin(), videoFiles.end());
    std::sort(metadataFiles.begin(), metadataFiles.end());

    // ✅ 디버그 메시지 추가
    // std::cout << "[Player] 🎥 Found " << videoFiles.size() << " videos and " << metadataFiles.size() << " metadata files.\n";

    // ✅ 영상이나 메타데이터가 없으면 오류 출력 후 종료
    if (!hasVideo || !hasMetadata) {
        // std::cerr << "[Player] ❌ No videos or metadata found in directory: " << directory << std::endl;
        return;
    }

    loadNextVideo();
}
Player::~Player() {
    videoCapture.release();
    if (metadataFile.is_open()) metadataFile.close();
}

bool Player::loadNextVideo() {
    if (currentVideoIndex >= videoFiles.size()) {
        // std::cout << "[Player] 📂 No more videos to load.\n";
        return false;
    }

    std::cout << "[Player] 🔄 Loading video: " << videoFiles[currentVideoIndex] << std::endl;

    videoCapture.release();
    if (metadataFile.is_open()) metadataFile.close();

    videoCapture.open(videoFiles[currentVideoIndex]);
    if (!videoCapture.isOpened()) {
        std::cerr << "[Player] ❌ Failed to open video file: " << videoFiles[currentVideoIndex] << std::endl;
        currentVideoIndex++;
        return loadNextVideo(); // 다음 영상으로 시도
    }

    int totalFrames = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_COUNT));
    std::cout << "[Player] 🎞 Total frames in video: " << totalFrames << std::endl;

    metadataFile.open(metadataFiles[currentVideoIndex]);
    if (!metadataFile.is_open()) {
        // std::cerr << "[Player] ❌ Failed to open metadata file: " << metadataFiles[currentVideoIndex] << std::endl;
        currentVideoIndex++;
        return loadNextVideo(); // 다음 영상으로 시도
    }

    metadataFile >> metadataJson;
    currentFrameIndex = 0;
    currentVideoIndex++;
    return true;
}

bool Player::hasNextFrame() {
    if (videoCapture.isOpened() && currentFrameIndex < metadataJson.size()) {
        return true;
    }

    std::cout << "[Player] ⏹ Current video ended, trying to load next video...\n";
    return loadNextVideo();
}

std::pair<cv::Mat, json> Player::getNextFrame() {
    if (!hasNextFrame()) {
        // std::cout << "[Player] ⚠ No next frame available, returning empty frame.\n";
        return {cv::Mat(), json()};
    }

    cv::Mat frame;
    videoCapture >> frame;

    if (frame.empty()) {
        // std::cout << "[Player] 🔄 Frame empty, attempting to load next video.\n";
        if (!loadNextVideo()) {
            // std::cout << "[Player] ❌ No more videos, stopping iteration.\n";
            return {cv::Mat(), json()};
        }
        return getNextFrame();
    }

    json frameData = metadataJson[currentFrameIndex++];
    return {frame, frameData};
}

Player::Iterator::Iterator(Player* player, bool is_end) : player(player), is_end(is_end) {
    if (player && !is_end && !player->hasNextFrame()) {
        is_end = true;
    }
}

bool Player::Iterator::operator==(const Iterator& other) const {
    return is_end == other.is_end;
}

bool Player::Iterator::operator!=(const Iterator& other) const {
    return !(*this == other);
}

Player::Iterator& Player::Iterator::operator++() {
    if (player && player->hasNextFrame()) {
        auto [frame, metadata] = player->getNextFrame();
        if (frame.empty()) {
            is_end = true;
        }
    } else {
        is_end = true;
    }
    return *this;
}

std::pair<cv::Mat, json> Player::Iterator::operator*() {
    if (!player || is_end) {
        throw std::out_of_range("[Iterator] ❌ Attempted to dereference end iterator.");
    }
    return player->getNextFrame();
}

Player::Iterator Player::begin() {
    return Iterator(this, false);
}

Player::Iterator Player::end() {
    return Iterator(this, true);
}
