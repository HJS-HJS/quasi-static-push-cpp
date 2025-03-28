#include "utils/viewer.h"
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <cmath>

SimulationViewer::SimulationViewer(int width, int height, float unit, float tableWidth, float tableHeight, bool showGrid, float gridSpacingMeters, bool displayWindow)
    : screenWidth(width), screenHeight(height), unit(unit), showGrid(showGrid), gridSpacingMeters(gridSpacingMeters),
      displayWindow(displayWindow), tableWidth((int)(tableWidth * unit)), tableHeight((int)(tableHeight * unit)),
      window(nullptr), renderer(nullptr) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        throw std::runtime_error("Failed to initialize SDL: " + std::string(SDL_GetError()));
    }

    if (displayWindow) {
        window = SDL_CreateWindow("Simulation Viewer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, screenWidth, screenHeight, SDL_WINDOW_SHOWN);
        if (!window) {
            throw std::runtime_error("Failed to create window: " + std::string(SDL_GetError()));
        }

        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
        if (!renderer) {
            SDL_DestroyWindow(window);
            throw std::runtime_error("Failed to create renderer: " + std::string(SDL_GetError()));
        }
    } else {
        window = SDL_CreateWindow("Simulation Viewer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, screenWidth, screenHeight, SDL_WINDOW_HIDDEN);
        if (!window) {
            throw std::runtime_error("Failed to create window: " + std::string(SDL_GetError()));
        }
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    }
}

SimulationViewer::~SimulationViewer() {
    for (auto& [diagram, texturePair] : diagramTextures) {
        SDL_DestroyTexture(texturePair.first);
    }
    if (renderer) {
        SDL_DestroyRenderer(renderer);
    }
    if (window) {
        SDL_DestroyWindow(window);
    }
    SDL_Quit();
}

void SimulationViewer::setGridVisibility(bool visibility) {
    showGrid = visibility;
}

void SimulationViewer::setGridSpacing(float spacing) {
    gridSpacingMeters = spacing;
}

void SimulationViewer::renderDiagram(const Diagram* diagram, const SDL_Color& color, bool priority, bool pattern) {
    auto it = diagramTextures.find(diagram);
    if (it != diagramTextures.end()) {
        SDL_DestroyTexture(it->second.first);
        diagramTextures.erase(it);
    }

    float rotation = diagram->q[2];
    int point_size = 1000;

    const_cast<Diagram*>(diagram)->q[2] = 0;
    auto points = diagram->points(point_size, 0, 2 * M_PI);
    const_cast<Diagram*>(diagram)->q[2] = rotation;

    // Calculate bounding box for the diagram
    for (auto& point : points) {
        point[0] -= diagram->q[0];
        point[1] -= diagram->q[1];
    }
    float width  = diagram->radius * 2 * unit;
    float height = diagram->radius * 2 * unit;

    // Create texture for diagram
    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, width, height);
    SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND); // Enable transparency
    SDL_SetRenderTarget(renderer, texture);

    // Clear the texture to transparent
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
    SDL_RenderClear(renderer);

    // Prepare points for SDL2_gfx's filledPolygonRGBA
    std::vector<Sint16> x_points;
    std::vector<Sint16> y_points;
    for (const auto& point : points) {
        x_points.push_back(static_cast<Sint16>(point[0] * unit + width / 2));
        y_points.push_back(static_cast<Sint16>(point[1] * unit + height / 2));
    }

    // Render the filled polygon
    filledPolygonRGBA(renderer, x_points.data(), y_points.data(), x_points.size(), color.r, color.g, color.b, color.a);

    if (pattern){
        std::vector<float> scale_factors = {0.8f, 0.65f, 0.5f, 0.35f, 0.2f}; 

        for (float scale : scale_factors) {
            auto inner_points = diagram->points(point_size, 0, 2 * M_PI);
            std::vector<Sint16> inner_x_points, inner_y_points;
            
            for (auto& point : points) {
                float point_x = point[0] * scale;
                float point_y = point[1] * scale;
                inner_x_points.push_back(static_cast<Sint16>(point_x * unit + width / 2));
                inner_y_points.push_back(static_cast<Sint16>(point_y * unit + height / 2));
            }

            filledPolygonRGBA(renderer, inner_x_points.data(), inner_y_points.data(), inner_x_points.size(), 255, 255, 255, 100);
        }

        // Add inner gradient color pattern
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a); 

        int center_x = static_cast<int>(width / 2);
        int center_y = static_cast<int>(height / 2);

        filledCircleRGBA(renderer, center_x, center_y, 6, color.r, color.g, color.b, 255);
    }

    SDL_SetRenderTarget(renderer, nullptr);
    diagramTextures[diagram] = std::make_pair(texture, priority);
}

void SimulationViewer::addDiagram(const Diagram* diagram, const std::string& colorName, bool priority, bool pattern) {
    if (colorMap.find(colorName) == colorMap.end()) {
        throw std::invalid_argument("Color name not found in predefined colors: " + colorName);
    }
    SDL_Color color = colorMap[colorName];
    renderDiagram(diagram, color, priority, pattern);
}


void SimulationViewer::addDiagram(const std::vector<std::unique_ptr<Diagram>>& diagrams, const std::string& colorName, bool priority, bool pattern) {
    if (colorMap.find(colorName) == colorMap.end()) {
        throw std::invalid_argument("Color name not found in predefined colors: " + colorName);
    }
    SDL_Color color = colorMap[colorName];
    for (const auto& diagram : diagrams) {
        renderDiagram(diagram.get(), color, priority, pattern);
    }
}

void SimulationViewer::removeDiagram(const Diagram* diagram) {
    auto it = diagramTextures.find(diagram);
    if (it != diagramTextures.end()) {
        SDL_DestroyTexture(it->second.first);
        diagramTextures.erase(it);
    }
    currentDiagrams.erase(diagram);
}
void SimulationViewer::changeDiagramColor(const Diagram* diagram, const std::string& newColorName, bool priority, bool pattern) {
    if (colorMap.find(newColorName) == colorMap.end()) {
        throw std::invalid_argument("Color name not found in predefined colors: " + newColorName);
    }

    auto it = diagramTextures.find(diagram);
    if (it != diagramTextures.end()) {
        SDL_DestroyTexture(it->second.first);
        diagramTextures.erase(it);
    }

    SDL_Color newColor = colorMap[newColorName];
    renderDiagram(diagram, newColor, priority, pattern);
}
void SimulationViewer::changeDiagramColor(const std::vector<std::unique_ptr<Diagram>>& diagrams, const std::string& newColorName, bool priority, bool pattern) {
    if (colorMap.find(newColorName) == colorMap.end()) {
        throw std::invalid_argument("Color name not found in predefined colors: " + newColorName);
    }
    for (const auto& diagram : diagrams) {
        auto it = diagramTextures.find(diagram.get());
        if (it != diagramTextures.end()) {
            SDL_DestroyTexture(it->second.first);
            diagramTextures.erase(it);
        }

        SDL_Color newColor = colorMap[newColorName];
        renderDiagram(diagram.get(), newColor, priority, pattern);
    }
}

void SimulationViewer::reset(float newtableWidth, float newtableHeight, bool newDisplayWindow) {
    tableWidth = (int)(newtableWidth * unit);
    tableHeight = (int)(newtableHeight * unit);

    for (auto& [diagram, texture] : diagramTextures) {
        SDL_DestroyTexture(texture.first);
    }
    diagramTextures.clear();

    displayWindow = newDisplayWindow;

    // if (!renderer) {
    //     throw std::runtime_error("Renderer not initialized");
    // }

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // Reset background to black
    SDL_RenderClear(renderer);
    render();
}

void SimulationViewer::drawGrid() const {
    if (!showGrid) return;

    SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
    int spacingPixels = static_cast<int>(gridSpacingMeters * unit);
    int centerX = screenWidth / 2;
    int centerY = screenHeight / 2;

    for (int x = centerX; x <= screenWidth; x += spacingPixels) {
        SDL_RenderDrawLine(renderer, x, 0, x, screenHeight);
    }
    for (int x = centerX; x >= 0; x -= spacingPixels) {
        SDL_RenderDrawLine(renderer, x, 0, x, screenHeight);
    }
    for (int y = centerY; y <= screenHeight; y += spacingPixels) {
        SDL_RenderDrawLine(renderer, 0, y, screenWidth, y);
    }
    for (int y = centerY; y >= 0; y -= spacingPixels) {
        SDL_RenderDrawLine(renderer, 0, y, screenWidth, y);
    }
}

void SimulationViewer::drawBackground() const {
    SDL_SetRenderDrawColor(renderer, 150, 150, 150, 255);
    int rectX = (screenWidth - tableWidth) / 2;
    int rectY = (screenHeight - tableHeight) / 2;
    SDL_Rect rect = {rectX, rectY, tableWidth, tableHeight};
    SDL_RenderFillRect(renderer, &rect);
}

void SimulationViewer::render(bool render_gripper,
                              const std::vector<std::vector<float>>& points, 
                              const std::vector<std::tuple<float, float, float, float>>& arrows
                              ) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // Set background to black
    SDL_RenderClear(renderer);

    drawBackground();
    drawGrid();
    
    for (const auto& [diagram, pair] : diagramTextures) {
        if (!pair.second) { // priority == false 인 도형
            renderTexture(diagram, pair.first);
        }
    }
    if (render_gripper) {
        for (const auto& [diagram, pair] : diagramTextures) {
            if (pair.second) { // priority == true 인 도형
                renderTexture(diagram, pair.first);
            }
        }
    }

    // Draw points and arrows
    drawPoints(points, "green", 10);
    drawArrows(arrows, "yellow", 5);

    if (displayWindow) {
        SDL_RenderPresent(renderer);
    }
}

void SimulationViewer::renderTexture(const Diagram* diagram, SDL_Texture* texture) {
    const auto& position = diagram->q;

    SDL_Rect targetRect;
    float width  = diagram->radius * 2 * unit;
    float height = diagram->radius * 2 * unit;
    targetRect.x = static_cast<int>(position[0] * unit + screenWidth / 2 - width / 2);
    targetRect.y = static_cast<int>(-position[1] * unit + screenHeight / 2 - height / 2);
    targetRect.w = static_cast<int>(width);
    targetRect.h = static_cast<int>(height);

    SDL_RenderCopyEx(renderer, texture, nullptr, &targetRect, -position[2] * 180 / M_PI, nullptr, SDL_FLIP_NONE);
}

void SimulationViewer::drawPoints(const std::vector<std::vector<float>>& points, const std::string& colorName, int thickness) const {
    if (colorMap.find(colorName) == colorMap.end()) {
        throw std::invalid_argument("Color name not found in predefined colors: " + colorName);
    }

    SDL_Color color = colorMap.at(colorName);
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

    for (const auto& point : points) {
        if (point.size() != 2) continue; // Ensure each point has x, y
        int x = static_cast<int>(point[0] * unit + screenWidth / 2);
        int y = static_cast<int>(-point[1] * unit + screenHeight / 2); // Flip y-axis

        SDL_Rect rect = {x - thickness / 2, y - thickness / 2, thickness, thickness};
        SDL_RenderFillRect(renderer, &rect);
    }
}

void SimulationViewer::drawArrows(const std::vector<std::tuple<float, float, float, float>>& arrows, const std::string& colorName, int thickness) const {
    if (colorMap.find(colorName) == colorMap.end()) {
        throw std::invalid_argument("Color name not found in predefined colors: " + colorName);
    }

    SDL_Color color = colorMap.at(colorName);
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

    for (const auto& [x, y, angle, length] : arrows) {
        int startX = static_cast<int>(x * unit + screenWidth / 2);
        int startY = static_cast<int>(-y * unit + screenHeight / 2); // Flip y-axis
        int endX = static_cast<int>(startX + length * unit * std::cos(angle));
        int endY = static_cast<int>(startY - length * unit * std::sin(angle)); // Flip y-axis

        // Draw a thick line for the arrow
        for (int i = -thickness / 2; i <= thickness / 2; ++i) {
            SDL_RenderDrawLine(renderer, startX + i, startY, endX + i, endY);
        }

        // Draw arrowhead
        float arrowheadLength = length * 0.2f; // Arrowhead length is 20% of the arrow length
        float arrowAngle = M_PI / 6;          // Arrowhead angle (30 degrees)

        int leftX = static_cast<int>(endX - arrowheadLength * unit * std::cos(angle - arrowAngle));
        int leftY = static_cast<int>(endY + arrowheadLength * unit * std::sin(angle - arrowAngle));
        int rightX = static_cast<int>(endX - arrowheadLength * unit * std::cos(angle + arrowAngle));
        int rightY = static_cast<int>(endY + arrowheadLength * unit * std::sin(angle + arrowAngle));

        for (int i = -thickness / 2; i <= thickness / 2; ++i) {
            SDL_RenderDrawLine(renderer, endX, endY, leftX + i, leftY);
            SDL_RenderDrawLine(renderer, endX, endY, rightX + i, rightY);
        }
    }
}


SDL_Surface* SimulationViewer::getRenderedImage() {
    SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormat(0, screenWidth, screenHeight, 32, SDL_PIXELFORMAT_RGBA32);
    SDL_RenderReadPixels(renderer, nullptr, SDL_PIXELFORMAT_RGBA32, surface->pixels, surface->pitch);
    return surface;
}

std::tuple<std::array<float, 5>, bool, bool> SimulationViewer::getKeyboardInput(){
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_KEYDOWN) {
            keyStates[event.key.keysym.sym] = true;
        }
        if (event.type == SDL_KEYUP) {
            keyStates[event.key.keysym.sym] = false;
        }
    }

    SDL_Delay(16);

    std::array<float, 5> output = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    if      (keyStates[SDLK_w]) output[0] = 1.0f;
    else if (keyStates[SDLK_s]) output[0] = -1.0f;
    if      (keyStates[SDLK_a]) output[1] = 1.0f;
    else if (keyStates[SDLK_d]) output[1] = -1.0f;
    if      (keyStates[SDLK_q]) output[2] = 1.0f;
    else if (keyStates[SDLK_e]) output[2] = -1.0f;
    if      (keyStates[SDLK_LEFT]) output[3] = -1.0f;
    else if (keyStates[SDLK_RIGHT]) output[3] = 1.0f;
    if      (keyStates[SDLK_SPACE]) output[4] = 1.0f;



    return std::make_tuple(output, keyStates[SDLK_ESCAPE], keyStates[SDLK_r]);
}
