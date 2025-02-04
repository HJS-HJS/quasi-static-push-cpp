#include "viewer/viewer.h"
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <cmath>

SimulationViewer::SimulationViewer(int width, int height, float unit, float table_size_x, float table_size_y, bool showGrid, bool displayWindow, float gridSpacingMeters)
    : screenWidth(width), screenHeight(height), unit(unit), showGrid(showGrid), displayWindow(displayWindow),
      gridSpacingMeters(gridSpacingMeters), rectangleWidth((int)(table_size_x * unit)), rectangleHeight((int)(table_size_y * unit)),
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
        renderer = SDL_CreateRenderer(nullptr, -1, SDL_RENDERER_ACCELERATED);
    }
}

SimulationViewer::~SimulationViewer() {
    for (auto& [diagram, texture] : diagramTextures) {
        SDL_DestroyTexture(texture);
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

void SimulationViewer::renderDiagram(const Diagram* diagram, const SDL_Color& color) {
    float rotation = diagram->q[2];
    int point_size = 2000;

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

    // Add a white cross in the center with thickness
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255); // White color
    int center_x = static_cast<int>(width / 2);
    int center_y = static_cast<int>(height / 2);
    int thickness = 3; // Thickness of the cross

    // Horizontal bar of the cross
    SDL_Rect horizontal_bar = {center_x - 10, center_y - thickness / 2, 20, thickness};
    SDL_RenderFillRect(renderer, &horizontal_bar);

    // Vertical bar of the cross
    SDL_Rect vertical_bar = {center_x - thickness / 2, center_y - 10, thickness, 20};
    SDL_RenderFillRect(renderer, &vertical_bar);

    SDL_SetRenderTarget(renderer, nullptr);
    diagramTextures[diagram] = texture;
}

void SimulationViewer::addDiagram(const Diagram* diagram, const std::string& colorName) {
    if (colorMap.find(colorName) == colorMap.end()) {
        throw std::invalid_argument("Color name not found in predefined colors: " + colorName);
    }
    SDL_Color color = colorMap[colorName];
    renderDiagram(diagram, color);
}

void SimulationViewer::addDiagram(const std::vector<std::unique_ptr<Diagram>>& diagrams, const std::string& colorName) {
    if (colorMap.find(colorName) == colorMap.end()) {
        throw std::invalid_argument("Color name not found in predefined colors: " + colorName);
    }
    SDL_Color color = colorMap[colorName];
    for (const auto& diagram : diagrams) {
        renderDiagram(diagram.get(), color);
    }
}

void SimulationViewer::removeDiagram(const Diagram* diagram) {
    auto it = diagramTextures.find(diagram);
    if (it != diagramTextures.end()) {
        SDL_DestroyTexture(it->second);
        diagramTextures.erase(it);
    }
    currentDiagrams.erase(diagram);
}

void SimulationViewer::reset(int newRectangleWidth, int newRectangleHeight, const std::vector<std::pair<const Diagram*, std::string>>& diagrams, bool newDisplayWindow) {
    rectangleWidth = newRectangleWidth;
    rectangleHeight = newRectangleHeight;

    for (auto& [diagram, texture] : diagramTextures) {
        SDL_DestroyTexture(texture);
    }
    diagramTextures.clear();

    for (const auto& [diagram, colorName] : diagrams) {
        addDiagram(diagram, colorName);
    }

    displayWindow = newDisplayWindow;

    if (!renderer) {
        throw std::runtime_error("Renderer not initialized");
    }

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
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    int rectX = (screenWidth - rectangleWidth) / 2;
    int rectY = (screenHeight - rectangleHeight) / 2;
    SDL_Rect rect = {rectX, rectY, rectangleWidth, rectangleHeight};
    SDL_RenderFillRect(renderer, &rect);
}

void SimulationViewer::render(const std::vector<std::vector<float>>& points, 
                              const std::vector<std::tuple<float, float, float, float>>& arrows) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // Set background to black
    SDL_RenderClear(renderer);

    drawBackground();
    drawGrid();

    for (const auto& [diagram, texture] : diagramTextures) {
        const auto& position = diagram->q;

        // Calculate target rectangle for rendering
        SDL_Rect targetRect;
        float width  = diagram->radius * 2 * unit;
        float height = diagram->radius * 2 * unit;
        // targetRect.x = static_cast<int>(position[0] * unit + width / 2);
        // targetRect.y = static_cast<int>(-position[1] * unit + height / 2);
        targetRect.x = static_cast<int>(position[0] * unit + screenWidth / 2 - width / 2);
        targetRect.y = static_cast<int>(-position[1] * unit + screenHeight / 2 - height / 2);
        targetRect.w = static_cast<int>(width);
        targetRect.h = static_cast<int>(height);

        // Render the texture at the updated position
        SDL_RenderCopyEx(renderer, texture, nullptr, &targetRect, -position[2] * 180 / M_PI, nullptr, SDL_FLIP_NONE);
    }

    // Draw points and arrows
    drawPoints(points, "green", 10);
    drawArrows(arrows, "yellow", 5);

    if (displayWindow) {
        SDL_RenderPresent(renderer);
    }
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
