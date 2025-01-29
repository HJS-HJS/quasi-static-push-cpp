#ifndef SIMULATION_VIEWER_H
#define SIMULATION_VIEWER_H

#include "diagram/diagram.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>

class SimulationViewer {
private:
    int screenWidth;
    int screenHeight;
    float unit; // meters per pixel
    bool showGrid;
    bool displayWindow;
    float gridSpacingMeters; // Moved above
    int rectangleWidth;
    int rectangleHeight;
    SDL_Window* window; // Moved below
    SDL_Renderer* renderer;
    std::unordered_map<const Diagram*, SDL_Color> currentDiagrams;
    std::unordered_map<const Diagram*, SDL_Texture*> diagramTextures; // Store pre-rendered textures

    // Predefined colors
    std::unordered_map<std::string, SDL_Color> colorMap = {
        {"red", {255, 0, 0, 255}},
        {"green", {0, 255, 0, 255}},
        {"blue", {0, 0, 255, 255}},
        {"yellow", {255, 255, 0, 255}},
        {"white", {255, 255, 255, 255}},
        {"black", {0, 0, 0, 255}}
    };

    void drawGrid() const;
    void drawBackground() const;

    // Internal helper functions for drawing
    void drawPoints(const std::vector<std::vector<float>>& points, const std::string& colorName = "red", int thickness = 5) const;
    void drawArrows(const std::vector<std::tuple<float, float, float, float>>& arrows, const std::string& colorName = "blue", int thickness = 5) const;

public:
    SimulationViewer(int width, int height, float unit, bool showGrid = true, bool displayWindow = true, float gridSpacingMeters = 1.0f);
    ~SimulationViewer();

    void setGridVisibility(bool visibility);
    void setGridSpacing(float spacing);
    // Add a single diagram
    void addDiagram(const Diagram* diagram, const std::string& colorName);
    // Add multiple diagrams
    void addDiagram(const std::vector<std::unique_ptr<Diagram>>& diagrams, const std::string& colorName);
    void renderDiagram(const Diagram* diagram, const SDL_Color& color);
    void removeDiagram(const Diagram* diagram);
    void reset(int newRectangleWidth, int newRectangleHeight, const std::vector<std::pair<const Diagram*, std::string>>& diagrams, bool newDisplayWindow);
    // void render();
    void render(const std::vector<std::vector<float>>& points = {}, 
                const std::vector<std::tuple<float, float, float, float>>& arrows = {});
    SDL_Surface* getRenderedImage();
};

#endif // SIMULATION_VIEWER_H
