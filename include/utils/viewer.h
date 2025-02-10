#ifndef SIMULATION_VIEWER_H
#define SIMULATION_VIEWER_H

#include "diagram/diagram.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include <bitset>

class SimulationViewer {
private:
    int screenWidth;
    int screenHeight;
    float unit; // meters per pixel
    bool showGrid;
    float gridSpacingMeters; // Moved above
    bool displayWindow;
    int tableWidth;
    int tableHeight;
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
        {"purple", {180, 85, 165, 255}},
        {"white", {255, 255, 255, 255}},
        {"black", {0, 0, 0, 255}},
        {"pink", {255, 194, 205, 255}},
        {"lightpurple", {137, 119, 173, 255}},
    };
    std::unordered_map<SDL_Keycode, bool> keyStates;

    void drawGrid() const;
    void drawBackground() const;

    // Internal helper functions for drawing
    void drawPoints(const std::vector<std::vector<float>>& points, const std::string& colorName = "red", int thickness = 5) const;
    void drawArrows(const std::vector<std::tuple<float, float, float, float>>& arrows, const std::string& colorName = "blue", int thickness = 5) const;

public:
    SimulationViewer(int width, int height, float unit, float tableWidth, float tableHeight, bool showGrid = true, float gridSpacingMeters = 1.0f, bool displayWindow = true);
    ~SimulationViewer();

    void setGridVisibility(bool visibility);
    void setGridSpacing(float spacing);
    // Add a single diagram
    void addDiagram(const Diagram* diagram, const std::string& colorName);
    // Add multiple diagrams
    void addDiagram(const std::vector<std::unique_ptr<Diagram>>& diagrams, const std::string& colorName);
    void renderDiagram(const Diagram* diagram, const SDL_Color& color);
    void removeDiagram(const Diagram* diagram);
    void changeDiagramColor(const Diagram* diagram, const std::string& newColorName);
    void changeDiagramColor(const std::vector<std::unique_ptr<Diagram>>& diagrams, const std::string& newColorName);
    void reset(float newTableWidth, float newTableHeight, bool newDisplayWindow);
    // void render();
    void render(const std::vector<std::vector<float>>& points = {}, 
                const std::vector<std::tuple<float, float, float, float>>& arrows = {});
    SDL_Surface* getRenderedImage();
    std::tuple<std::array<float, 5>, bool, bool> getKeyboardInput();
};

#endif // SIMULATION_VIEWER_H
