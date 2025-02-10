#include "diagram/smoothrpolygon.h"

SmoothRPolygon::SmoothRPolygon(float x, float y, float rotation, float a, int n)
    : Diagram(x, y, rotation, a) {
        shape_params = std::vector<float>{a, a / (n * 5.0f), (float)n};
        initialize();
    }

float SmoothRPolygon::funcRadius(float theta) const {
    return shape_params[0] + shape_params[1] * cos(shape_params[2] * theta);
}

float SmoothRPolygon::funcRadiusD(float theta) const {
    return -shape_params[2] * shape_params[1] * sin(shape_params[2] * theta);
}