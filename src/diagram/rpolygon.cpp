#include "diagram/rpolygon.h"

RPolygon::RPolygon(float x, float y, float rotation, float a, int n)
    : Diagram(x, y, rotation, a) {
        shape_params = std::vector<float>{a, (float)n};
        initialize();
    }

float RPolygon::funcRadius(float theta) const {
    return cos(M_PI / shape_params[1]) / cos(fmod(theta, 2 * M_PI / shape_params[1]) - M_PI / shape_params[1]) * shape_params[0];
}

float RPolygon::funcRadiusD(float theta) const {
    float theta_mod = fmod(theta, 2 * M_PI / shape_params[1]) - M_PI / shape_params[1];
    return shape_params[0] * cos(M_PI / shape_params[1]) * sin(theta_mod) / pow(cos(theta_mod), 2);
}
