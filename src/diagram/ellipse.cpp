#include "diagram/ellipse.h"

Ellipse::Ellipse(float x, float y, float rotation, float a, float b)
    : Diagram(x, y, rotation, sqrt(pow(a, 2) + pow(b, 2))) {
        shape_params = std::vector<float>{a, b};
        initialize();
    }

float Ellipse::funcRadius(float theta) const {
    return pow(pow(cos(theta) / shape_params[0], 2) + pow(sin(theta) / shape_params[1], 2), -0.5f);
}

float Ellipse::funcRadiusD(float theta) const {
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    float g_theta = -0.5f * pow(pow(cos_theta / shape_params[0], 2) + pow(sin_theta / shape_params[1], 2), -1.5f);
    float dg_dtheta = -2 * (cos_theta * sin_theta / (shape_params[0] * shape_params[0])) + 2 * (sin_theta * cos_theta / (shape_params[1] * shape_params[1]));
    return g_theta * dg_dtheta;
}