#include "diagram/superellipse.h"

SuperEllipse::SuperEllipse(float x, float y, float rotation, float a, float b, int n)
    : Diagram(x, y, rotation, sqrt(pow(a, 2) + pow(b, 2))) {
        shape_params = std::vector<float>{a, b, (float)n};
        initialize();
    }

float SuperEllipse::funcRadius(float theta) const {
    return pow(pow(fabs(cos(theta) / shape_params[0]), shape_params[2]) + pow(fabs(sin(theta) / shape_params[1]), shape_params[2]), -1.0f / shape_params[2]);
}

float SuperEllipse::funcRadiusD(float theta) const {
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    float term1 = pow(fabs(cos_theta / shape_params[0]), shape_params[2]);
    float term2 = pow(fabs(sin_theta / shape_params[1]), shape_params[2]);
    float g_theta = (-1.0f / shape_params[2]) * pow(term1 + term2, -1.0f / shape_params[2] - 1);
    float dg_dtheta = shape_params[2] * pow(fabs(cos_theta / shape_params[0]), shape_params[2] - 1) * (-sin_theta / shape_params[0]) * (cos_theta >= 0 ? 1 : -1) +
                      shape_params[2] * pow(fabs(sin_theta / shape_params[1]), shape_params[2] - 1) * (cos_theta / shape_params[1]) * (sin_theta >= 0 ? 1 : -1);
    return g_theta * dg_dtheta;
}