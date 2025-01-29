#include "diagram/ellipse.h"

Ellipse::Ellipse(float x, float y, float rotation, float a, float b)
    : Diagram(x, y, rotation, sqrt(pow(a, 2) + pow(b, 2))), a(a), b(b) {
        initialize();
    }

float Ellipse::funcRadius(float theta) const {
    return pow(pow(cos(theta) / a, 2) + pow(sin(theta) / b, 2), -0.5f);
}

float Ellipse::funcRadiusD(float theta) const {
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    float g_theta = -0.5f * pow(pow(cos_theta / a, 2) + pow(sin_theta / b, 2), -1.5f);
    float dg_dtheta = -2 * (cos_theta * sin_theta / (a * a)) + 2 * (sin_theta * cos_theta / (b * b));
    return g_theta * dg_dtheta;
}