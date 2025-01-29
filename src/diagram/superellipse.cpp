#include "diagram/superellipse.h"

SuperEllipse::SuperEllipse(float x, float y, float rotation, float a, float b, int n)
    : Diagram(x, y, rotation, sqrt(pow(a, 2) + pow(b, 2))), a(a), b(b), n(n) {
        initialize();
    }

float SuperEllipse::funcRadius(float theta) const {
    return pow(pow(fabs(cos(theta) / a), n) + pow(fabs(sin(theta) / b), n), -1.0f / n);
}

float SuperEllipse::funcRadiusD(float theta) const {
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    float term1 = pow(fabs(cos_theta / a), n);
    float term2 = pow(fabs(sin_theta / b), n);
    float g_theta = (-1.0f / n) * pow(term1 + term2, -1.0f / n - 1);
    float dg_dtheta = n * pow(fabs(cos_theta / a), n - 1) * (-sin_theta / a) * (cos_theta >= 0 ? 1 : -1) +
                      n * pow(fabs(sin_theta / b), n - 1) * (cos_theta / b) * (sin_theta >= 0 ? 1 : -1);
    return g_theta * dg_dtheta;
}