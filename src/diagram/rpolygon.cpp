#include "diagram/rpolygon.h"

RPolygon::RPolygon(float x, float y, float rotation, float a, int n)
    : Diagram(x, y, rotation, a), a(a), n(n) {
        initialize();
    }

float RPolygon::funcRadius(float theta) const {
    return cos(M_PI / n) / cos(fmod(theta, 2 * M_PI / n) - M_PI / n) * a;
}

float RPolygon::funcRadiusD(float theta) const {
    float theta_mod = fmod(theta, 2 * M_PI / n) - M_PI / n;
    return a * cos(M_PI / n) * sin(theta_mod) / pow(cos(theta_mod), 2);
}
