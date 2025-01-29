#include "diagram/smoothrpolygon.h"

SmoothRPolygon::SmoothRPolygon(float x, float y, float rotation, float a, int k)
    : Diagram(x, y, rotation, a), a(a), b(a / (k * 5.0f)), k(k) {
        initialize();
    }

float SmoothRPolygon::funcRadius(float theta) const {
    return a + b * cos(k * theta);
}

float SmoothRPolygon::funcRadiusD(float theta) const {
    return -k * b * sin(k * theta);
}