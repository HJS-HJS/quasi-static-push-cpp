#include "diagram/circle.h"

Circle::Circle(float x, float y, float rotation, float radius)
    : Diagram(x, y, rotation, radius) {
        initialize();
    }

float Circle::funcRadius(float theta) const {
    return radius;
}