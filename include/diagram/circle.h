#ifndef CIRCLE_H
#define CIRCLE_H

#include "diagram/diagram.h"
#include <cmath>

class Circle : public Diagram {
public:
    Circle(float x, float y, float rotation, float radius);
    float funcRadius(float theta) const override;
};

#endif // CIRCLE_H