#ifndef ELLIPSE_H
#define ELLIPSE_H

#include "diagram/diagram.h"
#include <cmath>

class Ellipse : public Diagram {
public:
    Ellipse(float x, float y, float rotation, float a, float b);
    float funcRadius(float theta) const override;
    float funcRadiusD(float theta) const override;
};

#endif // ELLIPSE_H