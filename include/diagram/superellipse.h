#ifndef SUPERELLIPSE_H
#define SUPERELLIPSE_H

#include "diagram/diagram.h"
#include <cmath>

class SuperEllipse : public Diagram {
public:
    SuperEllipse(float x, float y, float rotation, float a, float b, int n);
    float funcRadius(float theta) const override;
    float funcRadiusD(float theta) const override;
};

#endif // SUPERELLIPSE_H