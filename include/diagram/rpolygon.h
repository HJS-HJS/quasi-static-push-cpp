#ifndef RPOLYGON_H
#define RPOLYGON_H

#include "diagram/diagram.h"
#include <cmath>

class RPolygon : public Diagram {
public:
    RPolygon(float x, float y, float rotation, float a, int n);
    float funcRadius(float theta) const override;
    float funcRadiusD(float theta) const override;
};

#endif // RPOLYGON_H