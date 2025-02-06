#ifndef SMOOTHRPOLYGON_H
#define SMOOTHRPOLYGON_H

#include "diagram/diagram.h"
#include <cmath>

class SmoothRPolygon : public Diagram {
public:
    SmoothRPolygon(float x, float y, float rotation, float a, int k);
    float funcRadius(float theta) const override;
    float funcRadiusD(float theta) const override;
};

#endif // SMOOTHRPOLYGON_H