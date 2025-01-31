#ifndef DIAGRAM_H
#define DIAGRAM_H

#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <cmath>
#include <stdexcept>

struct PolarCoordinate {
    float radius;
    float theta;
    float x;
    float y;
};

class Diagram {
private:
    std::vector<Eigen::Vector2f> data;
    float normalize(const std::array<float, 2>& v) const;
    float normalize(const std::array<float, 2>& v1, const std::array<float, 2>& v2) const;
    std::array<float, 2> subtractArrays(const std::array<float, 2>& a, const std::array<float, 2>& b) const;

public:
    Diagram();
    Diagram(float x, float y, float rotation, float radius);

    virtual ~Diagram();

    virtual float funcRadius(float theta) const; // Virtual for derived classes
    virtual float funcRadiusD(float theta) const;

    Eigen::Vector3f q; // Position and orientation
    Eigen::Vector3f v = Eigen::Vector3f::Zero(); // Velocity
    float radius;
    float dt = 0.001;
    Eigen::Matrix3f limit_constant;

    void initialize();
    std::array<float, 2> funcDiagram(float theta) const;
    std::array<float, 2> funcGradient(float theta) const;
    std::array<float, 2> point(float theta) const;
    std::vector<std::array<float, 2>> points(int npts = 300, float tmin = 0, float trange = 2 * M_PI) const;
    std::array<float, 2> tangentVector(float theta) const;
    std::array<float, 2> normalVector(float theta) const;
    std::array<float, 2> localVelocity(float theta) const;
    Eigen::MatrixXf localVelocityGrad(float theta, float dt) const;
    Eigen::MatrixXf localVelocityGrad(float theta, float dt, const std::array<std::array<float, 3>, 4>& dv) const;
    void genLimitConstant();
    std::array<float, 2> rotVector(float theta) const;
    std::array<float, 3> cal_collision_data(const Diagram& diagram2) const;
};

#endif // DIAGRAM_H
