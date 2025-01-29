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
    Eigen::Vector2f funcDiagram(float theta) const;
    Eigen::Vector2f funcGradient(float theta) const;
    Eigen::Vector2f point(float theta) const;
    std::vector<Eigen::Vector2f> points(int npts = 300, float tmin = 0, float trange = 2 * M_PI) const;
    Eigen::Vector2f tangentVector(float theta) const;
    Eigen::Vector2f normalVector(float theta) const;
    Eigen::Vector2f localVelocity(float theta) const;
    Eigen::MatrixXf localVelocityGrad(float theta, float dt, const Eigen::MatrixXf& dv = Eigen::MatrixXf::Identity(3, 3)) const;
    void genLimitConstant();
    Eigen::Vector2f rotVector(float theta) const;
    std::vector<float> cal_collision_data(const Diagram& diagram2) const;
};

#endif // DIAGRAM_H
