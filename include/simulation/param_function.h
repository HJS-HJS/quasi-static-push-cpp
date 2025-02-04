#ifndef PARAM_FUNCTION_H
#define PARAM_FUNCTION_H

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <Eigen/Dense>
#include <iostream>
#include "diagram/diagram_all.h"
#include "simulation/object_pusher.h"
#include "simulation/object_slider.h"

class ParamFunction {
public:
    ParamFunction(ObjectSlider& sliders, ObjectPusher& pushers, ObjectSlider& obstacles,
                  float threshold = 1e-2, float fmscale = 0.2, float fascale = 0.9, float fbscale = 0.001);

    void update_param();

    Eigen::VectorXf qs;
    Eigen::VectorXf qp;
    Eigen::VectorXf phi;
    Eigen::MatrixXf nhat;
    Eigen::MatrixXf vc_jac;
    Eigen::MatrixXf m_JN;
    Eigen::MatrixXf m_JT;
    
    Eigen::MatrixXf m_JNS;
    Eigen::MatrixXf m_JNP;
    Eigen::MatrixXf m_JTS;
    Eigen::MatrixXf m_JTP;

    Eigen::MatrixXf m_mu;
    Eigen::MatrixXf m_A;
    Eigen::MatrixXf m_B;

private:
    ObjectSlider& sliders;
    ObjectPusher& pushers;
    ObjectSlider& obstacles;

    float threshold;
    float fmscale;
    float fascale;
    float fbscale;

    int n_phi;

    bool is_collision_available(const Diagram& diagram1, const Diagram& diagram2, float threshold);
    int combination(int n, int r);

    std::vector<float> q();
    std::vector<float> v();
};

#endif