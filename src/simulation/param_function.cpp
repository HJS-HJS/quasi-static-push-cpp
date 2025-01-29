#include "simulation/param_function.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <iostream>

ParamFunction::ParamFunction(ObjectSlider& sliders, ObjectPusher& pushers, ObjectSlider& obstacles,
                             float threshold, float fmscale, float fascale, float fbscale)
    : sliders(sliders), pushers(pushers), obstacles(obstacles),
      threshold(threshold), fmscale(fmscale), fascale(fascale), fbscale(fbscale) {

    n_phi = pushers.size() * sliders.size() + combination(sliders.size(), 2) +
            (pushers.size() + sliders.size()) * obstacles.size();

    phi = Eigen::VectorXf::Zero(n_phi);
    nhat = Eigen::MatrixXf::Zero(n_phi, 2);
    vc = Eigen::MatrixXf::Zero(n_phi, 2);
    vc_jac = Eigen::MatrixXf::Zero(n_phi * 2, v().size());

    m_JN = Eigen::MatrixXf::Zero(n_phi, q().size());
    m_JT = Eigen::MatrixXf::Zero(2 * n_phi, q().size());
}

void ParamFunction::update_param() {
    std::cout<<"check point 0"<<std::endl;
    phi.setZero();
    nhat.setZero();
    vc.setZero();
    vc_jac.setZero();
    m_JN.setZero();
    m_JT.setZero();
    std::cout<<"check point 1"<<std::endl;

    float _dt = 0.0001;
    auto pusher_dv = pushers.pusher_dv(_dt);
    std::cout<<"check point 2"<<std::endl;

    int i = -1;
    // int n_slider = sliders.size();

    for (const auto& sliderPtr : sliders) {
        auto& slider = *sliderPtr; // Dereference unique_ptr once
        auto i_s = std::distance(sliders.begin(), std::find(sliders.begin(), sliders.end(), sliderPtr));

        for (const auto& pusherPtr : pushers) {
            auto& pusher = *pusherPtr; // Dereference unique_ptr once
            auto i_p = std::distance(pushers.begin(), std::find(pushers.begin(), pushers.end(), pusherPtr));
            ++i;

            if (i_s != 0 && !is_collision_available(slider, pusher, threshold)) continue;

            auto ans = slider.cal_collision_data(pusher);
            phi[i] = ans[2];
            nhat.row(i) = slider.normalVector(ans[0]);
            vc_jac.block<2, 3>(2 * i, 3 * i_s) = -slider.localVelocityGrad(ans[0], _dt).transpose() / _dt;
            vc_jac.block<2, 4>(2 * i, 3 * sliders.size()) = pusher.localVelocityGrad(ans[1], _dt, pusher_dv[i_p]).transpose() / _dt;
        }
    }
    std::cout<<"check point 3"<<std::endl;

    for (auto slider1_it = sliders.begin(); slider1_it != sliders.end(); ++slider1_it) {
        auto& slider1 = **slider1_it; // Dereference the unique_ptr to get the actual Diagram
        auto i_s1 = std::distance(sliders.begin(), slider1_it); // Get index of slider1

        for (auto slider2_it = std::next(slider1_it); slider2_it != sliders.end(); ++slider2_it) {
            auto& slider2 = **slider2_it; // Dereference the unique_ptr to get the actual Diagram
            auto i_s2 = std::distance(sliders.begin(), slider2_it); // Get index of slider2
            ++i;

            if (!is_collision_available(slider1, slider2, threshold)) continue;

            auto ans = slider1.cal_collision_data(slider2);
            phi[i] = ans[2];
            nhat.row(i) = slider1.normalVector(ans[0]);
            vc_jac.block<2, 3>(2 * i, 3 * i_s1) = -slider1.localVelocityGrad(ans[0], _dt).transpose() / _dt;
            vc_jac.block<2, 3>(2 * i, 3 * i_s2) = slider2.localVelocityGrad(ans[1], _dt).transpose() / _dt;
        }
    }
    std::cout<<"check point 5"<<std::endl;
    std::cout<<vc_jac<<std::endl;


    Eigen::Matrix2f _rot;
    _rot << 0, -1, 1, 0;
    for (int i = 0; i < n_phi; ++i) {
        Eigen::MatrixXf vc_jac_block = vc_jac.block(2 * i, 0, 2, vc_jac.cols());
        m_JN.row(i) = nhat.row(i) * vc_jac_block;
        m_JT.row(2 * i) = -(_rot * nhat.row(i).transpose()).transpose() * vc_jac_block;
        m_JT.row(2 * i + 1) = (_rot * nhat.row(i).transpose()).transpose() * vc_jac_block;
    }
    std::cout<<"check point 6"<<std::endl;
}

bool ParamFunction::is_collision_available(const Diagram& diagram1, const Diagram& diagram2, float threshold) {
    return (diagram1.q - diagram2.q).head<2>().norm() - diagram1.radius - diagram2.radius < threshold;
}

int ParamFunction::combination(int n, int r) {
    if (n < r) return 0;
    return static_cast<int>(std::tgamma(n + 1) / (std::tgamma(r + 1) * std::tgamma(n - r + 1)));
}

Eigen::VectorXf ParamFunction::q() {
    Eigen::VectorXf qs = sliders.get_q();
    Eigen::VectorXf qp = pushers.q;
    return Eigen::VectorXf(qs.size() + qp.size());
}

Eigen::VectorXf ParamFunction::v() {
    Eigen::VectorXf vs = sliders.get_v();
    Eigen::VectorXf vp = pushers.v;
    return Eigen::VectorXf(vs.size() + vp.size());
}
