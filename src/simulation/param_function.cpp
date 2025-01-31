#include "simulation/param_function.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>

ParamFunction::ParamFunction(ObjectSlider& sliders, ObjectPusher& pushers, ObjectSlider& obstacles,
                             float threshold, float fmscale, float fascale, float fbscale)
    : sliders(sliders), pushers(pushers), obstacles(obstacles),
      threshold(threshold), fmscale(fmscale), fascale(fascale), fbscale(fbscale) {

    n_phi = pushers.size() * sliders.size() + combination(sliders.size(), 2) +
            (pushers.size() + sliders.size()) * obstacles.size();

    phi = Eigen::VectorXf::Zero(n_phi);
    nhat = Eigen::MatrixXf::Zero(n_phi, 2);
    vc_jac = Eigen::MatrixXf::Zero(n_phi * 2, v().size());
    m_JN = Eigen::MatrixXf::Zero(n_phi, q().size());
    m_JT = Eigen::MatrixXf::Zero(2 * n_phi, q().size());

    m_mu = Eigen::MatrixXf::Identity(n_phi, n_phi) * fmscale;
    m_A  = Eigen::MatrixXf::Zero(3 * sliders.size(), 3 * sliders    .size());
    m_B  = Eigen::MatrixXf::Identity(pushers.q.size(), pushers.q.size()) * fbscale;
    
    for (size_t idx = 0; idx < sliders.size(); ++idx) {
        const auto& slider = sliders[idx];
        m_A.block<3, 3>(3 * idx, 3 * idx) = slider->limit_constant * fascale;
    }
    m_B.block(2, 2, m_B.rows() - 2, m_B.cols() - 2) *= 0.01;
}

void ParamFunction::update_param() {
    std::cout << "check point 0" << std::endl;
    phi.setZero();
    nhat.setZero();
    vc_jac.setZero();
    m_JN.setZero();
    m_JT.setZero();

    std::cout << "check point 1" << std::endl;
    int n_slider = sliders.size();
    float _dt = 0.0001;
    auto pusher_dv = pushers.pusher_dv(_dt);

    std::cout << "check point 2" << std::endl;

    int i = -1;
    for (const auto& sliderPtr : sliders) {
        auto& slider = *sliderPtr;
        size_t i_s = std::distance(sliders.begin(), std::find(sliders.begin(), sliders.end(), sliderPtr));

        for (const auto& pusherPtr : pushers) {
            auto& pusher = *pusherPtr;
            size_t i_p = std::distance(pushers.begin(), std::find(pushers.begin(), pushers.end(), pusherPtr));
            ++i;

            if (!is_collision_available(slider, pusher, threshold)) continue;

            auto ans = slider.cal_collision_data(pusher);
            phi(i) = ans[2];
            auto normal_ = slider.normalVector(ans[0]);
            nhat.row(i) << normal_[0], normal_[1];

            Eigen::MatrixXf slider_grad = slider.localVelocityGrad(ans[0], _dt);
            Eigen::MatrixXf pusher_grad = pusher.localVelocityGrad(ans[1], _dt, pusher_dv[i_p]);

            vc_jac.block(2 * i, 3 * i_s, 2, 3) = slider_grad.transpose() * -1;
            vc_jac.block(2 * i, 3 * n_slider, 2, 4) = pusher_grad.transpose();
        }
    }

    // std::cout << "check point 3" << std::endl;
    for (auto slider1_it = sliders.begin(); slider1_it != sliders.end(); ++slider1_it) {
        auto& slider1 = **slider1_it;
        auto i_s1 = std::distance(sliders.begin(), slider1_it);

        for (auto slider2_it = std::next(slider1_it); slider2_it != sliders.end(); ++slider2_it) {
            auto& slider2 = **slider2_it;
            auto i_s2 = std::distance(sliders.begin(), slider2_it);
            ++i;

            if (!is_collision_available(slider1, slider2, threshold)) continue;

            auto ans = slider1.cal_collision_data(slider2);
            phi(i) = ans[2];
            auto normal_ = slider1.normalVector(ans[0]);
            nhat.row(i) << normal_[0], normal_[1];

            Eigen::MatrixXf slider1_grad = slider1.localVelocityGrad(ans[0], _dt);
            Eigen::MatrixXf slider2_grad = slider2.localVelocityGrad(ans[1], _dt);

            vc_jac.block(2 * i, 3 * i_s1, 2, 3) = slider1_grad.transpose() * -1;
            vc_jac.block(2 * i, 3 * i_s2, 2, 3) = slider2_grad.transpose();
        }
    }

    std::cout << "check point 5" << std::endl;

    // for (size_t i = 0; i < n_phi; ++i) {
    //     Eigen::Vector2f _rot(0, -1);
    //     m_JN.row(i) = nhat.row(i).dot(vc_jac.row(i).head<2>());
    //     m_JT.row(2 * i) = Eigen::Vector2f(-_rot.y() * nhat(i, 0) + _rot.x() * nhat(i, 1),
    //                                        _rot.y() * nhat(i, 1) + _rot.x() * nhat(i, 0));
    // }
    Eigen::Matrix2f _rot;
    _rot <<  0, -1,
            1,  0;
    for (size_t i = 0; i < n_phi; ++i) {
        m_JN.row(i) = nhat.row(i) * vc_jac.block(2 * i, 0, 2, vc_jac.cols());
        Eigen::Vector2f rotated_nhat = _rot * nhat.row(i).transpose();

        m_JT.row(2 * i) = -rotated_nhat.transpose() * vc_jac.block(2 * i, 0, 2, vc_jac.cols());
        m_JT.row(2 * i + 1) = rotated_nhat.transpose() * vc_jac.block(2 * i, 0, 2, vc_jac.cols());
    }
}

bool ParamFunction::is_collision_available(const Diagram& diagram1, const Diagram& diagram2, float threshold) {
    return std::hypot(diagram1.q[0] - diagram2.q[0], diagram1.q[1] - diagram2.q[1]) - diagram1.radius - diagram2.radius < threshold;
}

int ParamFunction::combination(int n, int r) {
    if (n < r) return 0;
    return static_cast<int>(std::tgamma(n + 1) / (std::tgamma(r + 1) * std::tgamma(n - r + 1)));
}

std::vector<float> ParamFunction::q() {
    std::vector<float> qs = sliders.get_q();
    std::array<float,4> qp = pushers.q;
    // std::vector<float> qp = pushers.q;
    qs.insert(qs.end(), qp.begin(), qp.end());
    return qs;
}

std::vector<float> ParamFunction::v() {
    std::vector<float> vs = sliders.get_v();
    std::array<float,4> vp = pushers.q;
    // std::vector<float> vp = pushers.v;
    vs.insert(vs.end(), vp.begin(), vp.end());
    return vs;
}
