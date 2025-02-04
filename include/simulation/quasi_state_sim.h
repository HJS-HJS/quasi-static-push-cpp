#ifndef QUASISTATESIM_H
#define QUASISTATESIM_H

#include <vector>
#include <Eigen/Dense>
#include "solver/lcp_solver.h"
#include "simulation/param_function.h"


class QuasiStateSim {
public:
    QuasiStateSim(int n_steps,
                  float threshold,
                  std::shared_ptr<ParamFunction> param,
                  bool perfect_u_control = true);

    std::tuple<std::vector<float>, std::vector<float>, bool> run(const Eigen::VectorXf& u_input);

private:
    int n_steps;
    bool perfect_u_control;
    float threshold;
    std::shared_ptr<ParamFunction> param;

    // 필터링된 데이터 (매번 갱신)
    Eigen::VectorXf phi_filtered;
    Eigen::MatrixXf JNS_filtered, JNP_filtered, JTS_filtered, JTP_filtered;
    Eigen::MatrixXf mu_filtered, A_filtered, B_filtered;

    void filterParameters();  // 매번 실행 전에 필터링
};

#endif // QUASISTATESIM_H
