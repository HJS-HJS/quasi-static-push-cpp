#include "simulation/quasi_state_sim.h"

QuasiStateSim::QuasiStateSim(int n_steps,
                             float threshold,
                             std::shared_ptr<ParamFunction> param
                             )
    : n_steps(n_steps),
      threshold(threshold),
      param(param) {}

void QuasiStateSim::filterParameters() {

    std::vector<int> thres_idx;
    for (int i = 0; i < param->phi.size(); ++i) {
        if ((param->phi(i) < threshold / 5) && (param->phi(i) != 0)) {
            thres_idx.push_back(i);
        }
    }

    int num_filtered = thres_idx.size();
    if (num_filtered == 0) {
        phi_filtered = Eigen::VectorXf();
        return;
    }

    std::vector<int> thres_idx_twice;
    for (int idx : thres_idx) {
        thres_idx_twice.push_back(idx * 2);
        thres_idx_twice.push_back(idx * 2 + 1);
    }

    phi_filtered.resize(num_filtered);
    JNS_filtered.resize(num_filtered, param->m_JNS.cols());
    JNP_filtered.resize(num_filtered, param->m_JNP.cols());
    JTS_filtered.resize(thres_idx_twice.size(), param->m_JTS.cols());
    JTP_filtered.resize(thres_idx_twice.size(), param->m_JTP.cols());
    mu_filtered.resize(num_filtered, num_filtered);

    for (int i = 0; i < num_filtered; ++i) {
        int idx = thres_idx[i];
        phi_filtered(i) = param->phi(idx);
        JNS_filtered.row(i) = param->m_JNS.row(idx);
        JNP_filtered.row(i) = param->m_JNP.row(idx);
    }

    for (int i = 0; i < thres_idx_twice.size(); ++i) {
        JTS_filtered.row(i) = param->m_JTS.row(thres_idx_twice[i]);
        JTP_filtered.row(i) = param->m_JTP.row(thres_idx_twice[i]);
    }

    mu_filtered = param->m_mu.block(0, 0, num_filtered, num_filtered);
}

std::tuple<std::vector<float>, std::vector<float>, bool> QuasiStateSim::run(const Eigen::VectorXf& u_input, bool perfect_u_control) {
    Eigen::VectorXf qs = param->qs;
    Eigen::VectorXf qp = param->qp;

    // 1. 실행 직전에 `phi` 값 기준으로 필터링
    filterParameters();

    int n_c = phi_filtered.size();
    int l = n_c * 3;

    if (n_c == 0) {
        std::vector<float> qs_vec(qs.data(), qs.data() + qs.size());
        std::vector<float> qp_vec(qp.data(), qp.data() + qp.size());
        for (size_t i = 0; i < qp_vec.size(); ++i) qp_vec[i] += u_input(i);
        return {qs_vec, qp_vec, true};
    }


    Eigen::MatrixXf E = Eigen::MatrixXf::Identity(n_c, n_c).replicate(2, 1);
    Eigen::MatrixXf ZE = Eigen::MatrixXf::Zero(2 * n_c, n_c);
    Eigen::MatrixXf Z = Eigen::MatrixXf::Zero(n_c, n_c);

    Eigen::MatrixXf M = Eigen::MatrixXf::Zero(4 * n_c, 4 * n_c);
    Eigen::VectorXf w = Eigen::VectorXf::Zero(4 * n_c);

    Eigen::MatrixXf JS(JNS_filtered.rows() + JTS_filtered.rows(), JNS_filtered.cols());
    JS << JNS_filtered, JTS_filtered;

    Eigen::MatrixXf JP(JNP_filtered.rows() + JTP_filtered.rows(), JNP_filtered.cols());
    JP << JNP_filtered, JTP_filtered;

    M.block(0, 0, l, l) = JS * (param->m_A * JS.transpose());
    M.block(0, l, n_c, n_c) = Z + Z;
    M.block(n_c, l, 2 * n_c, n_c) = E + ZE;
    M.block(l, 0, n_c, n_c) = mu_filtered + Z;
    M.block(l, n_c, n_c, 2 * n_c) = -E.transpose() + ZE.transpose();
    M.block(l, l, n_c, n_c) = 2 * Z;

    if (!perfect_u_control) {
        M.block(0, 0, l, l) += JP * (param->m_B * JP.transpose());
    }

    w.head(l) = JP * u_input;
    w.head(n_c) += phi_filtered;

    // LCP Solver 호출
    std::vector<std::vector<float>> M_std(M.rows(), std::vector<float>(M.cols()));
    for (int i = 0; i < M.rows(); ++i)
        for (int j = 0; j < M.cols(); ++j)
            M_std[i][j] = M(i, j);

    std::vector<float> w_std(w.data(), w.data() + w.size());
    LCPSolver solver(M, w, n_steps);
    auto sol = solver.solve();

    // Eigen Vector가 비어 있는지 확인
    if (sol.first.size() == 0) {
        Eigen::VectorXf qp_vec_eigen = qp + u_input; // Eigen 스타일 벡터 연산 (성능 최적화)
        std::vector<float> qp_vec(qp_vec_eigen.data(), qp_vec_eigen.data() + qp_vec_eigen.size());
        return {qp_vec, qp_vec, false};
    }

    // Eigen 벡터 변환
    Eigen::VectorXf sol_eigen = Eigen::Map<Eigen::VectorXf>(sol.first.data(), sol.first.size());

    // `qs`를 std::vector로 변환하고 업데이트
    Eigen::VectorXf q_s_eigen = qs + param->m_A * (JS.transpose() * sol.first.head(l)); // Eigen 행렬 연산 최적화
    std::vector<float> q_s(q_s_eigen.data(), q_s_eigen.data() + q_s_eigen.size());

    // `qp`를 std::vector로 변환하고 업데이트
    Eigen::VectorXf q_p_eigen = qp + u_input;
    if(!perfect_u_control){
        q_p_eigen += param->m_B * (JP.transpose() * sol.first.head(l));
    }
    std::vector<float> q_p(q_p_eigen.data(), q_p_eigen.data() + q_p_eigen.size());

    return {q_s, q_p, true};
    }
