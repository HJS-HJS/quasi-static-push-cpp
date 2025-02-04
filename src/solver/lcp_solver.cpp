#include "solver/lcp_solver.h"
#include <iomanip>

LCPSolver::LCPSolver(const Eigen::MatrixXf& M, const Eigen::VectorXf& q, int maxIter)
    : n(q.size()), maxIter(maxIter) {
    if (M.rows() != n || M.cols() != n) {
        throw std::invalid_argument("Matrix M must be square and match the size of vector q.");
    }

    W = 0;
    Z = 1;
    Y = 2;
    Q = 3;

    T.resize(n, 2 * n + 2);
    T.setZero();
    T.block(0, 0, n, n) = Eigen::MatrixXf::Identity(n, n);

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            T(i, j + n) = -M(i, j);
        }
        T(i, 2 * n) = -1.0f;
        T(i, 2 * n + 1) = q(i);
    }

    wPos.resize(n);
    zPos.resize(n);
    for (size_t i = 0; i < n; ++i) {
        wPos[i] = i;
        zPos[i] = i + n;
    }

    Eigen::MatrixXi TbInd(2, n);
    TbInd.row(0).setConstant(W);
    TbInd.row(1) = Eigen::VectorXi::LinSpaced(n, 0, n - 1);

    Eigen::MatrixXi TnbInd(2, n);
    TnbInd.row(0).setConstant(Z);
    TnbInd.row(1) = Eigen::VectorXi::LinSpaced(n, 0, n - 1);

    Eigen::MatrixXi DriveInd(2, 1);
    DriveInd << Y, 0;

    Eigen::MatrixXi QInd(2, 1);
    QInd << Q, 0;

    this->Tind = Eigen::MatrixXf(2, 2 * n + 2);
    this->Tind << TbInd.cast<float>(), TnbInd.cast<float>(), DriveInd.cast<float>(), QInd.cast<float>();
}

std::pair<Eigen::VectorXf, std::string> LCPSolver::solve() {
    if (!initialize()) {
        return {extractSolution(), "Solution Found"};
    }

    for (int k = 0; k < maxIter; ++k) {
        bool stepVal = step();
        if (Tind(0, 2 * n) == Y){
            return {extractSolution(), "Solution found"};
        }

        if (!stepVal) {
            if (!initialize()) {
                return {extractSolution(), "Secondary ray solution"};
            }
            return {Eigen::VectorXf::Zero(n), "Secondary ray found"};
        }
    }

    return {Eigen::VectorXf::Zero(n), "Max Iterations Exceeded"};
}

bool LCPSolver::initialize() {
    Eigen::VectorXf q = T.col(T.cols() - 1);
    float minQ = q.minCoeff();

        if (minQ < 0) {
        int ind;
        q.minCoeff(&ind);
        clearDriverColumn(ind);
        pivot(ind);
        return true;
    }
    return false;
}

bool LCPSolver::step() {
    size_t pivotRow = std::numeric_limits<size_t>::max();
    float minRatio = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < n; ++i) {
        if (T(i, 2 * n) > 1e-6f) {
            float ratio = T(i, 2 * n + 1) / T(i, 2 * n);

            if (std::isnan(ratio) || !std::isfinite(ratio)) continue;

            if (ratio < minRatio) {
                minRatio = ratio;
                pivotRow = i;
            }
        }
    }
    if (pivotRow == std::numeric_limits<size_t>::max()) return false;

    clearDriverColumn(pivotRow);
    pivot(pivotRow);
    return true;
}

Eigen::VectorXf LCPSolver::extractSolution() const {
    Eigen::VectorXf z = Eigen::VectorXf::Zero(n);
    Eigen::VectorXf q = T.col(T.cols() - 1);

    for (size_t i = 0; i < n; ++i) {
        if (static_cast<int>(Tind(0, i)) == Z) {
            z(static_cast<int>(Tind(1, i))) = q(i);
        }
    }
    return z;
}

std::optional<int> LCPSolver::partnerPos(size_t pos) {
    int v = static_cast<int>(Tind(0, pos));
    int ind = static_cast<int>(Tind(1, pos));

    if (v == W) return static_cast<int>(zPos[ind]);
    if (v == Z) return static_cast<int>(wPos[ind]);
    return std::nullopt;
}

bool LCPSolver::pivot(size_t pos) {
    auto pposOpt = partnerPos(pos);
    if (pposOpt.has_value()) {
        int ppos = pposOpt.value();
        swapColumns(pos, ppos);
        swapColumns(pos, 2 * n);
        return true;
    } else {
        swapColumns(pos, 2 * n);
        return false;
    }
}

void LCPSolver::swapPos(int v, int ind, int newPos) {
    if (v == W) {
        wPos[ind] = newPos % (2 * n + 2);
    } else if (v == Z) {
        zPos[ind] = newPos % (2 * n + 2);
    }
}

void LCPSolver::swapColumns(size_t i, size_t j) {
    Eigen::Vector2f iInd = Tind.col(i);
    Eigen::Vector2f jInd = Tind.col(j);

    // i → j
    swapPos(iInd[0], iInd[1], j);
    // j → i
    swapPos(jInd[0], jInd[1], i);

    Tind.col(i).swap(Tind.col(j));
    T.col(i).swap(T.col(j));
}


void LCPSolver::clearDriverColumn(size_t ind) {
    float divisor = T(ind, 2 * n);
    if (std::abs(divisor) < 1e-6f) {
        return;
    }

    T.row(ind) = T.row(ind) / divisor;

    for (size_t i = 0; i < n; ++i) {
        if (i != ind) {
            float factor = T(i, 2 * n);
            T.row(i) = T.row(i) - factor * T.row(ind);
        }
    }
}
