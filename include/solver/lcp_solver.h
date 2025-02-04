#ifndef LCP_SOLVER_H
#define LCP_SOLVER_H

#include <vector>
#include <stdexcept>
#include <string>
#include <cmath>
#include <iostream>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>
#include <optional>

class LCPSolver {
private:
    Eigen::MatrixXf T;
    Eigen::MatrixXf Tind;
    size_t n;
    int maxIter;
    int W, Z, Y, Q;
    std::vector<int> wPos, zPos;

    bool initialize();
    bool step();
    std::optional<int> partnerPos(size_t pos);
    bool pivot(size_t pos);
    void clearDriverColumn(size_t ind);
    Eigen::VectorXf extractSolution() const;
    void swapPos(int v, int ind, int newPos);
    void swapColumns(size_t i, size_t j);

public:
    LCPSolver(const Eigen::MatrixXf& M, const Eigen::VectorXf& q, int maxIter = 100);
    std::pair<Eigen::VectorXf, std::string> solve();
};

#endif // LCP_SOLVER_H