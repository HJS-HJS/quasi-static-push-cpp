#include "solver/lcp_solver.h"
#include <iostream>
#include <algorithm>
#include <cmath>

LCPSolver::LCPSolver(const std::vector<std::vector<float>>& M, const std::vector<float>& q, int maxIter)
    : n(q.size()), maxIter(maxIter) {
    if (M.size() != n || M[0].size() != n) {
        throw std::invalid_argument("Matrix M must be square and match the size of vector q.");
    }

    T.resize(n, std::vector<float>(2 * n + 2, 0.0f));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            T[i][j + n] = -M[i][j];
        }
        T[i][2 * n] = -1.0f;
        T[i][2 * n + 1] = q[i];
    }

    wPos.resize(n);
    zPos.resize(n);
    for (size_t i = 0; i < n; ++i) {
        wPos[i] = i;
        zPos[i] = i + n;
    }
}

std::pair<std::vector<float>, std::string> LCPSolver::solve() {
    if (!initialize()) {
        return {extractSolution(), "Solution Found"};
    }

    for (int k = 0; k < maxIter; ++k) {
        if (!step()) {
            return {extractSolution(), "Secondary ray solution"};
        }
        if (T.back()[2 * n] == Y) {
            return {extractSolution(), "Solution Found"};
        }
    }
    return {{}, "Max Iterations Exceeded"};
}

bool LCPSolver::initialize() {
    size_t minIndex = std::min_element(T.begin(), T.end(), [](const std::vector<float>& a, const std::vector<float>& b) {
        return a.back() < b.back();
    }) - T.begin();

    if (T[minIndex].back() < 0) {
        clearDriverColumn(minIndex);
        pivot(minIndex);
        return true;
    }
    return false;
}

bool LCPSolver::step() {
    size_t pivotRow = std::numeric_limits<size_t>::max(); // Replace -1 with max value of size_t
    float minRatio = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < n; ++i) {
        if (T[i][2 * n] > 0) {
            float ratio = T[i][2 * n + 1] / T[i][2 * n];
            if (ratio < minRatio) {
                minRatio = ratio;
                pivotRow = i;
            }
        }
    }

    if (pivotRow != std::numeric_limits<size_t>::max()) { // Update condition
        clearDriverColumn(pivotRow);
        pivot(pivotRow);
        return true;
    }
    return false;
}

void LCPSolver::pivot(size_t pos) {
    size_t partnerPos = wPos[pos];
    if (partnerPos != std::numeric_limits<size_t>::max()) { // Update condition
        swapColumns(pos, partnerPos);
    }
}

void LCPSolver::clearDriverColumn(size_t ind) {
    float divisor = T[ind][2 * n] + 1e-6f;
    for (float& val : T[ind]) {
        val /= divisor;
    }

    for (size_t i = 0; i < n; ++i) {
        if (i != ind) {
            float factor = T[i][2 * n];
            for (size_t j = 0; j < T[i].size(); ++j) {
                T[i][j] -= factor * T[ind][j];
            }
        }
    }
}

std::vector<float> LCPSolver::extractSolution() const {
    std::vector<float> z(n, 0.0f);
    for (size_t i = 0; i < n; ++i) {
        if (wPos[i] >= n) {
            z[wPos[i] - n] = T[i].back();
        }
    }
    return z;
}

void LCPSolver::swapColumns(size_t i, size_t j) {
    for (auto& row : T) {
        std::swap(row[i], row[j]);
    }
    std::swap(wPos[i], zPos[j]);
}
