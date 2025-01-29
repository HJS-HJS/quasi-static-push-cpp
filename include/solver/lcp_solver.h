#ifndef LCP_SOLVER_H
#define LCP_SOLVER_H

#include <vector>
#include <stdexcept>
#include <string>

class LCPSolver {
private:
    std::vector<std::vector<float>> T;
    size_t n;
    int maxIter;

    enum ColumnType { W = 0, Z = 1, Y = 2, Q = 3 };
    std::vector<int> wPos, zPos;

    bool initialize();
    bool step();
    void pivot(size_t pos);
    void clearDriverColumn(size_t ind);
    std::vector<float> extractSolution() const;

    void swapColumns(size_t i, size_t j);

public:
    LCPSolver(const std::vector<std::vector<float>>& M, const std::vector<float>& q, int maxIter = 100);
    std::pair<std::vector<float>, std::string> solve();
};

#endif // LCP_SOLVER_H