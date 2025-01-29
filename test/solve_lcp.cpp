#include "solver/lcp_solver.h"
#include <iostream>
#include <chrono>

int main() {
    std::vector<std::vector<std::vector<float>>> testCasesM = {
        {{1, 0, 0}, {2, 3, 0}, {4, 5, 6}},
        {{2, 1}, {0, 2}}
    };

    std::vector<std::vector<float>> testCasesQ = {
        {9, 8, 7},
        {-1, -2}
    };

    for (size_t i = 0; i < testCasesM.size(); ++i) {
        auto start = std::chrono::high_resolution_clock::now();

        LCPSolver solver(testCasesM[i], testCasesQ[i]);
        auto [solution, status] = solver.solve();

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;

        std::cout << "Case #" << i + 1 << std::endl;
        std::cout << "\tTime spent: " << elapsed.count() << "s" << std::endl;
        std::cout << "\tStatus: " << status << std::endl;

        if (!solution.empty()) {
            std::cout << "\tSolution: [";
            for (size_t j = 0; j < solution.size(); ++j) {
                std::cout << solution[j];
                if (j < solution.size() - 1) std::cout << ", ";
            }
            std::cout << "]\n";
        } else {
            std::cout << "\tNo solution found." << std::endl;
        }
    }

    return 0;
}