#include "solver/lcp_solver.h"
#include <iostream>
#include <chrono>
#include <vector>
#include <Eigen/Dense>

int main() {
    std::vector<Eigen::MatrixXf> testCasesM;
    // testCasesM.emplace_back((Eigen::MatrixXf(3, 3) << 
    //     1, 0, 0, 
    //     2, 3, 0, 
    //     4, 5, 6).finished());
    // testCasesM.emplace_back((Eigen::MatrixXf(2, 2) << 
    //     2, 1, 
    //     0, 2).finished());
    // testCasesM.emplace_back((Eigen::MatrixXf(8, 8) << 
    //        0.0263065,    0.0195291,  1.34687e-06, -1.34687e-06,   -0.0176257,    0.0176257,            0,            0,
    //        0.0195291,    0.0263065,    0.0176258,   -0.0176258, -1.51229e-06,  1.51229e-06,            0,            0,
    //      1.34687e-06,    0.0176258,     0.060809,    -0.060809,    0.0540128,   -0.0540128,            1,            0,
    //     -1.34687e-06,   -0.0176258,    -0.060809,     0.060809,   -0.0540128,    0.0540128,            0,            1,
    //       -0.0176257, -1.51229e-06,    0.0540128,   -0.0540128,     0.060809,    -0.060809,            1,            0,
    //        0.0176257,  1.51229e-06,   -0.0540128,    0.0540128,    -0.060809,     0.060809,            0,            1,
    //              0.2,            0,           -1,            0,           -1,            0,            0,            0,
    //                0,          0.2,            0,           -1,            0,           -1,            0,            0
    //                ).finished());
    testCasesM.emplace_back((Eigen::MatrixXf(12, 12) << 
           0.0263065,    0.0196206,    -0.001643,  1.43219e-06, -1.43219e-06,   -0.0175238,    0.0175238,    0.0252525,   -0.0252525,            0,            0,            0,
           0.0196206,    0.0263065,    -0.018046,    0.0175238,   -0.0175238, -1.47437e-06,  1.47437e-06,    0.0177407,   -0.0177407,            0,            0,            0,
           -0.001643,    -0.018046,    0.0477887,   -0.0252525,    0.0252525,   -0.0177407,    0.0177407,  0.000308009, -0.000308009,            0,            0,            0,
         1.43027e-06,    0.0175238,   -0.0252525,     0.060809,    -0.060809,    0.0541044,   -0.0541044,    0.0328503,   -0.0328503,            1,            0,            0,
        -1.43027e-06,   -0.0175238,    0.0252525,    -0.060809,     0.060809,   -0.0541044,    0.0541044,   -0.0328503,    0.0328503,            0,            1,            0,
          -0.0175238, -1.47623e-06,   -0.0177407,    0.0541044,   -0.0541044,     0.060809,    -0.060809,    0.0164474,   -0.0164474,            0,            0,            1,
           0.0175238,  1.47623e-06,    0.0177407,   -0.0541044,    0.0541044,    -0.060809,     0.060809,   -0.0164474,    0.0164474,            1,            0,            0,
           0.0252525,    0.0177407,  0.000308009,    0.0328503,   -0.0328503,    0.0164474,   -0.0164474,     0.101393,    -0.101393,            0,            1,            0,
          -0.0252525,   -0.0177407, -0.000308009,   -0.0328503,    0.0328503,   -0.0164474,    0.0164474,    -0.101393,     0.101393,            0,            0,            1,
                 0.2,            0,            0,           -1,            0,            0,           -1,            0,            0,            0,            0,            0,
                   0,          0.2,            0,            0,           -1,            0,            0,           -1,            0,            0,            0,            0,
                   0,            0,          0.2,            0,            0,           -1,            0,            0,           -1,            0,            0,            0
                   ).finished());

    std::vector<Eigen::VectorXf> testCasesQ;
    // testCasesQ.emplace_back((Eigen::VectorXf(3) << 9, 8, 7).finished());
    // testCasesQ.emplace_back((Eigen::VectorXf(2) << -1, -2).finished());
    // testCasesQ.emplace_back((Eigen::VectorXf(8) << 0.00242696, -0.000933826,  -0.00531309,   0.00531309,   0.00528202 , -0.00528202,            0,            0).finished());
    testCasesQ.emplace_back((Eigen::VectorXf(12) << 0.00440585, 0.00351519,0.000491571,-0.00529743, 0.00529743, 0.00528913,-0.00528913,          0,          0,          0,          0,          0).finished());


    for (size_t i = 0; i < testCasesM.size(); ++i) {
        auto start = std::chrono::high_resolution_clock::now();

        LCPSolver solver(testCasesM[i], testCasesQ[i]);
        auto [solution, status] = solver.solve();

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;

        std::cout << "Case #" << i + 1 << std::endl;
        std::cout << "\tTime spent: " << elapsed.count() << "s" << std::endl;
        std::cout << "\tStatus: " << status << std::endl;

        if (solution.size() > 0) {
            std::cout << "\tSolution: " << solution.transpose() << std::endl;
        } else {
            std::cout << "\tNo solution found." << std::endl;
        }
    }

    return 0;
}
