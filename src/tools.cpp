#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
    VectorXd rmse = VectorXd(4);
    rmse << 0, 0, 0, 0;

    // check that estimation and ground_truth are of the same size
    if (estimations.size() != ground_truth.size()) {
        return rmse;
    }

    for (unsigned int i = 0; i < estimations.size(); i++) {
        VectorXd error = estimations[i] - ground_truth[i];
        error = error.array() * error.array();
        rmse += error;
    }

    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
    MatrixXd Hj(3, 4);
    Hj << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float den = px*px + py*py;
    std::cout << den << std::endl;
    if (den < 1e-6) {
        return Hj;
    }

    float den05 = pow(den, 0.5);
    float den15 = pow(den, 1.5);
    float px_den05 = px/den05;
    float py_den05 = py/den05;

    Hj << px_den05, py_den05, 0, 0,
          -1*py/den, px/den, 0, 0,
          py*(vx*py - vy*px)/den15, px*(vy*px - vx*py)/den15,
          px_den05, py_den05;

    return Hj;
}
