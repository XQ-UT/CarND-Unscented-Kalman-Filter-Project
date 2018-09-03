#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if(estimations.size() != ground_truth.size()){
      cout << "estimation size is not equal to groud_truth size." << endl;
      return rmse;
  }

  if(estimations.size() == 0){
      cout << "estimation size is 0." << endl;
      return rmse;
  }

  for(int i = 0; i < estimations.size(); ++i){
      VectorXd diff = (estimations[i] - ground_truth[i]).array().square();
      rmse += diff;
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;

}