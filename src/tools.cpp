#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd squared_errors_sum(4) ;
  squared_errors_sum.fill(0) ;

  for(int i = 0 ; i < estimations.size() ; ++i)
  {
    VectorXd difference = estimations[i] - ground_truth[i] ;
    VectorXd squared_difference = difference.array() * difference.array() ;

    squared_errors_sum += squared_difference ;
  }

  VectorXd MSE = squared_errors_sum / estimations.size() ;
  return MSE.array().sqrt() ;

}

double Tools::getNormalizedAngle(double angle)
{

  while (angle < -M_PI) { angle += 2.0 * M_PI ; }
  while (angle >  M_PI) { angle -= 2.0 * M_PI ; }

  return angle ;

}