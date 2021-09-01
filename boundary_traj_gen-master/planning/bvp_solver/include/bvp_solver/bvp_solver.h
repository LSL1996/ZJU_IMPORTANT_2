#ifndef _BVP_SOLVER_H_
#define _BVP_SOLVER_H_

#include "poly_traj_utils/traj_utils.hpp"
#include <Eigen/Eigen>

namespace BVPSolver
{
using Eigen::Vector3d;
using Eigen::VectorXd;

class IntegratorBVP {
public:
  IntegratorBVP()
  {
    x0_ = Eigen::Matrix<double, 9, 1>::Zero();
    x1_ = Eigen::Matrix<double, 9, 1>::Zero(); 
  };
  ~IntegratorBVP() {};
  void setRho(double rho) {rho_ = rho;};
  bool solve(const VectorXd& start, const VectorXd& goal)
  {
    setBoundaries(start, goal);
    return solveTriple();
  };
  double getTauStar() {return tau_star_;};
  double getCostStar() { return cost_star_;};
  void getCoeff(CoefficientMat &coeff) {coeff = coeff_;};
  void calCoeffFromTau(double tau, CoefficientMat &coeff);
  double calCostAccKnown(const VectorXd &x0, const VectorXd &x1, double T);

private:
  VectorXd x0_, x1_;
  double rho_;
  CoefficientMat coeff_;
  double tau_star_, cost_star_;

  bool calTauStarTriple();
  bool solveTriple();
  void setBoundaries(const VectorXd& start, const VectorXd& goal) 
  {
    x0_ = start;
    x1_ = goal;
  };

};

}
#endif