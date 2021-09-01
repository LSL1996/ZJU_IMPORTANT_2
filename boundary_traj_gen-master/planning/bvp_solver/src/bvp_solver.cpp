#include "bvp_solver/bvp_solver.h"
#include <iostream>

namespace BVPSolver
{

bool IntegratorBVP::solveTriple()
{
  bool result = calTauStarTriple();
  
  double t2 = tau_star_*tau_star_;
  double t3 = tau_star_*t2;
  double t4 = tau_star_*t3;
  double t5 = tau_star_*t4;
  coeff_(0, 0) = -(12*x0_[0] + 6*tau_star_*x0_[3] + t2*x0_[6] - 12*x1_[0] + 6*tau_star_*x1_[3] - t2*x1_[6])/(2*t5); 
  coeff_(1, 0) = -(12*x0_[1] + 6*tau_star_*x0_[4] + t2*x0_[7] - 12*x1_[1] + 6*tau_star_*x1_[4] - t2*x1_[7])/(2*t5); 
  coeff_(2, 0) = -(12*x0_[2] + 6*tau_star_*x0_[5] + t2*x0_[8] - 12*x1_[2] + 6*tau_star_*x1_[5] - t2*x1_[8])/(2*t5);
  coeff_(0, 1) = -(-30*x0_[0] - 16*tau_star_*x0_[3] - 3*t2*x0_[6] + 30*x1_[0] - 14*tau_star_*x1_[3] + 2*t2*x1_[6])/(2*t4);
  coeff_(1, 1) = -(-30*x0_[1] - 16*tau_star_*x0_[4] - 3*t2*x0_[7] + 30*x1_[1] - 14*tau_star_*x1_[4] + 2*t2*x1_[7])/(2*t4);
  coeff_(2, 1) = -(-30*x0_[2] - 16*tau_star_*x0_[5] - 3*t2*x0_[8] + 30*x1_[2] - 14*tau_star_*x1_[5] + 2*t2*x1_[8])/(2*t4); 
  coeff_(0, 2) = -(20*x0_[0] + 12*tau_star_*x0_[3] + 3*t2*x0_[6] - 20*x1_[0] + 8*tau_star_*x1_[3] - t2*x1_[6])/(2*t3); 
  coeff_(1, 2) = -(20*x0_[1] + 12*tau_star_*x0_[4] + 3*t2*x0_[7] - 20*x1_[1] + 8*tau_star_*x1_[4] - t2*x1_[7])/(2*t3); 
  coeff_(2, 2) = -(20*x0_[2] + 12*tau_star_*x0_[5] + 3*t2*x0_[8] - 20*x1_[2] + 8*tau_star_*x1_[5] - t2*x1_[8])/(2*t3); 
  coeff_(0, 3) = x0_[6]/2; 
  coeff_(1, 3) = x0_[7]/2; 
  coeff_(2, 3) = x0_[8]/2;
  coeff_(0, 4) = x0_[3]; 
  coeff_(1, 4) = x0_[4];
  coeff_(2, 4) = x0_[5]; 
  coeff_(0, 5) = x0_[0]; 
  coeff_(1, 5) = x0_[1]; 
  coeff_(2, 5) = x0_[2];

  return result;
}

/* minimize Inte(1 + jerk^T * rho * jerk, 0, tau) */
bool IntegratorBVP::calTauStarTriple()
{
  double t1 = 3*(x0_[6]*x0_[6] + x0_[7]*x0_[7] + x0_[8]*x0_[8] + x1_[6]*x1_[6] + x1_[7]*x1_[7] + x1_[8]*x1_[8]);
  double t2 = t1 - 2*(x0_[6]*x1_[6] + x0_[7]*x1_[7] + x0_[8]*x1_[8]);
  double t3 = 3*(x0_[3]*x0_[6] + x0_[4]*x0_[7] + x0_[5]*x0_[8] - x1_[3]*x1_[6] - x1_[4]*x1_[7] - x1_[5]*x1_[8]);
  double t4 = t3 + 2*(x0_[6]*x1_[3] + x0_[7]*x1_[4] + x0_[8]*x1_[5] - x0_[3]*x1_[6] - x0_[4]*x1_[7] - x0_[5]*x1_[8]);
  double t5 = 8*(x0_[3]*x0_[3] + x0_[4]*x0_[4] + x0_[5]*x0_[5] + x1_[3]*x1_[3] + x1_[4]*x1_[4] + x1_[5]*x1_[5]);
  double t6 = t5 + 5*((x0_[0]-x1_[0])*(x0_[6]-x1_[6]) + (x0_[1]-x1_[1])*(x0_[7]-x1_[7]) + (x0_[2]-x1_[2])*(x0_[8]-x1_[8]));
  double t7 = t6 + 14*(x0_[3]*x1_[3] + x0_[4]*x1_[4] + x0_[5]*x1_[5]);
  double t8 = (x0_[0]-x1_[0])*(x0_[3]+x1_[3]) + (x0_[1]-x1_[1])*(x0_[4]+x1_[4]) + (x0_[2]-x1_[2])*(x0_[5]+x1_[5]);
  double t9 = (x0_[0]-x1_[0])*(x0_[0]-x1_[0]) + (x0_[1]-x1_[1])*(x0_[1]-x1_[1]) + (x0_[2]-x1_[2])*(x0_[2]-x1_[2]);

  VectorXd p(7);
  p[0] = 1.0;
  p[1] = 0.0;
  p[2] = - 3*rho_*t2;
  p[3] = - 48*rho_*t4;
  p[4] = - 72*rho_*t7;
  p[5] = - 2800*rho_*t8;
  p[6] = - 3600*rho_*t9;
  std::set<double> roots = RootFinder::solvePolynomial(p, DBL_EPSILON, DBL_MAX, 1e-6);
  
  bool result = false;
  double tau = DBL_MAX;
  double cost = DBL_MAX;
  
  for (const double& root : roots) 
  {
    double root2 = root*root;
    double root3 = root2*root;
    double root4 = root3*root;
    double root5 = root4*root;
    double root6 = root5*root;
    
    double current = (root6 + rho_*(720*t9 + 720*root*t8 + 24*root2*t7 + 24*root3*t4 + 3*root4*t2)) / root5;

    if (current < cost) 
    {
      tau = root;
      cost = current;
      result = true;
    }
  }

  tau_star_ = tau;
  cost_star_ = cost;
  return result;
}

void IntegratorBVP::calCoeffFromTau(double tau, CoefficientMat &coeff)
{
  double t2 = tau*tau;
  double t3 = tau*t2;
  double t4 = tau*t3;
  double t5 = tau*t4;
  coeff(0, 0) = -(8*x0_[0] + 5*tau*x0_[3] + t2*x0_[6] - 8*x1_[0] + 3*tau*x1_[3])/(3*t5); 
  coeff(1, 0) = -(8*x0_[1] + 5*tau*x0_[4] + t2*x0_[7] - 8*x1_[1] + 3*tau*x1_[4])/(3*t5); 
  coeff(2, 0) = -(8*x0_[2] + 5*tau*x0_[5] + t2*x0_[8] - 8*x1_[2] + 3*tau*x1_[5])/(3*t5);
  coeff(0, 1) = -(-50*x0_[0] - 32*tau*x0_[3] - 7*t2*x0_[6] + 50*x1_[0] - 18*tau*x1_[3])/(6*t4);
  coeff(1, 1) = -(-50*x0_[1] - 32*tau*x0_[4] - 7*t2*x0_[7] + 50*x1_[1] - 18*tau*x1_[4])/(6*t4);
  coeff(2, 1) = -(-50*x0_[2] - 32*tau*x0_[5] - 7*t2*x0_[8] + 50*x1_[2] - 18*tau*x1_[5])/(6*t4); 
  coeff(0, 2) = -(20*x0_[0] + 14*tau*x0_[3] + 4*t2*x0_[6] - 20*x1_[0] + 6*tau*x1_[3])/(3*t3); 
  coeff(1, 2) = -(20*x0_[1] + 14*tau*x0_[4] + 4*t2*x0_[7] - 20*x1_[1] + 6*tau*x1_[4])/(3*t3); 
  coeff(2, 2) = -(20*x0_[2] + 14*tau*x0_[5] + 4*t2*x0_[8] - 20*x1_[2] + 6*tau*x1_[5])/(3*t3); 
  coeff(0, 3) = x0_[6]/2; 
  coeff(1, 3) = x0_[7]/2; 
  coeff(2, 3) = x0_[8]/2;
  coeff(0, 4) = x0_[3]; 
  coeff(1, 4) = x0_[4];
  coeff(2, 4) = x0_[5]; 
  coeff(0, 5) = x0_[0]; 
  coeff(1, 5) = x0_[1]; 
  coeff(2, 5) = x0_[2];
}

double IntegratorBVP::calCostAccKnown(const VectorXd &x0, const VectorXd &x1, double T)
{
  double t1 = 3*(x0[6]*x0[6] + x0[7]*x0[7] + x0[8]*x0[8] + x1[6]*x1[6] + x1[7]*x1[7] + x1[8]*x1[8]);
  double t2 = t1 - 2*(x0[6]*x1[6] + x0[7]*x1[7] + x0[8]*x1[8]);
  double t3 = 3*(x0[3]*x0[6] + x0[4]*x0[7] + x0[5]*x0[8] - x1[3]*x1[6] - x1[4]*x1[7] - x1[5]*x1[8]);
  double t4 = t3 + 2*(x0[6]*x1[3] + x0[7]*x1[4] + x0[8]*x1[5] - x0[3]*x1[6] - x0[4]*x1[7] - x0[5]*x1[8]);
  double t5 = 8*(x0[3]*x0[3] + x0[4]*x0[4] + x0[5]*x0[5] + x1[3]*x1[3] + x1[4]*x1[4] + x1[5]*x1[5]);
  double t6 = t5 + 5*((x0[0]-x1[0])*(x0[6]-x1[6]) + (x0[1]-x1[1])*(x0[7]-x1[7]) + (x0[2]-x1[2])*(x0[8]-x1[8]));
  double t7 = t6 + 14*(x0[3]*x1[3] + x0[4]*x1[4] + x0[5]*x1[5]);
  double t8 = (x0[0]-x1[0])*(x0[3]+x1[3]) + (x0[1]-x1[1])*(x0[4]+x1[4]) + (x0[2]-x1[2])*(x0[5]+x1[5]);
  double t9 = (x0[0]-x1[0])*(x0[0]-x1[0]) + (x0[1]-x1[1])*(x0[1]-x1[1]) + (x0[2]-x1[2])*(x0[2]-x1[2]);

  double T2 = T*T;
  double T3 = T2*T;
  double T4 = T3*T;
  double T5 = T4*T;
  double T6 = T5*T;

  double cost = (T6 + rho_*(720*t9 + 720*T*t8 + 24*T2*t7 + 24*T3*t4 + 3*T4*t2)) / T5;
}

}