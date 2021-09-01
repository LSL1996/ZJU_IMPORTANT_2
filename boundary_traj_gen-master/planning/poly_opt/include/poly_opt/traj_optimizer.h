#pragma once

#include "visualization_utils/visualization_utils.h"
#include "poly_opt/lbfgs.hpp"
#include "poly_traj_utils/traj_utils.hpp"

#include <Eigen/Eigen>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <iostream>

using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::vector;
#define AS_START 0
#define AS_END 1

inline double quasi_exp(const double &x)
{
  // return x > 0.0 ? ((0.5 * x + 1.0) * x + 1.0) : 1.0 / ((0.5 * x - 1.0) * x + 1.0);
  return 0.001 + x * x;
}

inline double quasi_log(const double &x)
{
  // return x > 1.0 ? (sqrt(2.0 * x - 1.0) - 1.0) : (1.0 - sqrt(2.0 / x - 1.0));
  return sqrt(std::max(x - 0.001, 0.0));
}

inline double d_quasi_exp(const double &x)
{
  // if (x > 0.0)
  // {
  //   return x + 1.0;
  // }
  // else
  // {
  //   double denSqrt = (0.5 * x - 1.0) * x + 1.0;
  //   return (1.0 - x) / (denSqrt * denSqrt);
  // }
  return 2.0 * x;
}

class TrajOpt
{
private:
  // n middle nodes, m segments, n+1 = m
  // decision variables, px1, py1, pz1, vx1, vy1, vz1, ax1, ay1, az1, ..., pxn, pyn, pzn, vxn, vyn, vzn, axn, ayn, azn, T1, T2, ..., Tm
  double *x_;
  double fx_;
  int N_;
  // parent state
  VectorXd head_state_, tail_state_;
  int seg_num_, mdl_wp_num_;

  double rho_;
  double vel_limit_, acc_limit_, jrk_limit_;
  double squared_vel_limit_, squared_acc_limit_, squared_jrk_limit_;
  int K_;
  double rho_vel_, rho_acc_, rho_jrk_;

  VisualRviz::Ptr vis_ptr_;

public:
  TrajOpt(const ros::NodeHandle &node) : x_(NULL)
  {
    node.param("poly_opt/rho", rho_, 0.0);
    node.param("poly_opt/vel_limit", vel_limit_, 0.0);
    node.param("poly_opt/acc_limit", acc_limit_, 0.0);
    node.param("poly_opt/jerk_limit", jrk_limit_, 0.0);
    node.param("poly_opt/discretization", K_, 0);
    node.param("poly_opt/rho_vel", rho_vel_, 0.0);
    node.param("poly_opt/rho_acc", rho_acc_, 0.0);
    node.param("poly_opt/rho_jrk", rho_jrk_, 0.0);
    squared_vel_limit_ = vel_limit_ * vel_limit_;
    squared_acc_limit_ = acc_limit_ * acc_limit_;
    squared_jrk_limit_ = jrk_limit_ * jrk_limit_;
    ROS_WARN_STREAM("[poly_opt] param: rho: " << rho_);
    ROS_WARN_STREAM("[poly_opt] param: vel_limit: " << vel_limit_);
    ROS_WARN_STREAM("[poly_opt] param: acc_limit: " << acc_limit_);
    ROS_WARN_STREAM("[poly_opt] param: jerk_limit: " << jrk_limit_);
    ROS_WARN_STREAM("[poly_opt] param: discretization: " << K_);
    ROS_WARN_STREAM("[poly_opt] param: rho_vel: " << rho_vel_);
    ROS_WARN_STREAM("[poly_opt] param: rho_acc: " << rho_acc_);
    ROS_WARN_STREAM("[poly_opt] param: rho_jerk: " << rho_jrk_);
  }

  ~TrajOpt() {}

  void setVisualizer(const VisualRviz::Ptr &vis)
  {
    vis_ptr_ = vis;
  }

  bool optimizeTraj(vector<VectorXd> &middle_nodes_state,
                    vector<double> &seg_durs,
                    const VectorXd &head_state,
                    const VectorXd &tail_state,
                    double &cost)
  {
    bool res(false);
    seg_num_ = seg_durs.size();
    mdl_wp_num_ = seg_num_ - 1;
    N_ = mdl_wp_num_ * 9 + seg_num_;

    head_state_ = head_state;
    tail_state_ = tail_state;

    /* Initialize the variables. */
    x_ = new double[N_];
    for (int i = 0; i < mdl_wp_num_; i++)
    {
      for (int j = 0; j < 9; ++j)
        x_[i * 9 + j] = middle_nodes_state[i][j];
    }
    for (int i = 0; i < seg_num_; i++)
    {
      x_[i + mdl_wp_num_ * 9] = quasi_log(seg_durs[i]);
    }

    int opt_ret = run(N_);
    std::cout << "[opt res] " << opt_ret << std::endl;
    if (opt_ret < 0)
    {
      res = false;
    }
    else
    {
      for (int i = 0; i < mdl_wp_num_; i++)
      {
        for (int j = 0; j < 9; ++j)
          middle_nodes_state[i][j] = x_[i * 9 + j];
      }
      for (int i = 0; i < seg_num_; i++)
      {
        // seg_durs[i] = quasi_exp(x_[i + mdl_wp_num_ * 9]);  // different segment duration
        seg_durs[i] = quasi_exp(x_[mdl_wp_num_ * 9]); // equal segment duration
      }
      cost = fx_;
      res = true;
    }

    delete[] x_;
    return res;
  }

private:
  int run(int N)
  {
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.past = 3;
    lbfgs_params.delta = 1e-5;
    // lbfgs_params.min_step = 1e-32;
    // int ret = lbfgs::lbfgs_optimize(N, x_, &fx_, _evaluate, NULL, _progress, this, &lbfgs_params); // show progress
    int ret = lbfgs::lbfgs_optimize(N, x_, &fx_, _evaluate, NULL, NULL, this, &lbfgs_params); // don't show progress

    return ret;
  }

  static double _evaluate(void *instance,
                          const double *x,
                          double *g,
                          const int n)
  {
    return reinterpret_cast<TrajOpt *>(instance)->evaluate(x, g, n);
  }

  double evaluate(const double *x,
                  double *g,
                  const int n)
  {
    double obj(0.0);
    /* vector of ndoes state: [px py pz vx vy vz ax ay az]' */
    vector<VectorXd> D(mdl_wp_num_), g_D(mdl_wp_num_);
    for (int i = 0; i < mdl_wp_num_; ++i)
    {
      D[i] = Eigen::Map<const VectorXd>(x + i * 9, 9);
      g_D[i] = Eigen::Map<const VectorXd>(g + i * 9, 9);
      g_D[i].setZero();
    }

    /* time durations: [T1 T2 ... Tm]' */
    Eigen::Map<const VectorXd> Tau(x + mdl_wp_num_ * 9, seg_num_);
    Eigen::Map<VectorXd> g_Tau(g + mdl_wp_num_ * 9, seg_num_);
    g_Tau.setZero();

    /* different segment duration */ 
    // /* segment: head node to the first middle node */
    // obj += calSmoothCostAddGrad(head_state_, D[0], Tau[0], g_D[0], g_Tau[0], AS_END);
    // obj += calTimeItgPntAddGrad(head_state_, D[0], Tau[0], g_D[0], g_Tau[0], AS_END);

    // /* segment: middle nodes to successive middle node */
    // for (int idx_mdl_node = 0; idx_mdl_node < mdl_wp_num_ - 1; ++idx_mdl_node)
    // {
    //   int idx_seg = idx_mdl_node + 1;
    //   obj += calSmoothCostAddGrad(D[idx_mdl_node], D[idx_mdl_node + 1], Tau[idx_seg], g_D[idx_mdl_node], g_Tau[idx_seg], AS_START);
    //   obj += calTimeItgPntAddGrad(D[idx_mdl_node], D[idx_mdl_node + 1], Tau[idx_seg], g_D[idx_mdl_node], g_Tau[idx_seg], AS_START);
    //   /* for middle segments, there is NO NEED to add gradient of t and cost twice */
    //   double tmp;
    //   calSmoothCostAddGrad(D[idx_mdl_node], D[idx_mdl_node + 1], Tau[idx_seg], g_D[idx_mdl_node + 1], tmp, AS_END);
    //   calTimeItgPntAddGrad(D[idx_mdl_node], D[idx_mdl_node + 1], Tau[idx_seg], g_D[idx_mdl_node + 1], tmp, AS_END);
    // }

    // /* segment: the last middle node to tail node */
    // obj += calSmoothCostAddGrad(D[mdl_wp_num_ - 1], tail_state_, Tau[seg_num_ - 1], g_D[mdl_wp_num_ - 1], g_Tau[seg_num_ - 1], AS_START);
    // obj += calTimeItgPntAddGrad(D[mdl_wp_num_ - 1], tail_state_, Tau[seg_num_ - 1], g_D[mdl_wp_num_ - 1], g_Tau[seg_num_ - 1], AS_START);
    /* different segment duration */ 


    /* equal segment duration */ 
    /* segment: head node to the first middle node */
    obj += calSmoothCostAddGrad(head_state_, D[0], Tau[0], g_D[0], g_Tau[0], AS_END);
    obj += calTimeItgPntAddGrad(head_state_, D[0], Tau[0], g_D[0], g_Tau[0], AS_END);

    /* segment: middle nodes to successive middle node */
    for (int idx_mdl_node = 0; idx_mdl_node < mdl_wp_num_ - 1; ++idx_mdl_node)
    {
      int idx_seg = idx_mdl_node + 1;
      obj += calSmoothCostAddGrad(D[idx_mdl_node], D[idx_mdl_node + 1], Tau[0], g_D[idx_mdl_node], g_Tau[0], AS_START);
      obj += calTimeItgPntAddGrad(D[idx_mdl_node], D[idx_mdl_node + 1], Tau[0], g_D[idx_mdl_node], g_Tau[0], AS_START);
      /* for middle segments, there is NO NEED to add gradient of t and cost twice */
      double tmp;
      calSmoothCostAddGrad(D[idx_mdl_node], D[idx_mdl_node + 1], Tau[0], g_D[idx_mdl_node + 1], tmp, AS_END);
      calTimeItgPntAddGrad(D[idx_mdl_node], D[idx_mdl_node + 1], Tau[0], g_D[idx_mdl_node + 1], tmp, AS_END);
    }

    /* segment: the last middle node to tail node */
    obj += calSmoothCostAddGrad(D[mdl_wp_num_ - 1], tail_state_, Tau[0], g_D[mdl_wp_num_ - 1], g_Tau[0], AS_START);
    obj += calTimeItgPntAddGrad(D[mdl_wp_num_ - 1], tail_state_, Tau[0], g_D[mdl_wp_num_ - 1], g_Tau[0], AS_START);
    /* equal segment duration */ 

    for (int i = 0; i < mdl_wp_num_; ++i)
    {
      for (int j = 0; j < 9; ++j)
        g[i * 9 + j] = g_D[i][j];
    }

    return obj;
  }

  double calSmoothCostAddGrad(const VectorXd &x0, const VectorXd &x1, double Tau, VectorXd &grad_d, double &grad_Tau, int type)
  {
    double t1 = 3 * (x0[6] * x0[6] + x0[7] * x0[7] + x0[8] * x0[8] + x1[6] * x1[6] + x1[7] * x1[7] + x1[8] * x1[8]);
    double t2 = t1 - 2 * (x0[6] * x1[6] + x0[7] * x1[7] + x0[8] * x1[8]);
    double t3 = 3 * (x0[3] * x0[6] + x0[4] * x0[7] + x0[5] * x0[8] - x1[3] * x1[6] - x1[4] * x1[7] - x1[5] * x1[8]);
    double t4 = t3 + 2 * (x0[6] * x1[3] + x0[7] * x1[4] + x0[8] * x1[5] - x0[3] * x1[6] - x0[4] * x1[7] - x0[5] * x1[8]);
    double t5 = 8 * (x0[3] * x0[3] + x0[4] * x0[4] + x0[5] * x0[5] + x1[3] * x1[3] + x1[4] * x1[4] + x1[5] * x1[5]);
    double t6 = t5 + 5 * ((x0[0] - x1[0]) * (x0[6] - x1[6]) + (x0[1] - x1[1]) * (x0[7] - x1[7]) + (x0[2] - x1[2]) * (x0[8] - x1[8]));
    double t7 = t6 + 14 * (x0[3] * x1[3] + x0[4] * x1[4] + x0[5] * x1[5]);
    double t8 = (x0[0] - x1[0]) * (x0[3] + x1[3]) + (x0[1] - x1[1]) * (x0[4] + x1[4]) + (x0[2] - x1[2]) * (x0[5] + x1[5]);
    double t9 = (x0[0] - x1[0]) * (x0[0] - x1[0]) + (x0[1] - x1[1]) * (x0[1] - x1[1]) + (x0[2] - x1[2]) * (x0[2] - x1[2]);

    double T(quasi_exp(Tau));
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    double T6 = T5 * T;

    double cost = (T6 + rho_ * (720 * t9 + 720 * T * t8 + 24 * T2 * t7 + 24 * T3 * t4 + 3 * T4 * t2)) / T5;
    grad_Tau += (T6 - rho_ * (3600 * t9 + 2880 * T * t8 + 72 * T2 * t7 + 48 * T3 * t4 + 3 * T4 * t2)) / T6 * d_quasi_exp(Tau);
    // ROS_ERROR_STREAM("smt T grad: " << grad_T);

    if (type == AS_START)
    {
      grad_d[0] += 120 * rho_ * (12 * (x0[0] - x1[0]) + 6 * T * (x0[3] + x1[3]) + T2 * (x0[6] - x1[6])) / T5;
      grad_d[1] += 120 * rho_ * (12 * (x0[1] - x1[1]) + 6 * T * (x0[4] + x1[4]) + T2 * (x0[7] - x1[7])) / T5;
      grad_d[2] += 120 * rho_ * (12 * (x0[2] - x1[2]) + 6 * T * (x0[5] + x1[5]) + T2 * (x0[8] - x1[8])) / T5;
      grad_d[3] += 24 * rho_ * (30 * (x0[0] - x1[0]) + T * (16 * x0[3] + 14 * x1[3]) + T2 * (3 * x0[6] - 2 * x1[6])) / T4;
      grad_d[4] += 24 * rho_ * (30 * (x0[1] - x1[1]) + T * (16 * x0[4] + 14 * x1[4]) + T2 * (3 * x0[7] - 2 * x1[7])) / T4;
      grad_d[5] += 24 * rho_ * (30 * (x0[2] - x1[2]) + T * (16 * x0[5] + 14 * x1[5]) + T2 * (3 * x0[8] - 2 * x1[8])) / T4;
      grad_d[6] += 6 * rho_ * (20 * (x0[0] - x1[0]) + T * (12 * x0[3] + 8 * x1[3]) + T2 * (3 * x0[6] - x1[6])) / T3;
      grad_d[7] += 6 * rho_ * (20 * (x0[1] - x1[1]) + T * (12 * x0[4] + 8 * x1[4]) + T2 * (3 * x0[7] - x1[7])) / T3;
      grad_d[8] += 6 * rho_ * (20 * (x0[2] - x1[2]) + T * (12 * x0[5] + 8 * x1[5]) + T2 * (3 * x0[8] - x1[8])) / T3;
    }
    else if (type == AS_END)
    {
      grad_d[0] += -120 * rho_ * (12 * (x0[0] - x1[0]) + 6 * T * (x0[3] + x1[3]) + T2 * (x0[6] - x1[6])) / T5;
      grad_d[1] += -120 * rho_ * (12 * (x0[1] - x1[1]) + 6 * T * (x0[4] + x1[4]) + T2 * (x0[7] - x1[7])) / T5;
      grad_d[2] += -120 * rho_ * (12 * (x0[2] - x1[2]) + 6 * T * (x0[5] + x1[5]) + T2 * (x0[8] - x1[8])) / T5;
      grad_d[3] += 24 * rho_ * (30 * (x0[0] - x1[0]) + T * (14 * x0[3] + 16 * x1[3]) + T2 * (2 * x0[6] - 3 * x1[6])) / T4;
      grad_d[4] += 24 * rho_ * (30 * (x0[1] - x1[1]) + T * (14 * x0[4] + 16 * x1[4]) + T2 * (2 * x0[7] - 3 * x1[7])) / T4;
      grad_d[5] += 24 * rho_ * (30 * (x0[2] - x1[2]) + T * (14 * x0[5] + 16 * x1[5]) + T2 * (2 * x0[8] - 3 * x1[8])) / T4;
      grad_d[6] += -6 * rho_ * (20 * (x0[0] - x1[0]) + T * (8 * x0[3] + 12 * x1[3]) + T2 * (x0[6] - 3 * x1[6])) / T3;
      grad_d[7] += -6 * rho_ * (20 * (x0[1] - x1[1]) + T * (8 * x0[4] + 12 * x1[4]) + T2 * (x0[7] - 3 * x1[7])) / T3;
      grad_d[8] += -6 * rho_ * (20 * (x0[2] - x1[2]) + T * (8 * x0[5] + 12 * x1[5]) + T2 * (x0[8] - 3 * x1[8])) / T3;
    }

    return cost;
  }

  double calTimeItgPntAddGrad(const VectorXd &x0, const VectorXd &x1, double Tau, VectorXd &grad_d, double &grad_Tau, int type)
  {
    CoefficientMat coeff; /* [cx5,cx4,cx3,cx2,cx1,cx0; 
                              cy5,cy4,cy3,cy2,cy1,cy0; 
                              cz5,cz4,cz3,cz2,cz1,cz0] */
    double T(quasi_exp(Tau));
    double T2(T * T), T3(T2 * T), T4(T3 * T), T5(T4 * T);
    coeff.col(0) = (0.5 * (x1.segment(6, 3) - x0.segment(6, 3)) * T2 -
                    3.0 * (x0.segment(3, 3) + x1.segment(3, 3)) * T +
                    6.0 * (x1.segment(0, 3) - x0.segment(0, 3))) /
                   T5;
    coeff.col(1) = ((-x1.segment(6, 3) + 1.5 * x0.segment(6, 3)) * T2 +
                    (8.0 * x0.segment(3, 3) + 7.0 * x1.segment(3, 3)) * T +
                    15.0 * (-x1.segment(0, 3) + x0.segment(0, 3))) /
                   T4;
    coeff.col(2) = ((0.5 * x1.segment(6, 3) - 1.5 * x0.segment(6, 3)) * T2 -
                    (6.0 * x0.segment(3, 3) + 4.0 * x1.segment(3, 3)) * T +
                    10.0 * (x1.segment(0, 3) - x0.segment(0, 3))) /
                   T3;
    coeff.col(3) = 0.5 * x0.segment(6, 3);
    coeff.col(4) = x0.segment(3, 3);
    coeff.col(5) = x0.segment(0, 3);

    Matrix<double, 3, 3> grad_Jv_to_d = Matrix<double, 3, 3>::Zero(3, 3);
    Matrix<double, 3, 3> grad_Ja_to_d = Matrix<double, 3, 3>::Zero(3, 3);
    Matrix<double, 3, 3> grad_Jj_to_d = Matrix<double, 3, 3>::Zero(3, 3);
    Vector3d grad_Jv_to_v, grad_Ja_to_a, grad_Jj_to_j;
    Vector3d grad_v_to_t, grad_a_to_t, grad_j_to_t;
    Matrix<double, 3, 6> grad_Jv_to_c, grad_Ja_to_c, grad_Jj_to_c;
    Matrix<double, 6, 3> grad_c_to_d = Matrix<double, 6, 3>::Zero(6, 3);
    Matrix<double, 3, 6> grad_c_to_t = Matrix<double, 3, 6>::Zero(3, 6);

    double grad_Jv_to_t(0.0), grad_Ja_to_t(0.0), grad_Jj_to_t(0.0);
    if (type == AS_START)
    {
      grad_c_to_d << -6. / T5, -3. / T4, -0.5 / T3,
          15. / T4, 8. / T3, 1.5 / T2,
          -10 / T3, -6. / T2, -1.5 / T,
          0, 0, 0.5,
          0, 1, 0,
          1, 0, 0;
    }
    else if (type == AS_END)
    {
      grad_c_to_d << 6. / T5, -3. / T4, 0.5 / T3,
          -15. / T4, 7. / T3, -1 / T2,
          10. / T3, -4. / T2, 0.5 / T,
          0, 0, 0,
          0, 0, 0,
          0, 0, 0;
    }
    for (int i = 0; i < 3; ++i)
    {
      double tmp = x0[i] - x1[i];
      grad_c_to_t(i, 0) = (30 * tmp + T * (1.5 * x0[6 + i] * T - 1.5 * x1[6 + i] * T + 12 * x0[3 + i] + 12 * x1[3 + i])) / T5 / T;
      grad_c_to_t(i, 1) = (-60 * tmp + T * (-3 * x0[6 + i] * T + 2 * x1[6 + i] * T - 24 * x0[3 + i] - 21 * x1[3 + i])) / T5;
      grad_c_to_t(i, 2) = (30 * tmp + T * (1.5 * x0[6 + i] * T - 0.5 * x1[6 + i] * T + 12 * x0[3 + i] + 8 * x1[3 + i])) / T4;
    }
    double step, omega;
    Vector3d pos, vel, acc, jrk;
    Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    double t1(0.0), t2, t3, t4, t5;
    step = T / K_;
    double cost_vel(0.0), cost_acc(0.0), cost_jrk(0.0);
    double cost_tmp(0.0);
    for (int j = 0; j <= K_; ++j, t1 += step)
    {
      t2 = t1 * t1;
      t3 = t2 * t1;
      t4 = t2 * t2;
      t5 = t4 * t1;
      beta0 << t5, t4, t3, t2, t1, 1.0;
      beta1 << 5.0 * t4, 4.0 * t3, 3.0 * t2, 2.0 * t1, 1.0, 0.0;
      beta2 << 20.0 * t3, 12.0 * t2, 6.0 * t1, 2.0, 0.0, 0.0;
      beta3 << 60.0 * t2, 24.0 * t1, 6.0, 0.0, 0.0, 0.0;
      beta4 << 120.0 * t1, 24.0, 0.0, 0.0, 0.0, 0.0;

      pos = coeff * beta0;
      vel = coeff * beta1;
      acc = coeff * beta2;
      jrk = coeff * beta3;

      omega = (j == 0 || j == K_) ? 0.5 : 1.0;
      if (highDerivativeCostGrad(vel, squared_vel_limit_, grad_Jv_to_v, cost_tmp))
      {
        // ROS_INFO_STREAM("AS_END: " << type << ", j: " << j << ", T: " << T << ", vel: " << vel.norm() << ", grad: " << grad_Jv_to_v.transpose());
        grad_Jv_to_c = grad_Jv_to_v * beta1.transpose();
        grad_Jv_to_d += omega * grad_Jv_to_c * grad_c_to_d;
        grad_v_to_t = grad_c_to_t * beta1 + coeff * beta2 * j / K_;
        grad_Jv_to_t += omega * (cost_tmp / K_ + step * grad_Jv_to_v.transpose() * grad_v_to_t);
        cost_vel += omega * cost_tmp;
      }
      if (highDerivativeCostGrad(acc, squared_acc_limit_, grad_Ja_to_a, cost_tmp))
      {
        // ROS_INFO_STREAM("AS_END: " << type << ", j: " << j << ", T: " << T << ", acc: " << acc.norm() << ", grad: " << grad_Ja_to_a.transpose());
        grad_Ja_to_c = grad_Ja_to_a * beta2.transpose();
        grad_Ja_to_d += omega * grad_Ja_to_c * grad_c_to_d;
        grad_a_to_t = grad_c_to_t * beta2 + coeff * beta3 * j / K_;
        grad_Ja_to_t += omega * (cost_tmp / K_ + step * grad_Ja_to_a.transpose() * grad_a_to_t);
        cost_acc += omega * cost_tmp;
      }
      if (highDerivativeCostGrad(jrk, squared_jrk_limit_, grad_Jj_to_j, cost_tmp))
      {
        grad_Jj_to_c = grad_Jj_to_j * beta3.transpose();
        grad_Jj_to_d += omega * grad_Jj_to_c * grad_c_to_d;
        grad_j_to_t = grad_c_to_t * beta3 + coeff * beta4 * j / K_;
        grad_Jj_to_t += omega * (cost_tmp / K_ + step * grad_Jj_to_j.transpose() * grad_j_to_t);
        cost_jrk += omega * cost_tmp;
      }
    }

    Matrix<double, 3, 3> grad_J_to_d = grad_Jv_to_d * rho_vel_ + grad_Ja_to_d * rho_acc_ + grad_Jj_to_d * rho_jrk_;
    grad_J_to_d *= step;
    grad_d.head(3) += grad_J_to_d.col(0);
    grad_d.segment(3, 3) += grad_J_to_d.col(1);
    grad_d.tail(3) += grad_J_to_d.col(2);

    double tmp_grad_T = grad_Jv_to_t * rho_vel_ + grad_Ja_to_t * rho_acc_ + grad_Jj_to_t * rho_jrk_;
    grad_Tau += tmp_grad_T * d_quasi_exp(Tau);
    // ROS_ERROR_STREAM("itg T grad: " << grad_T);

    double cost(cost_vel * rho_vel_ + cost_acc * rho_acc_ + cost_jrk * rho_jrk_);
    cost *= step;
    return cost;
  }

  /*
  * For penalty of higher derivatives like vel\acc\jerk\...
  * f = max((v^2 - v_max^2)^3, 0)
  */
  bool highDerivativeCostGrad(const Vector3d &derivative,
                              const double &squared_limit,
                              Vector3d &grad,
                              double &cost)
  {
    double squared_diff = derivative.squaredNorm() - squared_limit;
    if (squared_diff > 0)
    {
      grad = 6 * squared_diff * squared_diff * derivative;
      cost = squared_diff * squared_diff * squared_diff;
      return true;
    }
    return false;
  }

  static int _progress(void *instance,
                       const double *x,
                       const double *g,
                       const double fx,
                       const double xnorm,
                       const double gnorm,
                       const double step,
                       int n,
                       int k,
                       int ls)
  {
    return reinterpret_cast<TrajOpt *>(instance)->progress(x, g, fx, xnorm, gnorm, step, n, k, ls);
  }

  int progress(const double *x,
               const double *g,
               const double fx,
               const double xnorm,
               const double gnorm,
               const double step,
               int n,
               int k,
               int ls)
  {
    vector<StatePVA> middle_nodes_state(mdl_wp_num_);
    vector<double> seg_durs(seg_num_);
    for (int i = 0; i < mdl_wp_num_; i++)
    {
      for (int j = 0; j < 9; ++j)
        middle_nodes_state[i][j] = x[i * 9 + j];

      std::cout << "middle_nodes_state: " << middle_nodes_state[i].transpose() << std::endl;
    }
    for (int i = 0; i < seg_num_; i++)
    {
      // seg_durs[i] = quasi_exp(x[i + mdl_wp_num_ * 9]); // different segment duration
      seg_durs[i] = quasi_exp(x[mdl_wp_num_ * 9]);  // equal segment duration
      std::cout << "seg_durs: " << seg_durs[i] << std::endl;
    }
    vector<CoefficientMat> coeffs;
    CoefficientMat coeff; /* [cx5,cx4,cx3,cx2,cx1,cx0; 
                    cy5,cy4,cy3,cy2,cy1,cy0; 
                    cz5,cz4,cz3,cz2,cz1,cz0] */
    double T;
    VectorXd x0(9), x1(9);
    for (int i = 0; i < seg_num_; ++i)
    {
      T = seg_durs[i];
      /*boundary_condition to coeff */
      double T2(T * T), T3(T2 * T), T4(T3 * T), T5(T4 * T);
      if (i == 0)
      {
        x0 = head_state_;
        x1 = middle_nodes_state[i];
      }
      else if (i == seg_num_ - 1)
      {
        x0 = middle_nodes_state[i - 1];
        x1 = tail_state_;
      }
      else
      {
        x0 = middle_nodes_state[i - 1];
        x1 = middle_nodes_state[i];
      }
      coeff.col(0) = (0.5 * (x1.segment(6, 3) - x0.segment(6, 3)) * T2 -
                      3.0 * (x0.segment(3, 3) + x1.segment(3, 3)) * T +
                      6.0 * (x1.segment(0, 3) - x0.segment(0, 3))) /
                     T5;
      coeff.col(1) = ((-x1.segment(6, 3) + 1.5 * x0.segment(6, 3)) * T2 +
                      (8.0 * x0.segment(3, 3) + 7.0 * x1.segment(3, 3)) * T +
                      15.0 * (-x1.segment(0, 3) + x0.segment(0, 3))) /
                     T4;
      coeff.col(2) = ((0.5 * x1.segment(6, 3) - 1.5 * x0.segment(6, 3)) * T2 -
                      (6.0 * x0.segment(3, 3) + 4.0 * x1.segment(3, 3)) * T +
                      10.0 * (x1.segment(0, 3) - x0.segment(0, 3))) /
                     T3;
      coeff.col(3) = 0.5 * x0.segment(6, 3);
      coeff.col(4) = x0.segment(3, 3);
      coeff.col(5) = x0.segment(0, 3);
      coeffs.emplace_back(coeff);
    }
    Trajectory tmp_traj(seg_durs, coeffs);
    /* visualize */
    vector<StatePVA> vis_x;
    vector<Vector3d> knots;
    for (int i = 0; i < tmp_traj.getPieceNum(); ++i)
    {
      Vector3d p = tmp_traj[i].getPos(tmp_traj[i].getDuration());
      knots.push_back(p);
    }
    vis_x.clear();
    tmp_traj.sampleWholeTrajectory(&vis_x);
    vis_ptr_->visualizeStates(vis_x, FMTTraj, ros::Time::now());
    vis_ptr_->visualizeKnots(knots, ros::Time::now());
    printf("Iteration %d:\n", k);
    printf("  fx = %f, step = %f, xnorm = %f, gnorm = %f\n", fx, step, xnorm, gnorm);
    // getchar();

    return 0;
  }
};
