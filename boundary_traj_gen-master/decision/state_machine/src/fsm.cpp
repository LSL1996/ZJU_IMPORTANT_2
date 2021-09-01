#include "state_machine/fsm.h"
#include <ros/console.h>

void FSM::init(const ros::NodeHandle &nh)
{
  nh.param("fsm/rho", rho_, 0.0);
  nh.param("fsm/seg_num", seg_num_, 0);
  nh.param("fsm/vel_limit", vel_limit_, 0.0);
  nh.param("fsm/acc_limit", acc_limit_, 0.0);
  nh.param("fsm/jerk_limit", jrk_limit_, 0.0);
  ROS_WARN_STREAM("rho_: " << rho_);
  ROS_WARN_STREAM("seg_num_: " << seg_num_);

  bvp_.setRho(rho_);
  vis_ptr_.reset(new VisualRviz(nh));
  traj_optimizer_ptr_.reset(new TrajOpt(nh));
  traj_optimizer_ptr_->setVisualizer(vis_ptr_);

  goal_sub_ = nh_.subscribe("/goal", 1, &FSM::goalCallback, this);
  execution_timer_ = nh_.createTimer(ros::Duration(0.01), &FSM::executionCallback, this); // 100Hz

  new_goal_ = false;
  started_ = false;
  machine_state_ = WAIT_GOAL;

  start_state_ = Eigen::Matrix<double, 9, 1>::Zero();
  end_state_ = Eigen::Matrix<double, 9, 1>::Zero();
}

inline void FSM::goalCallback(const nav_msgs::Odometry::ConstPtr &goal_msg)
{
  end_state_ << goal_msg->pose.pose.position.x, 
                goal_msg->pose.pose.position.y, 
                goal_msg->pose.pose.position.z,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  new_goal_ = true;
}

void FSM::executionCallback(const ros::TimerEvent &event)
{
  static ros::Time start_follow_time, last_replan_start_time;
  static int replan_state = 1;
  static StatePVA last_replan_start_state;

  switch (machine_state_)
  {
  case WAIT_GOAL:
  {
    if (!new_goal_)
    {
      return;
    }
    else
    {
      new_goal_ = false;
      getPlanStartState(start_state_);
      changeState(GENERATE_TRAJ);
    }
    break;
  }

  case GENERATE_TRAJ:
  {
    ros::Time t_s(ros::Time::now());
    bool res = planOnce(start_state_, end_state_);
    if (res)
    {
      started_ = true;
      ros::Time t_e(ros::Time::now());
      ROS_WARN_STREAM("opt succeed, used: " << (t_e - t_s).toSec() * 1000 << " ms");
      /* visualize */
      vector<StatePVA> vis_x;
      vector<Vector3d> knots;
      knots.push_back(traj_.getPos(0.0));
      for (int i=0; i<traj_.getPieceNum(); ++i) {
        Vector3d p = traj_[i].getPos(traj_[i].getDuration());
        knots.push_back(p);
      }
      vis_x.clear();
      traj_.sampleWholeTrajectory(&vis_x);
      vis_ptr_->visualizeStates(vis_x, FinalTraj, ros::Time::now());
      vis_ptr_->visualizeKnots(knots, ros::Time::now());

      curr_traj_start_time_ = ros::Time::now();
      new_goal_ = false;
      changeState(FOLLOW_TRAJ);
    }
    else
    {
      ros::Time t_e(ros::Time::now());
      ROS_ERROR_STREAM("opt failed, used: " << (t_e - t_s).toSec() * 1000 << " ms");
      new_goal_ = false;
      changeState(WAIT_GOAL);
    }
    break;
  }

  case FOLLOW_TRAJ:
  {
    double t_during_traj = (ros::Time::now() - curr_traj_start_time_).toSec();
    VectorXd curr_expected_state(9);
    curr_expected_state.head(3) = traj_.getPos(t_during_traj);
    curr_expected_state.segment(3, 3) = traj_.getVel(t_during_traj);
    curr_expected_state.tail(3) = traj_.getAcc(t_during_traj);
    vis_ptr_->visualizeCurrExpectedState(curr_expected_state, ros::Time::now());
    if (t_during_traj >= traj_.getTotalDuration() - 0.02)
    {
      changeState(WAIT_GOAL);
    }
    else if (new_goal_)
    {
      new_goal_ = false;
      getPlanStartState(start_state_);
      changeState(GENERATE_TRAJ);
    }
    break;
  }

  default:
    break;
  }
}

bool FSM::planOnce(const VectorXd& head, const VectorXd& tail)
{
  if (bvp_.solve(head, tail))
  {
    /* use bvp to get initial traj */
    double initial_total_T = bvp_.getTauStar();
    CoefficientMat initial_coeff;
    bvp_.getCoeff(initial_coeff);
    Piece initial_traj(initial_total_T, initial_coeff);
    ROS_INFO_STREAM("initial T: " << initial_total_T 
                 << "\tcost: " << bvp_.getCostStar()
                 << "\tmax vel: " << initial_traj.getMaxVelRate() 
                 << "\tmax acc: " << initial_traj.getMaxAccRate() 
                 << "\tmax jrk: " << initial_traj.getMaxJerkRate());

    vector<StatePVA> vis_x;
    vis_x.clear();
    initial_traj.sampleOneSeg(&vis_x);
    vis_ptr_->visualizeStates(vis_x, FirstTraj, ros::Time::now());

    /* optimize */
    vector<VectorXd> middle_nodes_state;
    vector<double> seg_durs;
    VectorXd node(9);
    double initial_T = initial_total_T / seg_num_;
    for (int i=1; i<seg_num_; ++i)
    {
      double t(initial_T * i); 
      node.head(3) = initial_traj.getPos(t);
      node.segment(3, 3) = initial_traj.getVel(t);
      node.tail(3) = initial_traj.getAcc(t);
      middle_nodes_state.push_back(node);
      seg_durs.emplace_back(initial_T);
    }
    seg_durs.emplace_back(initial_T);
    double cost(0.0);
    bool res = traj_optimizer_ptr_->optimizeTraj(middle_nodes_state, seg_durs, head, tail, cost);
    if (res)
    {
      vector<CoefficientMat> coeffs;
      CoefficientMat coeff; /* [cx5,cx4,cx3,cx2,cx1,cx0; 
                                cy5,cy4,cy3,cy2,cy1,cy0; 
                                cz5,cz4,cz3,cz2,cz1,cz0] */
      double T;
      VectorXd x0(9), x1(9);
      for (int i=0; i<seg_num_; ++i)
      {
        T = seg_durs[i];
        /*boundary_condition to coeff */
        double T2(T*T), T3(T2*T), T4(T3*T), T5(T4*T);
        if (i == 0)
        {
          x0 = head;
          x1 = middle_nodes_state[i];
        }
        else if (i == seg_num_ - 1)
        {
          x0 = middle_nodes_state[i - 1];
          x1 = tail;
        }
        else 
        {
          x0 = middle_nodes_state[i - 1];
          x1 = middle_nodes_state[i];
        }
        coeff.col(0) = (0.5 * (x1.segment(6, 3) - x0.segment(6, 3)) * T2 -
                        3.0 * (x0.segment(3, 3) + x1.segment(3, 3)) * T +
                        6.0 * (x1.segment(0, 3) - x0.segment(0, 3))) / T5;
        coeff.col(1) = ((-x1.segment(6, 3) + 1.5 * x0.segment(6, 3)) * T2 +
                        (8.0 * x0.segment(3, 3) + 7.0 * x1.segment(3, 3)) * T +
                        15.0 * (-x1.segment(0, 3) + x0.segment(0, 3))) / T4;
        coeff.col(2) = ((0.5 * x1.segment(6, 3) - 1.5 * x0.segment(6, 3)) * T2 -
                        (6.0 * x0.segment(3, 3) + 4.0 * x1.segment(3, 3)) * T +
                        10.0 * (x1.segment(0, 3) - x0.segment(0, 3))) / T3;
        coeff.col(3) = 0.5 * x0.segment(6, 3);
        coeff.col(4) = x0.segment(3, 3);
        coeff.col(5) = x0.segment(0, 3);
        coeffs.emplace_back(coeff);
      }
      traj_ = Trajectory(seg_durs, coeffs);
      ROS_INFO_STREAM("optimized T: " << traj_.getTotalDuration() 
                 << "\tcost: " << cost
                 << "\tmax vel: " << traj_.getMaxVelRate() 
                 << "\tmax acc: " << traj_.getMaxAccRate() 
                 << "\tmax jrk: " << traj_.getMaxJerkRate());
      return true;
    }
    return false;
  }
  else
  {
    ROS_ERROR("sth wrong with bvp solver");
    return false;
  }

}

void FSM::getPlanStartState(VectorXd& state)
{
  if (machine_state_ == WAIT_GOAL && started_)
  {
    double t_during_traj = traj_.getTotalDuration();
    state.head(3) = traj_.getPos(t_during_traj);
    state.segment(3, 3) = traj_.getVel(t_during_traj);
    state.tail(3) = traj_.getAcc(t_during_traj);
  }
  else if (machine_state_ == FOLLOW_TRAJ)
  {
    double t_during_traj = (ros::Time::now() - curr_traj_start_time_).toSec();
    state.head(3) = traj_.getPos(t_during_traj);
    state.segment(3, 3) = traj_.getVel(t_during_traj);
    state.tail(3) = traj_.getAcc(t_during_traj);
  }
  /* round */
  double vel_norm = state.segment(3, 3).norm();
  double acc_norm = state.tail(3).norm();
  if (vel_norm > vel_limit_)
  {
    state.segment(3, 3) /= vel_norm;
    state.segment(3, 3) *= vel_limit_;
    ROS_WARN_STREAM("vel rounded from " << vel_norm << " to " << vel_limit_);
  }
  if (acc_norm > acc_limit_)
  {
    state.tail(3) /= acc_norm;
    state.tail(3) *= acc_limit_;
    ROS_WARN_STREAM("acc rounded from " << acc_norm << " to " << acc_limit_);
  }
}

inline void FSM::changeState(FSM::MACHINE_STATE new_state)
{
  machine_state_ = new_state;
}