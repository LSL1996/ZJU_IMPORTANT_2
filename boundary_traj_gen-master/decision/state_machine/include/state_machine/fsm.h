#ifndef _FSM_H_
#define _FSM_H_

#include "visualization_utils/visualization_utils.h"
#include "poly_opt/traj_optimizer.h"
#include "bvp_solver/bvp_solver.h"
#include "poly_traj_utils/traj_utils.hpp"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Empty.h>

class FSM
{
public:
  FSM() {};
  ~FSM() {};
  void init(const ros::NodeHandle& nh);
    
private:
  double rho_;
  bool new_goal_, started_;
  int seg_num_;
  double vel_limit_, acc_limit_, jrk_limit_;
  VectorXd start_state_, end_state_;
  Trajectory traj_;
  ros::Time curr_traj_start_time_;

  BVPSolver::IntegratorBVP bvp_;
  std::shared_ptr<TrajOpt> traj_optimizer_ptr_;
  VisualRviz::Ptr vis_ptr_;
  
  // ros 
  ros::NodeHandle nh_;
  ros::Subscriber goal_sub_;
  ros::Timer execution_timer_;
  void goalCallback(const nav_msgs::Odometry::ConstPtr& goal_msg);
  void executionCallback(const ros::TimerEvent& event);

  // execution states 
  enum MACHINE_STATE{
    WAIT_GOAL, 
    GENERATE_TRAJ, 
    FOLLOW_TRAJ,
    REPLAN_TRAJ
  };
  MACHINE_STATE machine_state_;
  
  bool planOnce(const VectorXd& head, const VectorXd& tail);
  void getPlanStartState(VectorXd& state);
  void changeState(MACHINE_STATE new_state);
};
    

#endif //_FSM_H_
