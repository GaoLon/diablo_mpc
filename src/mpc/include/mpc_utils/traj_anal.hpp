#pragma once

#include "mpc/Polynome.h"
#include "mpc_utils/minco.hpp"
#include <ros/ros.h>

class MPCState
{
public:
    double x = 0;
    double y = 0;
    double v = 0; 
    double theta = 0;
};

class TrajPoint
{
public:
    double x = 0;
    double y = 0;
    double v = 0; 
    double a = 0;
    double theta = 0;
    double w = 0;
};

class TrajAnalyzer{
private:
    mpc_utils::MinJerkOpt jerk_opt;
    mpc_utils::Trajectory minco_traj;
    ros::Time start_time;
    double traj_duration;

public:

    bool back_track = false;
    bool at_goal = false;

    TrajAnalyzer() {}

    void setTraj(mpc::PolynomeConstPtr msg)
    {
      start_time = msg->start_time;

      Eigen::MatrixXd posP(3, msg->pos_pts.size() - 2);
      Eigen::VectorXd T(msg->t_pts.size());
      Eigen::MatrixXd initS, tailS;

      for (int i = 1; i < (int)msg->pos_pts.size() - 1; i++)
      {
        posP(0, i - 1) = msg->pos_pts[i].x;
        posP(1, i - 1) = msg->pos_pts[i].y;
        posP(2, i - 1) = msg->pos_pts[i].z;
      }
      for (int i = 0; i < (int)msg->t_pts.size(); i++)
      {
        T(i) = msg->t_pts[i];
      }

      initS.setZero(3, 3);
      tailS.setZero(3, 3);
      initS.col(0) = Eigen::Vector3d(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
      initS.col(1) = Eigen::Vector3d(msg->init_v.x, msg->init_v.y, msg->init_v.z);
      initS.col(2) = Eigen::Vector3d(msg->init_a.x, msg->init_a.y, msg->init_a.z);
      tailS.col(0) = Eigen::Vector3d(msg->pos_pts.back().x, msg->pos_pts.back().y, msg->pos_pts.back().z);
      tailS.col(1) = Eigen::Vector3d::Zero();
      tailS.col(2) = Eigen::Vector3d::Zero();
      jerk_opt.reset(initS, msg->pos_pts.size() - 1);
      jerk_opt.generate(posP, tailS, T);
      minco_traj = jerk_opt.getTraj();
      traj_duration = minco_traj.getTotalDuration();
    }

    // get check point by time
    vector<TrajPoint> getTrackPoint(int T, double dt)
    {
      double t0 = (ros::Time::now() - start_time).toSec();
      vector<TrajPoint> tp;

      return tp;
    }

   
    ~TrajAnalyzer() {}
};