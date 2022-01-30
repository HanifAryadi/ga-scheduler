#pragma once

#include <queue>
#include <ros/ros.h>

#include "rosmsg.hh"


class GAScheduler {
public:
  GAScheduler(ros::NodeHandle&);

  void receiveProblem(const ga_scheduler::Problem&);

  void checkProblem(const ros::TimerEvent&);

  void solve(const Problem&);

private:
  ros::NodeHandle nodeHandle;

  ros::Subscriber problemSub;
  ros::Publisher solutionPub;

  ros::Timer checkProbTimer;

  std::queue<Problem> unsolvedProbs;
  std::unique_ptr<Problem> currentProb;
};
