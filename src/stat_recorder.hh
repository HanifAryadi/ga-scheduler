#pragma once

#include <vector>
#include <chrono>

struct StatRecorder {
  struct Stat {
    int time;
    int makespan;
  };
  
  std::vector<Stat> stats; 

  std::chrono::steady_clock::time_point startTime; 

  void startRecord();

  void addStat(int makespan);

  void printToFile(const std::string) const;
};

