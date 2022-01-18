#pragma once

#include <iostream>
#include <sstream>
#include <unordered_set>
#include <unordered_map>
#include <vector>

struct Problem { 
  struct Job {
    int jobNum;

    struct Operation{
      int jobNum;
      int opNum;
      std::set<int> prevOps;
      std::set<int> nextOps; 
      std::unordered_map<int, int> machineNumToOpDuration;

      inline std::string stringRep() const { return std::to_string(jobNum)+"-"+std::to_string(opNum); }
    };

    std::unordered_map<int, Operation> operations;  
  };

  struct Machine {
    int machineNum;
    std::unordered_map<int, int> machineNumToTrpDuration;
  };

  std::unordered_map<int, Job> jobs;
  std::unordered_map<int, Machine> machines;
  int numOfTransporters;

  static Problem loadProblem(const std::string input_file);
};

void getNum(const std::string string_rep, int& job_num, int& op_num); // Get jobNum and opNum from stringRep()

std::ostream& operator<<(std::ostream&, const Problem::Job::Operation&); 
std::ostream& operator<<(std::ostream&, const Problem::Job&);
std::ostream& operator<<(std::ostream&, const Problem::Machine&); 
std::ostream& operator<<(std::ostream&, const Problem&);