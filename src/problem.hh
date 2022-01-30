#pragma once

#include <iostream>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <set>


typedef int JobNum;
typedef int OpNum;
typedef int MachineNum;
typedef int TransporterNum;
typedef int Duration;

struct Problem { 
  
  struct Operation{
    JobNum jobNum;
    OpNum opNum;
    std::set<OpNum> prevOps;
    std::set<OpNum> nextOps; 
    std::unordered_map<MachineNum, Duration> machineNumToOpDuration;

    inline std::string stringRep() const { return std::to_string(jobNum)+"-"+std::to_string(opNum); }
  };

  struct Job {
    JobNum jobNum;

    std::unordered_map<OpNum, Operation> operations;  
  };

  struct Machine {
    MachineNum machineNum;
    std::unordered_map<MachineNum, Duration> machineNumToTrpDuration;
  };

  std::unordered_map<JobNum, Job> jobs;
  std::unordered_map<MachineNum, Machine> machines;
  int numOfTransporters;

  static Problem loadProblem(const std::string input_file);
};

void getNum(const std::string string_rep, JobNum& job_num, OpNum& op_num); // Get jobNum and opNum from stringRep()

std::ostream& operator<<(std::ostream&, const Problem::Operation&); 
std::ostream& operator<<(std::ostream&, const Problem::Job&);
std::ostream& operator<<(std::ostream&, const Problem::Machine&); 
std::ostream& operator<<(std::ostream&, const Problem&);