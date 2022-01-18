#include <algorithm>
#include <sstream>
#include <set>
#include <map>
#include <bits/stdc++.h>

#include "problem.hh"
#include "encoded_solution.hh"
#include "decoded_solution.hh"


std::vector<std::string> generateOS(const Problem& problem) {
  std::vector<std::string> OS;

  std::unordered_set<std::string> releasedOpList;
  std::unordered_set<std::string> insertedOpList;
  for(auto& job: problem.jobs) {
    for(auto& op: job.second.operations) {
      if(op.second.prevOps.empty())
        releasedOpList.insert(op.second.stringRep());
    }
  }

  while(!releasedOpList.empty()) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    srand((time_t)ts.tv_nsec);

    auto it = std::begin(releasedOpList);
    std::advance(it, rand() % releasedOpList.size());
    std::string randomOp = *it;
    OS.push_back(randomOp);
    insertedOpList.insert(randomOp);
    releasedOpList.erase(releasedOpList.find(randomOp));

    int job_num, op_num;

    getNum(randomOp, job_num, op_num);
     
    std::set<int> next_ops = problem.jobs.at(job_num).operations.at(op_num).nextOps; 
    for(auto& next: next_ops) {
      std::string next_str = std::to_string(job_num)+"-"+std::to_string(next);
      if(insertedOpList.find(next_str) != insertedOpList.end())
        continue;
      bool is_released = true;
      std::set<int> prev_ops = problem.jobs.at(job_num).operations.at(next).prevOps;
      for(auto& prev: prev_ops) {
        std::string prev_str = std::to_string(job_num)+"-"+std::to_string(prev);
        if(insertedOpList.find(prev_str) == insertedOpList.end()) {
          is_released = false;
          break;
        }
      }
      if(is_released)
        releasedOpList.insert(next_str);
    }
  }
  return OS;
}

std::vector<int> generateMS(const Problem& problem) {
  std::vector<int> MS;

  for(int job_num=1; job_num <= problem.jobs.size(); job_num++) {
    for(int op_num=1; op_num <= problem.jobs.at(job_num).operations.size(); op_num++) {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      srand((time_t)ts.tv_nsec);

      int num_of_machine = problem.jobs.at(job_num).operations.at(op_num).machineNumToOpDuration.size();
      int machine_choice = rand() % num_of_machine;

      MS.push_back(machine_choice);
    }
  }
  return MS;
}

std::vector<int> generateTS(const Problem& problem) {
  std::vector<int> TS;

  for(int job_num=1; job_num <= problem.jobs.size(); job_num++) {
    for(int op_num=1; op_num <= problem.jobs.at(job_num).operations.size(); op_num++) {

      std::set<int> sorted_next = std::set<int>(problem.jobs.at(job_num).operations.at(op_num).nextOps.begin(),
                                                problem.jobs.at(job_num).operations.at(op_num).nextOps.end());
      for(auto& next : sorted_next) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        srand((time_t)ts.tv_nsec);

        int transporter_choice = rand() % problem.numOfTransporters + 1;

        TS.push_back(transporter_choice);
      }
    }
  }
  return TS;
}

EncodedSolution EncodedSolution::generateRandom(const Problem& problem) {

  std::vector<std::string> OS {generateOS(problem)};
  std::vector<int> MS {generateMS(problem)};
  std::vector<int> TS {generateTS(problem)};

  EncodedSolution solution{OS, MS, TS};

  return solution;
}


std::ostream& operator<<(std::ostream& os, const EncodedSolution& es) {
  os << "(OS: [";
  bool addComma = false;
  for(auto &i: es.OS) {
    if(addComma) os << ", "; addComma = true;
    os << i;
  }
  os << "], MS: [";
  addComma = false;
  for(auto &i: es.MS) {
    if(addComma) os << ", "; addComma = true;
    os << i;
  }
  os << "], TS: [";
  addComma = false;
  for(auto &i: es.TS) {
    if(addComma) os << ", "; addComma = true;
    os << i;
  }
  os << "])";
  return os;
}

EncodedSolution getBestSolution(const EncodedSolutionSet& set,
                                const Problem& problem) {
  std::shared_ptr<EncodedSolution> best_solution;
  int min_makespan = INT_MAX;
  for(auto solution: set) {
    int makespan = decode(*solution, problem).getMakespan();
    if(makespan < min_makespan) {
      best_solution = solution;
      min_makespan = makespan;
    }
  }

  return *best_solution;
}


SortedEncodedSolutionSet sortEncodedSolutionSet(const EncodedSolutionSet& set, 
                                                const Problem& problem) {
 
  std::multimap<int, std::shared_ptr<EncodedSolution>> makespan_to_solution;

  for(auto solution: set) {
    int makespan = decode(*solution, problem).getMakespan();
    makespan_to_solution.insert(std::make_pair(makespan, solution));
  }

  SortedEncodedSolutionSet sorted_set;
  for(auto pair: makespan_to_solution){
    sorted_set.push_back(pair.second);
  }
  
  return sorted_set;
}