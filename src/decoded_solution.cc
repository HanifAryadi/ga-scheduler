#include <map>
#include <bits/stdc++.h>
#include <memory>
#include <iostream>
#include <fstream>

#include "encoded_solution.hh"
#include "decoded_solution.hh"
#include "problem.hh"


std::unordered_map<int, std::unordered_map<int, int>> splitMS(const std::vector<int> MS, const Problem& problem) {

  std::unordered_map<int, std::unordered_map<int, int>> splitted_MS;

  int pos = 0;
  for(int job_num=1; job_num <= problem.jobs.size(); job_num++) {
    std::unordered_map<int, int> splitted_MS_per_job;

    for(int op_num=1; op_num <= problem.jobs.at(job_num).operations.size(); op_num++) {
      splitted_MS_per_job.insert(std::make_pair(op_num, MS[pos]));
      pos++;
    }
    splitted_MS.insert(std::make_pair(job_num, splitted_MS_per_job));
  }

  return splitted_MS;
}

std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, int>>> 
  splitTS(const std::vector<int> TS, const Problem& problem) {

  std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, int>>> splitted_TS;

  int pos = 0;
  for(int job_num=1; job_num <= problem.jobs.size(); job_num++) {
    std::unordered_map<int, std::unordered_map<int, int>> splitted_TS_per_job;

    for(int op_num=1; op_num <= problem.jobs.at(job_num).operations.size(); op_num++) {
      std::unordered_map<int, int> splitted_TS_per_op;

      for(auto& next : problem.jobs.at(job_num).operations.at(op_num).nextOps) {
        splitted_TS_per_op.insert(std::make_pair(next, TS[pos]));
        pos++;
      }
      splitted_TS_per_job.insert(std::make_pair(op_num, splitted_TS_per_op));
    }
    splitted_TS.insert(std::make_pair(job_num, splitted_TS_per_job));
  }

  return splitted_TS;
}

void addMachineTask(std::string op_name, int earliest_start, int duration, int machine_num, 
                    int& finish, std::set<DecodedSolution::MachineTask>& tasks) {
  int max_start = earliest_start;

  for(auto& task: tasks) {
    int start = task.finish;
    if(start > max_start) max_start = start;
  }

  std::vector<bool> is_free(max_start + duration, true);

  for(auto& task: tasks) {
    for(int t = task.start; t < task.finish; t++) {
      is_free[t] = false;
    }
  }

  int start = earliest_start;
  bool is_possible = false;
  while(!is_possible) {
    is_possible = true;
    for(int t = start; t < start+duration; t++) {
      if(!is_free[t]) {
        is_possible = false;
        break;
      }
    }
    start++;
  }
  start--;
  finish = start + duration;

  DecodedSolution::MachineTask new_task{op_name, machine_num, start, finish, duration};
  tasks.insert(new_task);
}

void addTransporterTask(std::string from_op_name, std::string to_op_name, int earliest_start, 
                        int transporter_num, int from_machine_num, int to_machine_num, 
                        const std::unordered_map<int, Problem::Machine>& machines, 
                        int& finish, std::set<DecodedSolution::TransporterTask>& tasks) {
  int duration = machines.at(from_machine_num).machineNumToTrpDuration.at(to_machine_num);
  int max_start = earliest_start;

  for(auto& task: tasks) {
    int start = task.finish + machines.at(task.toMachineNum).machineNumToTrpDuration.at(from_machine_num);
    if(start > max_start) max_start = start;
  }

  std::vector<bool> is_free(max_start + duration, true);
  std::vector<int> loc(max_start + duration, 0);


  for(auto& task: tasks) {
    for(int t = task.start; t < task.finish; t++) {
      is_free[t] = false;
    }
    for(int t = task.finish; t < max_start + duration; t++) {
      loc[t] = task.toMachineNum;
    }
  }

  int start = earliest_start;
  int prev_loc = 0;
  bool is_possible = false;
  while(!is_possible) {
    is_possible = true;
    prev_loc = loc[start];

    if (prev_loc != 0) {
      int pickup_duration = machines.at(prev_loc).machineNumToTrpDuration.at(from_machine_num);
      if (start < pickup_duration) {
        is_possible = false;
        goto next_iter;
      }
      for(int t = start - pickup_duration; t < start; t++) {
        if(!is_free[t]) {
          is_possible = false;
          goto next_iter;
        }
      }
    }

    for(int t = start; t < start+duration; t++) {
      if(!is_free[t]) {
        is_possible = false;
        goto next_iter;
      }
    }

    if(!tasks.empty()) {
      std::unique_ptr<DecodedSolution::TransporterTask> next_delivery;
      int next_delivery_start = INT_MAX;
      
      for(auto& task: tasks) {
        if(task.taskType == DecodedSolution::TransporterTaskType::DELIVERY &&
          start + duration <= task.start && task.start < next_delivery_start) {
          
          next_delivery = std::make_unique<DecodedSolution::TransporterTask>(task);
          next_delivery_start = next_delivery->start;
        }
      }

      if(next_delivery) {
        if(to_machine_num == next_delivery->fromMachineNum) {
          std::string next_delivery_name = next_delivery->name;
          std::string next_pickup_name = "->" + next_delivery_name.substr(0, next_delivery_name.find("->"));
          std::unique_ptr<DecodedSolution::TransporterTask> next_pickup;

          for(auto& task: tasks) {
            if(task.name == next_pickup_name) {
              next_pickup = std::make_unique<DecodedSolution::TransporterTask>(task);
              break;
            }
          }
          if(next_pickup) tasks.erase(*next_pickup);
        }
        else {
          int next_pickup_duration = machines.at(to_machine_num).machineNumToTrpDuration.at(next_delivery->fromMachineNum);

          if(start + duration + next_pickup_duration > next_delivery_start) {
            is_possible = false; 
            goto next_iter;
          } 

          std::string next_delivery_name = next_delivery->name;
          std::string next_pickup_name = "->" + next_delivery_name.substr(0, next_delivery_name.find("->"));
          std::unique_ptr<DecodedSolution::TransporterTask> next_pickup;

          for(auto& task: tasks) {
            if(task.name == next_pickup_name) {
              next_pickup = std::make_unique<DecodedSolution::TransporterTask>(task);
              break;
            }
          }
          if(next_pickup) tasks.erase(*next_pickup);

          DecodedSolution::TransporterTask new_pickup{next_pickup_name, transporter_num, next_delivery->start - next_pickup_duration, 
                                                      next_delivery->start, next_pickup_duration, 
                                                      DecodedSolution::TransporterTaskType::PICKUP,
                                                      to_machine_num, next_delivery->fromMachineNum};
          
          tasks.insert(new_pickup);   
        }
      }
    }

    
    next_iter:
    start++;
  }
  start--;
  finish = start + duration;

  if(prev_loc != from_machine_num && prev_loc != 0) {
    std::string pickup_name = "->" + from_op_name;
    int pickup_duration = machines.at(prev_loc).machineNumToTrpDuration.at(from_machine_num);
    DecodedSolution::TransporterTask pickup{pickup_name, transporter_num, start - pickup_duration, start, 
                                            pickup_duration, DecodedSolution::TransporterTaskType::PICKUP,
                                            prev_loc, from_machine_num};
    tasks.insert(pickup);                                   
  }


  std::string delivery_name = from_op_name + "->" + to_op_name;
  DecodedSolution::TransporterTask delivery{delivery_name, transporter_num, start, finish, 
                                                 duration, DecodedSolution::TransporterTaskType::DELIVERY,
                                                 from_machine_num, to_machine_num};
  tasks.insert(delivery);
}


DecodedSolution decode(const EncodedSolution& code, const Problem& problem) {
  std::unordered_map<MachineNum, 
                     std::set<DecodedSolution::MachineTask>> 
                     machine_schedule;
  std::unordered_map<TransporterNum, 
                     std::set<DecodedSolution::TransporterTask>> 
                     transporter_schedule;

  // Initialize empty schedule
  for(MachineNum i = 1; i <= problem.machines.size(); i++) {
    machine_schedule.insert(std::make_pair(i, std::set<DecodedSolution::MachineTask>()));
  }
  for(TransporterNum i = 1; i <= problem.numOfTransporters; i++) {
    transporter_schedule.insert(std::make_pair(i, std::set<DecodedSolution::TransporterTask>()));
  }

  // Get splitted MS and TS
  auto splitted_MS = splitMS(code.MS, problem);
  auto splitted_TS = splitTS(code.TS, problem);

  // Initialize earliest starts for each operation to zero
  std::unordered_map<int, std::unordered_map<int, int>> earliest_starts;
  for(auto& i: problem.jobs) {
    std::unordered_map<int, int> earliest_starts_per_job;

    for(auto& j: i.second.operations) {
      earliest_starts_per_job.insert(std::make_pair(j.second.opNum, 0));
    }
    earliest_starts.insert(std::make_pair(i.second.jobNum, earliest_starts_per_job));
  }
  
  for(auto op_name: code.OS) {
    int job_num, op_num;
    getNum(op_name, job_num, op_num);

    // Get machineNum and duration for the operation
    int machine_index = splitted_MS.at(job_num).at(op_num);
    auto it = std::begin(problem.jobs.at(job_num).operations.at(op_num).machineNumToOpDuration);
    std::advance(it, machine_index);
    int machine_num = it->first;
    int duration = it->second;

    // Find time slot, add to the schedule, and get the finish time
    int finish = 0;
    addMachineTask(op_name, earliest_starts.at(job_num).at(op_num), duration, 
                   machine_num, finish, machine_schedule.at(machine_num));

    for(auto& next: problem.jobs.at(job_num).operations.at(op_num).nextOps) {
      // Get machineNum for the next operation
      int next_machine_index = splitted_MS.at(job_num).at(next);
      auto it_ = std::begin(problem.jobs.at(job_num).operations.at(next).machineNumToOpDuration);
      std::advance(it_, next_machine_index);
      int next_machine_num = it_->first;

      // If same machine is used for the next operation, just update earliest start time
      if(next_machine_num == machine_num) {
        if(earliest_starts.at(job_num).at(next) < finish) 
          earliest_starts.at(job_num).at(next) = finish;
        continue;
      }

      // Get transporterNum
      int transporter_num = splitted_TS.at(job_num).at(op_num).at(next);

      // Find time slot, add to the schedule, and get the finish time
      std::string next_op_name = std::to_string(job_num) + "-" + std::to_string(next);
      int delivery_finish = 0;
      addTransporterTask(op_name, next_op_name, finish, transporter_num,
                         machine_num, next_machine_num, problem.machines,
                         delivery_finish, transporter_schedule.at(transporter_num));

      if(earliest_starts.at(job_num).at(next) < delivery_finish) 
        earliest_starts.at(job_num).at(next) = delivery_finish;
    }
  }

  DecodedSolution solution{machine_schedule, transporter_schedule};

  return solution;
}


Duration DecodedSolution::getMakespan() const {
  int makespan = 0;
  for(auto& machine: machineSchedule) {
    if(machine.second.empty()) continue;
    int finish = machine.second.rbegin()->finish;
    if(finish > makespan) 
      makespan = finish;
  }
  for(auto& transporter: transporterSchedule) {
    if(transporter.second.empty()) continue;
    int finish = transporter.second.rbegin()->finish;
    if(finish > makespan) 
      makespan = finish;
  }

  return makespan;
}

void DecodedSolution::printToFile(const std::string file_name) const {
  std::ofstream output_file;
  output_file.open(file_name);
  output_file << machineSchedule.size() <<" " << transporterSchedule.size() <<std::endl;
  for(auto& machine: machineSchedule) {
    output_file << machine.first <<" ";
    for(auto& task: machine.second) {
      output_file << task.start << " " << task.finish << " " << task.name << " ";
    }
    output_file << std::endl;
  }
  for(auto& machine: transporterSchedule) {
    output_file << machine.first <<" ";
    for(auto& task: machine.second) {
      output_file << task.start << " " << task.finish << " " << task.name << " ";
    }
    output_file << std::endl;
  }

  output_file.close();
}





std::ostream& operator<<(std::ostream& os, const DecodedSolution::MachineTask& mt) {
  os << "Task " << mt.name;
  os << " (machineNum: " << mt.machineNum;
  os << ", start: " << mt.start;
  os << ", finish: " << mt.finish;
  os << ", duration: " << mt.duration << ")";
  return os;
}

std::ostream& operator<<(std::ostream& os, const DecodedSolution::TransporterTaskType& t) {
  switch(t){
    case DecodedSolution::TransporterTask::Type::PICKUP:
      os << "PICKUP";
      break;
    case DecodedSolution::TransporterTask::Type::DELIVERY:
      os << "DELIVERY";
      break;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const DecodedSolution::TransporterTask& tt) {
  os << "Task " << tt.name;
  os << " (transporterNum: " << tt.transporterNum;
  os << ", start: " << tt.start;
  os << ", finish: " << tt.finish;
  os << ", duration: " << tt.duration;
  os << ", taskType: " << tt.taskType;
  os << ", fromMachineNum: " << tt.fromMachineNum;
  os << ", toMachineNum: " << tt.toMachineNum << ")";
  return os;
}

std::ostream& operator<<(std::ostream& os, const DecodedSolution& ds) {
  std::map<int, std::set<DecodedSolution::MachineTask>> sortedMachineSchedule(ds.machineSchedule.begin(),
                                                                              ds.machineSchedule.end());
  std::map<int, std::set<DecodedSolution::TransporterTask>> sortedTransporterSchedule(ds.transporterSchedule.begin(),
                                                                                      ds.transporterSchedule.end());
  os << "Schedule (";
  bool addComma = false;
  for(auto& i: sortedMachineSchedule) {
    if(addComma) os << ", ";
    os << "Machine " << i.first << " [";
    addComma = false;
    for(auto& j: i.second) {
      if(addComma) os << ", "; addComma = true;
      os << j;
    }
    os <<"]";
    addComma = true;
  }
  for(auto& i: sortedTransporterSchedule) {
    if(addComma) os << ", "; addComma = true;
    os << "Transporter " << i.first << " [";
    addComma = false;
    for(auto& j: i.second) {
      if(addComma) os << ", "; addComma = true;
      os << j;
    }
    os <<"]";
    addComma = true;
  }
  os <<")";

  return os;
}