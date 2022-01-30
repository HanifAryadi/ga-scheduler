#include <fstream>
#include <sstream>
#include <set>
#include <map>

#include "problem.hh"

Problem Problem::loadProblem(const std::string input_file) {
  std::ifstream file(input_file.c_str());

  if(!file.is_open()) {
    throw std::runtime_error("File "+ input_file + " is not found");
  }

  std::string line;
  getline(file, line);
  std::istringstream iss(line);

  int num_of_jobs, num_of_machines, num_of_transporters;
  iss >> num_of_jobs >> num_of_machines >> num_of_transporters;

  std::unordered_map<JobNum, Job> jobs;

  for(JobNum i = 1; i <= num_of_jobs; i++) {
    std::getline(file, line);
    iss.clear();
    iss.str(line);

    int num_of_operations;
    iss >> num_of_operations;
    
    std::unordered_map<OpNum, Operation> operations;

    for(OpNum j = 1; j <= num_of_operations; j++) {
      int num_of_machines;
      iss >> num_of_machines;

      std::unordered_map<MachineNum, Duration> machine_num_to_op_duration;

      for(int k = 0; k < num_of_machines; k++) {
        MachineNum machine_num;
        Duration duration;
        iss >> machine_num >> duration;
        machine_num_to_op_duration.insert(
          std::make_pair(machine_num, duration));
      }

      int num_of_prev_ops;
      iss >> num_of_prev_ops;

      std::set<OpNum> prev_ops;

      for(int k = 0; k < num_of_prev_ops; k++) {
        OpNum prev_op_num;
        iss >> prev_op_num;
        prev_ops.insert(prev_op_num);

        operations[prev_op_num].nextOps.insert(j);
      }
      Operation operation{i, j, prev_ops,
                                        std::set<OpNum>(), 
                                        machine_num_to_op_duration};
      operations.insert(std::make_pair(j, operation)); 
    }

    Job job{i, operations};
    jobs.insert(std::make_pair(i, job));
  }

  std::unordered_map<MachineNum, Machine> machines;

  for(MachineNum i = 1; i <= num_of_machines; i++) {
    std::unordered_map<MachineNum, Duration> machine_num_to_trp_duration;

    for(MachineNum j = 1; j <= num_of_machines; j++) {
      machine_num_to_trp_duration.insert(std::make_pair(j, 0));
    }

    Machine machine{i, machine_num_to_trp_duration};
    machines.insert(std::make_pair(i, machine));
  }

  for(int i = 1; i < num_of_machines; i++) {
    std::getline(file, line);
    iss.clear();
    iss.str(line);

    for(int j = 0; j < num_of_machines-i; j++) {
      Duration trp_duration;
      iss >> trp_duration;
      machines[i].machineNumToTrpDuration[i+j+1] = trp_duration;
      machines[i+j+1].machineNumToTrpDuration[i] = trp_duration;
    }
  }

  Problem problem{jobs, machines, num_of_transporters};

  return problem;
}

void getNum(const std::string string_rep, JobNum& job_num, OpNum& op_num) {
  std::vector<std::string> v;
  std::stringstream ss(string_rep);
  while (ss.good()) {
    std::string substr;
    std::getline(ss, substr, '-');
    v.push_back(substr);
  }
  job_num = stoi(v[0]);
  op_num = stoi(v[1]);
}



std::ostream& operator<<(std::ostream& os, const Problem::Operation& o) {
  std::map<MachineNum, Duration> sortedMachineNumToOpDuration(o.machineNumToOpDuration.begin(),
                                                  o.machineNumToOpDuration.end());
  os << "Operation " << o.jobNum << "-" << o.opNum << " (prevOps: [";
  bool addComma = false;
  for(auto &i: o.prevOps) {
    if(addComma) os << ", "; addComma = true;
    os << i;
  }
  os << "], nextOps: [";
  addComma = false;
  for(auto &i: o.nextOps) {
    if(addComma) os << ", "; addComma = true;
    os << i;
  }
  os << "], machineNumToOpDuration: [";
  addComma = false;
  for (auto &i: sortedMachineNumToOpDuration) {
    if(addComma) os << ", "; addComma = true;
    os << "(" << i.first << ", " << i.second << ")"; 
  }
  os << "])";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Problem::Job& j) {
  std::map<OpNum, Problem::Operation> sortedOperations(j.operations.begin(),
                                                          j.operations.end());
  os << "Job " << j.jobNum << " [";
  bool addComma = false;
  for(auto &i: sortedOperations) {
    if(addComma) os << ", "; addComma = true;
    os << i.second;
  }
  os << "]";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Problem::Machine& m) {
  std::map<MachineNum, Duration> sortedMachineNumToTrpDuration(m.machineNumToTrpDuration.begin(),
                                                   m.machineNumToTrpDuration.end());
  os << "Machine " << m.machineNum << " (machineNumToTrpDuration: [";
  bool addComma = false;
  for(auto &i: sortedMachineNumToTrpDuration) {
    if(addComma) os << ", "; addComma = true;
    os << "(" << i.first << ", " << i.second << ")";
  }
  os << "]";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Problem& p) {
  std::map<JobNum, Problem::Job> sortedJobs(p.jobs.begin(),
                                         p.jobs.end());
  std::map<MachineNum, Problem::Machine> sortedMachines(p.machines.begin(),
                                                 p.machines.end());
  os << "Problem (Jobs: [";
  bool addComma = false;
  for(auto& i: sortedJobs) {
    if(addComma) os << ", "; addComma = true;
    os << i.second;
  }
  os << "], Machines: [";
  addComma = false;
  for(auto& i: sortedMachines) {
    if(addComma) os << ", "; addComma = true;
    os << i.second;
  }
  os << "], numOfTransporters: " << p.numOfTransporters << ")";

  return os;
}