#pragma once

#include <set>

#include "encoded_solution.hh"

struct DecodedSolution {
  struct MachineTask {
    std::string name;
    int machineNum;
    int start;
    int finish;
    int duration;

    inline bool operator<(const MachineTask &other) const {
      return start < other.start || 
        (start == other.start && machineNum < other.machineNum);
    }
  };

  struct TransporterTask {
    enum class Type {
      PICKUP,
      DELIVERY
    };

    std::string name;
    int transporterNum;
    int start;
    int finish;
    int duration;
    Type taskType;
    int fromMachineNum;
    int toMachineNum;

    inline bool operator<(const TransporterTask &other) const {
      return start < other.start || 
        (start == other.start && transporterNum < other.transporterNum);
    }
  };
  typedef TransporterTask::Type TransporterTaskType;

  std::unordered_map<int, std::set<MachineTask>> machineSchedule;
  std::unordered_map<int, std::set<TransporterTask>> transporterSchedule;

  int getMakespan() const;

  void printToFile(const std::string) const; 
};

DecodedSolution decode(const EncodedSolution&, const Problem&);


std::ostream& operator<<(std::ostream&, const DecodedSolution::MachineTask&);
std::ostream& operator<<(std::ostream&, const DecodedSolution::TransporterTaskType&);
std::ostream& operator<<(std::ostream&, const DecodedSolution::TransporterTask&);
std::ostream& operator<<(std::ostream&, const DecodedSolution&);