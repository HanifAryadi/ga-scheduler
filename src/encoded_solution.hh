#pragma once

#include <string>
#include <vector>
#include <memory>

#include "problem.hh"

struct EncodedSolution {
  std::vector<std::string> OS; 
  std::vector<int> MS;
  std::vector<int> TS;

  static EncodedSolution generateRandom(const Problem&);

  inline bool operator ==(const EncodedSolution& other) const {
    return OS == other.OS  && MS == other.MS && TS == other.TS;
  }
};

std::ostream& operator<<(std::ostream&, const EncodedSolution&);


typedef std::unordered_set<std::shared_ptr<EncodedSolution>> EncodedSolutionSet; 
typedef std::vector<std::shared_ptr<EncodedSolution>> SortedEncodedSolutionSet;

EncodedSolution getBestSolution(const EncodedSolutionSet&, const Problem&);
SortedEncodedSolutionSet sortEncodedSolutionSet(const EncodedSolutionSet&, const Problem&);