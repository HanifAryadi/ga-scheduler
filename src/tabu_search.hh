#pragma once 

#include "decoded_solution.hh"
#include "encoded_solution.hh"
#include "genetic_algorithm.hh"
#include "problem.hh"

struct TabuSearch {
  void run();

  const Problem problem;

  EncodedSolution& solution;

  const int iterationsNum;
  const int neighborsSize;
};

SortedEncodedSolutionSet generateNeighbors(const EncodedSolution&, const Problem&, 
                                           int neighbors_size);