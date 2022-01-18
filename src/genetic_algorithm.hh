#pragma once

#include "encoded_solution.hh"
#include "problem.hh"

struct GeneticAlgorithm {
  void selection(const SortedEncodedSolutionSet&, EncodedSolutionSet& pop, 
                 EncodedSolutionSet& elites) const;
  void crossover(EncodedSolutionSet& pop) const;
  void mutation (EncodedSolutionSet& pop) const; 

  const Problem problem;

  const int populationSize; 
  const int tournamentSize;
  const int eliteSize;
  const double crossoverProb;
  const double mutationProb; 
};

void crossoverOS(std::vector<std::string>&, std::vector<std::string>&, 
                 const Problem&);
void crossoverMS(std::vector<int>&, std::vector<int>&);
void crossoverTS(std::vector<int>&, std::vector<int>&);

void mutationOS(std::vector<std::string>&, const Problem&);
void mutationMS(std::vector<int>&, const Problem&);
void mutationTS(std::vector<int>&, const Problem&);
