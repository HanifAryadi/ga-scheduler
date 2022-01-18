#include <algorithm>
#include <map>

#include "tabu_search.hh"

void TabuSearch::run() {
  int min_makespan = decode(solution, problem).getMakespan();

  std::vector<EncodedSolution> tabu_list; //TODO change to unordered_set to allow find in constant time (?)
  tabu_list.push_back(solution);

  int current_iter = 1;
  while(current_iter <= iterationsNum) {
    auto neighbors = generateNeighbors(solution, problem, neighborsSize);

    int i = 0;
    std::shared_ptr<EncodedSolution> best;

    while(best == nullptr && i < neighbors.size()) {
      if(std::find(tabu_list.begin(), tabu_list.end(), *neighbors[i]) == tabu_list.end()) {
        best = neighbors[i];
      }
      i++;
    }

    if(best == nullptr) continue;

    tabu_list.push_back(*best);

    int makespan = decode(*best, problem).getMakespan();
    if(makespan < min_makespan) {
      solution = *best;
      min_makespan = makespan;
    }

    current_iter++;
  }
}

SortedEncodedSolutionSet generateNeighbors(const EncodedSolution& solution, const Problem& problem,
                                           int neighbors_size) {
  std::multimap<int, std::shared_ptr<EncodedSolution>> makespan_to_neighbors;
  

  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  srand((time_t)ts.tv_nsec);

  for(int i = 0; i < neighbors_size; i++) {
    EncodedSolution solution_copy(solution);

    int choice = rand() % 3;
    switch(choice) {
      case 0:
        mutationOS(solution_copy.OS, problem);
        break;
      case 1:
        mutationMS(solution_copy.MS, problem);
        break;
      case 2:
        mutationTS(solution_copy.TS, problem);
        break;
    }
    int makespan = decode(solution_copy, problem).getMakespan();
    makespan_to_neighbors.insert(std::make_pair(makespan, 
                                                std::make_shared<EncodedSolution>(solution_copy)));
  }

  SortedEncodedSolutionSet sorted_neighbors;
  for(auto pair: makespan_to_neighbors){
    sorted_neighbors.push_back(pair.second);
  }

  return sorted_neighbors; 
}