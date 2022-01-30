#include <thread>

#include <ros/ros.h>

#include "scheduler_node.hh"

#include "genetic_algorithm.hh"
#include "stat_recorder.hh"
#include "tabu_search.hh"

GAScheduler::GAScheduler(ros::NodeHandle& node_handle) {
  nodeHandle = node_handle;

  problemSub = nodeHandle.subscribe(
    "problem", 10, &GAScheduler::receiveProblem, this);
  solutionPub = nodeHandle.advertise<ga_scheduler::Solution>(
    "solution", 10);

  checkProbTimer = nodeHandle.createTimer(
    ros::Duration(1.0), &GAScheduler::checkProblem, this);
}

void GAScheduler::receiveProblem(const ga_scheduler::Problem& msg) {
  Problem new_problem = loadProblemFromMsg(msg);
  unsolvedProbs.push(new_problem);
}

void GAScheduler::checkProblem(const ros::TimerEvent& /*event*/) {
  if(currentProb) return;
  
  if(!unsolvedProbs.empty()) {
    Problem problem = unsolvedProbs.front();
    unsolvedProbs.pop();
    currentProb = std::make_unique<Problem>(problem);
    return solve(problem);
  }
}

void GAScheduler::solve(const Problem& problem) {

  const int num_of_iters = 100;
  const int population_size = 100;
  const int tournament_size = 2;
  const double elite_prob = 0.05;
  const int elite_size = elite_prob*(double)population_size;
  const double crossover_prob = 0.8;
  const double mutation_prob = 0.1;

  const bool use_tabu_search = false;
  const int num_of_iters_tabu_search = 20;
  const int neighbors_size = 10;

  StatRecorder stat_recorder;
  stat_recorder.startRecord();

  EncodedSolutionSet population;
  for(int i = 0; i < population_size; i++) {
    auto new_indiv = EncodedSolution::generateRandom(problem);
    population.insert(std::make_shared<EncodedSolution>(new_indiv));
  }
  
  SortedEncodedSolutionSet sorted_pop = sortEncodedSolutionSet(population, problem);

  const GeneticAlgorithm geneticAlgorithm{problem, population_size, tournament_size, 
                                          elite_size, crossover_prob, mutation_prob};

  int current_iter = 1;
  while(current_iter <= num_of_iters) {
    EncodedSolutionSet elites;
    geneticAlgorithm.selection(sorted_pop, population, elites);

    std::vector<std::thread> tabu_search_threads;
    if(use_tabu_search) {
      for(auto& elite: elites) {
        TabuSearch tabu_search{problem, *elite, num_of_iters_tabu_search, neighbors_size};
        tabu_search_threads.emplace_back(&TabuSearch::run, tabu_search);
      }
    }

    geneticAlgorithm.crossover(population);
    geneticAlgorithm.mutation(population);

    if(use_tabu_search) {
      for(auto& thread: tabu_search_threads) {
        thread.join();
      }
    }
    population.insert(elites.begin(), elites.end());
    
    sorted_pop = sortEncodedSolutionSet(population, problem);
    
    auto best_solution = *sorted_pop[0];

    stat_recorder.addStat(decode(best_solution, problem).getMakespan());

    ROS_INFO_STREAM(current_iter);
    current_iter++;    
  }

  auto best_solution = *(*sortEncodedSolutionSet(population, problem).begin());

  decode(best_solution, problem).printToFile("output.txt");
  stat_recorder.printToFile("stat.txt");

  ga_scheduler::Solution solution_msg = printMsgFromSolution(decode(best_solution, problem));
  solutionPub.publish(solution_msg);

  currentProb.reset();
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "scheduler");
  ros::NodeHandle node_handle;

  GAScheduler scheduler(node_handle);

  ros::spin();
}