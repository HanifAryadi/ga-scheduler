#include <iostream>
#include <chrono>
#include <thread>

#include <boost/program_options.hpp>

#include "decoded_solution.hh"
#include "encoded_solution.hh"
#include "genetic_algorithm.hh"
#include "problem.hh"
#include "stat_recorder.hh"
#include "tabu_search.hh"


int main(int argc, char** argv) {
  namespace po = boost::program_options;

  const int default_num_of_iters = 100;
  const int default_population_size = 100;
  const int default_tournament_size = 2;
  const double default_elite_prob = 0.05;
  const double default_crossover_prob = 0.8;
  const double default_mutation_prob = 0.1;
  const int default_num_of_iters_tabu_search = 20;
  const int default_neighbors_size = 10;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("input,i", po::value<std::string>()->required(), "input file for name problem")
    ("output,o", po::value<std::string>()->default_value("solution"), "output file name for solution")
    ("num_of_iters,n", po::value<int>()->default_value(default_num_of_iters), "number of iterations")
    ("population_size,p", po::value<int>()->default_value(default_population_size), "population size of genetic algorithm")
    ("tournament_size", po::value<int>()->default_value(default_tournament_size), "tournament size of selection operation")
    ("elite_prob", po::value<double>()->default_value(default_elite_prob), "probability of elites")
    ("crossover_prob", po::value<double>()->default_value(default_crossover_prob), "probability of crossover operation")
    ("mutation_prob", po::value<double>()->default_value(default_mutation_prob), "probability of mutation operation")
    ("use_tabu_search,t", po::value<bool>()->default_value(false), "use tabu search (default = false)")
    ("num_of_iters_tabu_search", po::value<int>()->default_value(default_num_of_iters_tabu_search), "number of iterations in tabu search")
    ("neighbors_size", po::value<int>()->default_value(default_neighbors_size), "neighbors size of tabu search");

  po::variables_map temp;
  po::store(po::parse_command_line(argc, argv, desc), temp);
  const po::variables_map vm = temp;

  const std::string input_file = vm["input"].as<std::string>();
  const std::string output_file = vm["output"].as<std::string>();

  const int num_of_iters = vm["num_of_iters"].as<int>();
  const int population_size = vm["population_size"].as<int>();
  const int tournament_size = vm["tournament_size"].as<int>();
  const int elite_size = vm["elite_prob"].as<double>()*(double)population_size;
  const double crossover_prob = vm["crossover_prob"].as<double>();
  const double mutation_prob = vm["mutation_prob"].as<double>();

  const bool use_tabu_search = vm["use_tabu_search"].as<bool>();
  const int num_of_iters_tabu_search = vm["num_of_iters_tabu_search"].as<int>();
  const int neighbors_size = vm["neighbors_size"].as<int>();

  const Problem problem = Problem::loadProblem(input_file);

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

    current_iter++;
  }

  auto best_solution = *(*sortEncodedSolutionSet(population, problem).begin());

  decode(best_solution, problem).printToFile("output.txt");
  stat_recorder.printToFile("stat.txt");

  return 0;
}