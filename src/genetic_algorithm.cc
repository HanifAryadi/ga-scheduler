#include <set>

#include "encoded_solution.hh"
#include "genetic_algorithm.hh"

void GeneticAlgorithm::selection(const SortedEncodedSolutionSet& sorted_pop, EncodedSolutionSet& pop, 
                                 EncodedSolutionSet& elites) const{
  pop.clear();

  for(auto it = sorted_pop.begin(); it < sorted_pop.begin() + eliteSize; it++) {
    EncodedSolution indiv_copy(*(*it));
    elites.insert(std::make_shared<EncodedSolution>(indiv_copy));
  }

  while(pop.size() < populationSize) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    srand((time_t)ts.tv_nsec);

    std::set<int> tournament_rank;

    for(int i = 0; i < tournamentSize; i++) {
      tournament_rank.insert(rand() % sorted_pop.size());
    }

    EncodedSolution indiv_copy(*sorted_pop[*tournament_rank.begin()]);
    pop.insert(std::make_shared<EncodedSolution>(indiv_copy));
  }
}

void crossoverOS(std::vector<std::string>& OS1, std::vector<std::string>& OS2, const Problem& problem) {
  int num_of_jobs = problem.jobs.size();

  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  srand((time_t)ts.tv_nsec);
  int crossover_type = rand()%2;
  
  int max_job_set_size = rand() % (num_of_jobs-1) + 1;  
  std::unordered_set<int> job_set;
  for(int i = 0; i < max_job_set_size; i++) {
    int job_num = rand() % (num_of_jobs) + 1;
    job_set.insert(job_num);
  } 

  std::vector<std::string> OS1_copy(OS1), OS2_copy(OS2), OS1_added, OS2_added;

  for(int i = 0; i < OS1.size(); i++) {
    int job_num, op_num;
    getNum(OS1_copy[i], job_num, op_num);

    if(job_set.find(job_num) != job_set.end()){
      OS1[i] = OS1_copy[i];
      if(crossover_type == 0) OS1_added.push_back(OS1_copy[i]);
    }
    else {
      OS1[i] = "X";
      if(crossover_type == 1) OS1_added.push_back(OS1_copy[i]);
    }

    getNum(OS2_copy[i], job_num, op_num);

    if(job_set.find(job_num) != job_set.end()){
      OS2[i] = "X";
      if(crossover_type == 0) OS2[i] = "X";
      else OS2[i] = OS2_copy[i];
    }
    else {
      if(crossover_type == 0) OS2[i] = OS2_copy[i];
      else OS2[i] = "X";
      OS2_added.push_back(OS2_copy[i]);
      
    }
  }

  int j = 0;
  for(int i = 0; i < OS1.size(); i++) {
    if(OS1[i] == "X"){
      OS1[i] = OS2_added[j];
      j++;
    }
  }

  j = 0;
  for(int i = 0; i < OS2.size(); i++) {
    if(OS2[i] == "X"){
      OS2[i] = OS1_added[j];
      j++;
    }
  }
}

void crossoverMS(std::vector<int>& MS1, std::vector<int>& MS2) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  srand((time_t)ts.tv_nsec);
  
  int pos_1 = rand() % (MS1.size()-1);
  int pos_2 = rand() % (MS1.size()-1);

  if(pos_1 == pos_2) return;

  if(pos_1 > pos_2) {
    int temp = pos_2;
    pos_2 = pos_1;
    pos_1 = temp;
  }

  std::vector<int> croppedMS1(MS1.begin()+pos_1, MS1.begin()+pos_2);
  std::vector<int> croppedMS2(MS2.begin()+pos_1, MS2.begin()+pos_2);

  auto iter_1 = croppedMS1.begin();
  auto iter_2 = MS2.begin()+pos_1; 

  while(iter_1 < croppedMS1.end()){
    *iter_2 = *iter_1;
    iter_1++;
    iter_2++;
  }

  iter_1 = croppedMS2.begin();
  iter_2 = MS1.begin()+pos_1; 

  while(iter_1 < croppedMS2.end()){
    *iter_2 = *iter_1;
    iter_1++;
    iter_2++;
  }
}

void crossoverTS(std::vector<int>& TS1, std::vector<int>& TS2) { // two-point crossover
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  srand((time_t)ts.tv_nsec);
  
  int pos_1 = rand() % (TS1.size()-1);
  int pos_2 = rand() % (TS1.size()-1);

  if(pos_1 == pos_2) return;

  if(pos_1 > pos_2) {
    int temp = pos_2;
    pos_2 = pos_1;
    pos_1 = temp;
  }

  std::vector<int> croppedTS1(TS1.begin()+pos_1, TS1.begin()+pos_2);
  std::vector<int> croppedTS2(TS2.begin()+pos_1, TS2.begin()+pos_2);

  auto iter_1 = croppedTS1.begin();
  auto iter_2 = TS2.begin()+pos_1; 

  while(iter_1 < croppedTS1.end()){
    *iter_2 = *iter_1;
    iter_1++;
    iter_2++;
  }

  iter_1 = croppedTS2.begin();
  iter_2 = TS1.begin()+pos_1; 

  while(iter_1 < croppedTS2.end()){
    *iter_2 = *iter_1;
    iter_1++;
    iter_2++;
  }
}



void GeneticAlgorithm::crossover(EncodedSolutionSet& pop) const{
  int pop_size = pop.size();
  auto pop_copy(pop);

  pop.clear();
  int i=0;

  while(pop.size() < pop_size) {
    auto it = std::begin(pop_copy);
    auto indiv_1 = it;
    std::advance(it, 1);
    auto indiv_2 = it;


    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    srand((time_t)ts.tv_nsec);
    double rand_val = (double) rand()/RAND_MAX;
    if(rand_val < crossoverProb) {
      crossoverOS((*indiv_1)->OS, (*indiv_2)->OS, problem);
      crossoverMS((*indiv_1)->MS, (*indiv_2)->MS);
      crossoverTS((*indiv_1)->TS, (*indiv_2)->TS);
    }

    pop.insert(*indiv_1);
    pop.insert(*indiv_2);
    pop_copy.erase(*indiv_1);
    pop_copy.erase(*indiv_2);
  }
}


void mutationOS(std::vector<std::string>& OS, const Problem& problem) { // adjacent swap mutation
  int mutation_size = OS.size()/2;

  for(int i = 0; i < mutation_size; i++) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    srand((time_t)ts.tv_nsec);
    
    int pos = rand() % (OS.size()-1);
    int job_num_1, op_num_1, job_num_2, op_num_2;
    getNum(OS[pos], job_num_1, op_num_1);
    getNum(OS[pos+1], job_num_2, op_num_2);

    auto next_ops = problem.jobs.at(job_num_1).operations.at(op_num_1).nextOps;

    if(job_num_1 != job_num_2 || next_ops.find(op_num_2) == next_ops.end()){
      OS[pos] = std::to_string(job_num_2) + "-" + std::to_string(op_num_2);
      OS[pos+1] = std::to_string(job_num_1) + "-" + std::to_string(op_num_1);
    }
  } 
}

void mutationMS(std::vector<int>& MS, const Problem& problem) { // half mutation
  int mutation_size = MS.size()/2;

  std::unordered_set<int> mutation_positions;

  for(int i = 0; i < mutation_size; i++) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    srand((time_t)ts.tv_nsec);
    
    int pos = rand() % MS.size();
    mutation_positions.insert(pos);
  }

  int i = 0;
  for(int job_num=1; job_num <= problem.jobs.size(); job_num++) {
    for(int op_num=1; op_num <= problem.jobs.at(job_num).operations.size(); op_num++) {
      if(mutation_positions.find(i) != mutation_positions.end()) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        srand((time_t)ts.tv_nsec);

        int num_of_machines = problem.jobs.at(job_num).operations.at(op_num).machineNumToOpDuration.size();
      
        MS[i] = rand() % num_of_machines;
        i++;
      }
    }
  }
}

void mutationTS(std::vector<int>& TS, const Problem& problem) { // half mutation
  int mutation_size = TS.size()/2;

  for(int i = 0; i < mutation_size; i++) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    srand((time_t)ts.tv_nsec);
    
    int pos = rand() % TS.size();

    TS[pos] = rand() % problem.numOfTransporters + 1;
  }
}


void GeneticAlgorithm::mutation(EncodedSolutionSet& pop) const{
  int pop_size = pop.size();
  auto pop_copy(pop);

  pop.clear();

  while(pop.size() < pop_size) {
    auto indiv = pop_copy.begin();

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    srand((time_t)ts.tv_nsec);
    double rand_val = (double) rand()/RAND_MAX;
    if(rand_val < mutationProb) {
      mutationOS((*indiv)->OS, problem);
      mutationMS((*indiv)->MS, problem);
      mutationTS((*indiv)->TS, problem);
    }

    pop.insert(*indiv);
    pop_copy.erase(*indiv);
  }
}