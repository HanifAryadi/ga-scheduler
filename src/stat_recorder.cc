#include <fstream>

#include "stat_recorder.hh"

void StatRecorder::startRecord() {
  startTime = std::chrono::steady_clock::now();
}

void StatRecorder::addStat(int makespan) {
  std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
  int time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - startTime).count();

  stats.push_back(Stat{time, makespan});
}

void StatRecorder::printToFile(const std::string file_name) const {
  std::ofstream output_file;
  output_file.open(file_name);
  
  output_file << "time[ms] makespan[s]\n"; 

  for(auto& stat: stats) {
    output_file << stat.time << " " << stat.makespan <<std::endl;
  }

  output_file.close();

}

