#include "rosmsg.hh"

Problem loadProblemFromMsg(const ga_scheduler::Problem& msg) {
  std::unordered_map<JobNum, Problem::Job> jobs;
  for(auto& job_msg: msg.jobs) {
    int job_num = job_msg.job_num;

    std::unordered_map<OpNum, Problem::Operation> operations;
    for(auto& op_msg: job_msg.operations) {
      assert(job_num == op_msg.job_num);
      int op_num = op_msg.operation_num;

      std::set<OpNum> prev_ops;
      for(auto& op: op_msg.prev_ops) {
        prev_ops.insert(op);
      }

      std::set<OpNum> next_ops;
      for(auto& op: op_msg.next_ops) {
        next_ops.insert(op);
      }

      std::unordered_map<MachineNum, Duration> 
       machine_num_to_op_duration;
      MachineNum machine_num = 1;
      for(auto& dur: op_msg.durations) {
        if(dur != -1) 
          machine_num_to_op_duration.insert(
            std::make_pair(machine_num, dur));
        machine_num++;
      }      

      Problem::Operation operation{job_num, op_num, prev_ops, 
                          next_ops, machine_num_to_op_duration};
      operations.insert(std::make_pair(op_num, operation));
    }

    Problem::Job job{job_num, operations};
    jobs.insert(std::make_pair(job_num, job));
  }

  std::unordered_map<MachineNum, Problem::Machine> machines;
  for(auto& machine_msg: msg.machines) {
    int machine_num = machine_msg.machine_num;

    std::unordered_map<MachineNum, Duration> 
      machine_num_to_trp_duration;
    MachineNum other_machine_num = 1;
    for(auto& dur: machine_msg.transport_durations) {
      machine_num_to_trp_duration.insert(
        std::make_pair(other_machine_num, dur));
      other_machine_num ++;
    }

    Problem::Machine machine{machine_num, machine_num_to_trp_duration};
    machines.insert(std::make_pair(machine_num, machine));
  }

  int num_of_transporters = msg.num_of_transporters;

  Problem problem{jobs, machines, num_of_transporters};

  return problem; 
}

ga_scheduler::Solution printMsgFromSolution(const DecodedSolution& solution) {
  ga_scheduler::Solution msg;

  for(auto& pair: solution.machineSchedule) {
    ga_scheduler::MachineTasks machine_tasks_msg;

    machine_tasks_msg.machine_num = pair.first;   

    for(auto& task: pair.second) {
      ga_scheduler::MachineTask machine_task_msg;

      machine_task_msg.name = task.name;
      machine_task_msg.machine_num = task.machineNum;
      machine_task_msg.start = task.start;
      machine_task_msg.finish = task.finish;
      machine_task_msg.duration = task.duration;
      
      machine_tasks_msg.machine_tasks.push_back(machine_task_msg);
    }

    msg.all_machine_tasks.push_back(machine_tasks_msg);
  }

  for(auto& pair: solution.transporterSchedule) {
    ga_scheduler::TransporterTasks transporter_tasks_msg;

    transporter_tasks_msg.transporter_num = pair.first;   

    for(auto& task: pair.second) {
      ga_scheduler::TransporterTask transporter_task_msg;

      transporter_task_msg.name = task.name;
      transporter_task_msg.transporter_num = task.transporterNum;
      transporter_task_msg.start = task.start;
      transporter_task_msg.finish = task.finish;
      transporter_task_msg.duration = task.duration;

      transporter_task_msg.task_type = (int)task.taskType;

      transporter_task_msg.from_machine_num = task.fromMachineNum;
      transporter_task_msg.to_machine_num = task.toMachineNum;
      
      transporter_tasks_msg.transporter_tasks.push_back(transporter_task_msg);
    }
    
    msg.all_transporter_tasks.push_back(transporter_tasks_msg);
  }

  return msg;  
}