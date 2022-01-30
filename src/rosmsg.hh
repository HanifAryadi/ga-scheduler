#pragma once

#include "problem.hh"
#include "decoded_solution.hh"

#include <ros/ros.h>

#include <ga_scheduler/Problem.h>
#include <ga_scheduler/Solution.h>

Problem loadProblemFromMsg(const ga_scheduler::Problem&);

ga_scheduler::Solution printMsgFromSolution(const DecodedSolution&);