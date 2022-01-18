# Genetic Algorithm Scheduler for Job-Shop Scheduling with Operation Dependencies and Transport Constraints

## Installation steps
```
git clone https://github.com/HanifAryadi/ga-scheduler.git
cd ga-scheduler
mkdir build && cd build
cmake ..
make 
```

## Run scheduling program 
```
./scheduling_alg -i ../problem/1.fjs
```
The scheduling result can be seen in 'output.txt' and the computing statistics can be seen in 'stat.txt'

## Plot the Gantt chart
```
python3 ../src/plot_gantt.py output.txt
```
The plotted Gantt chart can be seen in 'solution.svg'