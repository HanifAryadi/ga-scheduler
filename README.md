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

## References
- Zhang, Guohui & Gao, Liang & Shi, Yang. (2011). An effective genetic algorithm for the flexible job-shop scheduling problem. Expert Syst. Appl.. 38. 3563-3573. 10.1016/j.eswa.2010.08.145. 
- Li, Xinyu & Gao, Liang. (2016). An Effective Hybrid Genetic Algorithm and Tabu Search for Flexible Job Shop Scheduling Problem. International Journal of Production Economics. 174. 10.1016/j.ijpe.2016.01.016. 
- Al Aqel, Ghiath & Li, Xinyu & Gao, Liang. (2019). A Modified Iterated Greedy Algorithm for Flexible Job Shop Scheduling Problem. Chinese Journal of Mechanical Engineering. 32. 10.1186/s10033-019-0337-7
- Zhang, Guohui & Hu, Yifan & Sun, Jinghe & Zhang, Wenqiang. (2020). An improved genetic algorithm for the flexible job shop scheduling problem with multiple time constraints. Swarm and Evolutionary Computation. 54. 100664. 10.1016/j.swevo.2020.100664. 
- Qihao, Liu & Li, Xinyu & Gao, Liang & Li, Yingli. (2020). A Modified Genetic Algorithm With New Encoding and Decoding Methods for Integrated Process Planning and Scheduling Problem. IEEE Transactions on Cybernetics. PP. 1-10. 10.1109/TCYB.2020.3026651. 