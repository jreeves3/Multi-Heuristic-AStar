# Multi-Heuristic-AStar
Multi-Heuristic A* Algorithms (IMHA and SMHA) for N-queens problem

Heuristic usage and calls to imha and smha are internal to code.

New problem domain can be implemented with new planner and state, but imha should be updated to reflect the possibility of reaching same state with lower cost within a given Open,i queue.

N-Queens Problem: Fill an NxN grid columnwise with N queens such that they do not attack each other (share diagonal, column, or row).

A* setting: give heuristic value to each state (valid placement of some number of queens on the board).
Action takes a state with x queens placed in columns 0 .. x-1, and places a queen in column x.


-n number of queens

-t experiment to run
```bash
> python mhastar.py -n 10 -t 0
```
