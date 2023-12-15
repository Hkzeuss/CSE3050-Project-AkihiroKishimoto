# CSE3050 Artificial Intelligence Project by Akihiro Kishimoto

In this project, you will work in a team of 4-5 members to implement several search algorithms that calculate optimal solution costs for the Traveling Salesperson Problem (TSP), a well-known NP-hard problem, and conduct performance comparisons.

## The TSP Problem
In TSP, given N cities, starting with the first city, a salesperson must visit each city exactly once and return to the first city. The salesperson must travel with the smallest sum of edge costs on the route taken by them. [Wikipedia](https://en.wikipedia.org/wiki/Travelling_salesman_problem)

## Project Description
Implement the following algorithms and compare their performance:
1. IDA* with heuristic function h(n)=0
2. IDA* with the min-out heuristic function
3. A* with h(n)=0 (Dijkstra’s algorithm)
4. A* with the min-out heuristic function

You can use any programming language. The program specifications are as follows:
- For N cities, let the id of each city be 0, 1, 2, …, and N-1. Assume the salesperson starts with id=0 and comes back to the state with id=0.
- Let i and j be the ids of two cities (i and j are different). Cities i and j are always connected, and cost(i, j) is an integer satisfying 1<=cost(i, j)<=100.
- Assume that the search is currently at node n with city i. The min-out heuristic h(n) is defined as h(n)=min(cost(i, j1), cost(i, j2), …., cost(i, jk)) where j1, j2, …, jk are the ids of the cities which can be visited from node n (i.e., ignore the cities already visited).

## Experiments
- Measure experiments for the number of cities N=5, 10, 11, and 12. For each N, prepare five problems for performance evaluation.
- Initialize cost(i, j) in the range [1, 100] using a pseudo-random generator.
- Set seeds to 1, 2, 3, 4, and 5 for consistency.

## Performance Metrics
Prepare a table in your project report summarizing the performance of each algorithm for each N. Include the following information:
- Number of solved problems
- Average run time
- Average optimal path cost
- Average number of expanded nodes
- Average number of generated nodes

## Implementation Details
- Time limit for each problem: 20 minutes.
- Real time or CPU time is acceptable for time measurement.
- See [Python Time Library](https://docs.python.org/3/library/time.html#time.time) or [C++ Time Reference](https://en.cppreference.com/w/cpp/chrono) for time measurement functions.
- Use a simple state representation with a class called `State`.

## A* Implementation Notes
- For A*, use a priority queue. Libraries like [Python heapq](https://docs.python.org/3/library/heapq.html) or [C++ priority_queue](https://en.cppreference.com/w/cpp/container/priority_queue) can be utilized.
- Implement the structure of `REACHED` for A*.

## Bonus (Advanced Topic)
Implement an enhanced version of IDA* to solve TSP with h(n)=0 and the min-out heuristic using a hash table to alleviate the reexpansion issue. Compare its performance with the other algorithms.

## Submission
- Due date: 11:59 PM December 22nd.
- Submit one zipped file containing source code and a PDF file of performance comparison tables.

## Important Notes
- All team members will receive the same marks.
- Submit even if the project is incomplete.
- Start early due to the complexity of the algorithms.
- Consult with the instructor if difficulties arise.
