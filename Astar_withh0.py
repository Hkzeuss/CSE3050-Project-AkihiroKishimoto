import math
import random
import heapq
import time

def generate_cost_matrix(num_cities, seed):
    random.seed(seed)
    cost_matrix = [[0] * num_cities for _ in range(num_cities)]

    for i in range(num_cities):
        for j in range(num_cities):
            if i != j:
                cost = random.randint(1, 100)
                cost_matrix[i][j] = cost

    return cost_matrix

def astar(cost_matrix, start, goal):
    num_rows, num_cols = len(cost_matrix), len(cost_matrix[0])

    def heuristic(node):
        # Manhattan distance heuristic (assuming movement is only allowed in four directions)
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    # Priority queue to store nodes with their total cost
    priority_queue = [(0, start)]

    # Dictionary to store the best-known cost from start to each node
    best_known_cost = {start: 0}

    # Dictionary to store the parent node for each node
    parents = {start: None}

    start_time = time.time()

    while priority_queue:
        current_cost, current_node = heapq.heappop(priority_queue)

        # Check if we reached the goal
        if current_node == goal:
            end_time = time.time()
            solve_time = end_time - start_time

            # Reconstruct the path from the goal to the start
            path = []
            current = goal
            while current is not None:
                path.insert(0, current)
                current = parents[current]

            return solve_time, best_known_cost[goal], path

        # Explore neighbors
        neighbors = [
            (current_node[0] + 1, current_node[1]),
            (current_node[0] - 1, current_node[1]),
            (current_node[0], current_node[1] + 1),
            (current_node[0], current_node[1] - 1)
        ]

        for neighbor in neighbors:
            row, col = neighbor
            if 0 <= row < num_rows and 0 <= col < num_cols:
                total_cost = best_known_cost[current_node] + cost_matrix[row][col]

                # Update best known cost if a shorter path is found
                if neighbor not in best_known_cost or total_cost < best_known_cost[neighbor]:
                    best_known_cost[neighbor] = total_cost
                    priority = total_cost + heuristic(neighbor)
                    heapq.heappush(priority_queue, (priority, neighbor))
                    parents[neighbor] = current_node

    return float('inf'), float('inf'), []  # Return infinity if no path is found

# Example usage:
seeds = [1, 2, 3, 4, 5]
num_cities_values = [5, 10, 11, 12]

total_solve_time = 0

for num_cities in num_cities_values:
    for seed in seeds:
        cost_matrix = generate_cost_matrix(num_cities, seed)

        print(f"\nCost Matrix (Seed {seed}, Num Cities {num_cities}):")
        for row in cost_matrix:
            print(row)

        start_node = (0, 0)
        goal_node = (num_cities - 1, num_cities - 1)

        start_time = time.time()
        solve_time, total_cost, path = astar(cost_matrix, start_node, goal_node)
        end_time = time.time()

        total_solve_time += solve_time

        print("\nOptimal Path:")
        if path:
            for node in path:
                print(f"({node[0]}, {node[1]}) - Cost: {cost_matrix[node[0]][node[1]]}")
        else:
            print("No path found.")

        print(f"\nTotal Cost: {'Infinity' if math.isinf(total_cost) else total_cost}")
        print(f"Solve Time: {solve_time} seconds")

print(f"\nAverage Solve Time: {total_solve_time / (len(seeds) * len(num_cities_values))} seconds\n")
