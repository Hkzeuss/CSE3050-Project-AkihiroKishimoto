import sys
import random
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

def ida_star(cost_matrix, start, goal):
    def heuristic(node):
        # h(n) = 0, as specified
        return 0

    def dfs(current, g, bound, path):
        f = g + heuristic(current)
        if f > bound:
            return f, path
        if current == goal:
            return "FOUND", path + [current]

        min_cost = float('inf')
        next_path = None

        for neighbor in range(len(cost_matrix[current])):
            if cost_matrix[current][neighbor] > 0:
                result, new_path = dfs(neighbor, g + cost_matrix[current][neighbor], bound, path + [neighbor])
                if result == "FOUND":
                    return "FOUND", new_path
                if result < min_cost:
                    min_cost = result
                    next_path = new_path

        return min_cost, next_path

    bound = heuristic(start)
    path = [start]
    start_time = time.time()
    while True:
        result, path = dfs(start, 0, bound, path)
        end_time = time.time()
        if result == "FOUND":
            return path, bound, end_time - start_time
        if result == float('inf'):
            return "No path found", None, end_time - start_time
        bound = result

# Example usage:
seeds = [1, 2, 3, 4, 5]
num_cities_values = [5, 10, 11, 12]

for num_cities in num_cities_values:
    total_run_time = 0.0
    num_experiments = 5  # Adjust the number of experiments as needed
    for seed in seeds:
        cost_matrix = generate_cost_matrix(num_cities, seed)

        print(f"\nCost Matrix (Seed {seed}, Num Cities {num_cities}):")
        for row in cost_matrix:
            print(row)

        start_node = 0  # Assuming the starting node is 0
        goal_node = num_cities - 1  # Assuming the goal node is the last node

        print("\nRunning IDA*...")
        for _ in range(num_experiments):
            _, _, run_time = ida_star(cost_matrix, start_node, goal_node)
            total_run_time += run_time

    average_run_time = total_run_time / (num_experiments * len(seeds))
    print(f"\nAverage Run Time for Num Cities {num_cities}: {average_run_time} seconds\n")
