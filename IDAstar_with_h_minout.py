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

def ida_star_min_out(cost_matrix, start, goal):
    def heuristic_min_out(node, visited_cities):
        remaining_cities = [j for j in range(len(cost_matrix)) if j not in visited_cities]
        if not remaining_cities:
            return 0
        return min(cost_matrix[node][j] for j in remaining_cities)

    def dfs(current, g, bound, path, visited_cities):
        nonlocal expanded_nodes, generated_nodes

        expanded_nodes += 1

        f = g + heuristic_min_out(current, visited_cities)
        if f > bound:
            return f, path
        if current == goal:
            return "FOUND", path + [current]

        min_cost = float('inf')
        next_path = None

        for neighbor in range(len(cost_matrix[current])):
            if cost_matrix[current][neighbor] > 0 and neighbor not in visited_cities:
                new_visited_cities = visited_cities + [neighbor]
                generated_nodes += 1
                result, new_path = dfs(neighbor, g + cost_matrix[current][neighbor], bound, path + [neighbor], new_visited_cities)
                if result == "FOUND":
                    return "FOUND", new_path
                if result < min_cost:
                    min_cost = result
                    next_path = new_path

        return min_cost, next_path

    bound = heuristic_min_out(start, [])
    path = [start]
    start_time = time.time()
    expanded_nodes = 0
    generated_nodes = 1  # Start node is generated
    while True:
        result, path = dfs(start, 0, bound, path, [])
        end_time = time.time()
        if result == "FOUND":
            return path, bound, end_time - start_time, expanded_nodes, generated_nodes
        if result == float('inf'):
            return "No path found", None, end_time - start_time, expanded_nodes, generated_nodes
        bound = result

# Example usage:
seeds = [1, 2, 3, 4, 5]
num_cities_values = [5, 10, 11, 12]

total_run_time = 0
total_shortest_path_cost = 0
total_expanded_nodes = 0
total_generated_nodes = 0
total_problems_solved = 0

for num_cities in num_cities_values:
    for seed in seeds:
        cost_matrix = generate_cost_matrix(num_cities, seed)

        print(f"\nCost Matrix (Seed {seed}, Num Cities {num_cities}):")
        for row in cost_matrix:
            print(row)

        start_node = 0  # Assuming the starting node is 0
        goal_node = num_cities - 1  # Assuming the goal node is the last node

        print("\nRunning IDA* with min-out heuristic...")
        path, cost, run_time, expanded_nodes, generated_nodes = ida_star_min_out(cost_matrix, start_node, goal_node)

        print("\nOptimal Path:")
        for node in path:
            print(f"({node // num_cities}, {node % num_cities}) - Cost: {cost_matrix[node // num_cities][node % num_cities]}")
        print(f"Shortest Path Cost: {cost}")
        print(f"Run Time: {run_time} seconds")
        print(f"Expanded Nodes: {expanded_nodes}")
        print(f"Generated Nodes: {generated_nodes}\n")

        total_run_time += run_time
        total_shortest_path_cost += cost
        if cost != float('inf'):
            total_problems_solved += 1
        total_expanded_nodes += expanded_nodes
        total_generated_nodes += generated_nodes

print(f"\nTotal Problems Solved: {total_problems_solved}")
print(f"Average Run Time: {total_run_time / (len(seeds) * len(num_cities_values))} seconds")
print(f"Average Shortest Path Cost: {total_shortest_path_cost / total_problems_solved if total_problems_solved > 0 else 0}")
print(f"Average Expanded Nodes: {total_expanded_nodes / (len(seeds) * len(num_cities_values))}")
print(f"Average Generated Nodes: {total_generated_nodes / (len(seeds) * len(num_cities_values))}")
