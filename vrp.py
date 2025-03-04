import sys
import numpy as np

def read_input():
    """
    Reads input from stdin and returns the number of locations, vehicle capacity,
    distance matrix, and demand vector.
    """
    n = int(sys.stdin.readline().strip())
    Q = int(sys.stdin.readline().strip())
    
    D = []
    for _ in range(n):
        D.append(list(map(int, sys.stdin.readline().strip().split())))
    
    q = list(map(int, sys.stdin.readline().strip().split()))
    
    return n, Q, D, q

def solve_cvrp_greedy_np(n, Q, D, q):
    """
    Optimized greedy CVRP solver using NumPy for vectorized distance computations.
    
    Approach:
      - Convert the distance matrix and demand vector to NumPy arrays.
      - For each route, while there are unvisited customers, compute the distances
        from the current node to all unvisited candidates at once.
      - Use vectorized operations to filter feasible candidates (i.e., those that do not
        exceed capacity) and select the one with the minimum distance.
      - Update the route and capacity, and repeat until no feasible candidate remains.
    """
    # Convert D and q to NumPy arrays for fast vectorized operations.
    D_np = np.array(D)
    q_np = np.array(q)
    
    routes = []
    unvisited = set(range(1, n))  # Customers to visit (excluding depot)
    
    while unvisited:
        current_route = [0]  # Start at depot (node 0)
        current_load = 0
        current_position = 0
        
        while True:
            if not unvisited:
                break
            # Convert unvisited set to NumPy array of candidate indices.
            candidates = np.array(list(unvisited))
            # Get distances from current_position to all candidates.
            distances = D_np[current_position, candidates]
            # Determine which candidates are feasible with respect to capacity.
            feasible_mask = (current_load + q_np[candidates] <= Q)
            if not np.any(feasible_mask):
                break  # No feasible candidate; end current route.
            # Among feasible candidates, choose the one with minimum distance.
            feasible_candidates = candidates[feasible_mask]
            feasible_distances = distances[feasible_mask]
            best_index = np.argmin(feasible_distances)
            best_candidate = int(feasible_candidates[best_index])
            
            # Update route, capacity, and current position.
            current_route.append(best_candidate)
            current_load += q_np[best_candidate]
            unvisited.remove(best_candidate)
            current_position = best_candidate
        
        current_route.append(0)  # Return to depot.
        routes.append(current_route)
    
    return routes

def check(routes, n, Q, D, q):
    """
    Validates the solution to ensure:
      - Each route starts and ends at the depot (node 0).
      - The total demand on each route does not exceed the vehicle capacity.
      - Each customer (nodes 1 to n-1) is visited exactly once.
    """
    visited = set()
    for route in routes:
        # Calculate total demand for the route (ignore depot 0).
        total_demand = sum(q[i] for i in route if i != 0)
        if total_demand > Q:
            return False
        
        # Check route structure: must start and end at depot and contain at least one customer.
        if route[0] != 0 or route[-1] != 0 or len(route) < 3:
            return False
        
        visited.update(i for i in route if i != 0)
    
    # Ensure every customer (1 to n-1) is visited exactly once.
    if visited != set(range(1, n)):
        return False
    
    return True

def main():
    n, Q, D, q = read_input()
    routes = solve_cvrp_greedy_np(n, Q, D, q)
    
    if check(routes, n, Q, D, q):
        for route in routes:
            print(" ".join(map(str, route)))
    else:
        print("Invalid solution", file=sys.stderr)

if __name__ == "__main__":
    main()