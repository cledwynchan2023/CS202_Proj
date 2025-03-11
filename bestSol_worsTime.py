import sys
import math
import random
import copy

def read_input():
    """
    Reads input from stdin and returns:
      n: total number of locations (including depot),
      Q: vehicle capacity,
      D_full: full symmetric distance matrix,
      q: demand vector.
    """
    n = int(sys.stdin.readline().strip())
    Q = int(sys.stdin.readline().strip())
    
    D_full = []
    for _ in range(n):
        D_full.append(list(map(int, sys.stdin.readline().strip().split())))
    
    q = list(map(int, sys.stdin.readline().strip().split()))
    
    return n, Q, D_full, q

def convert_to_lower_triangular(D_full):
    """
    Given a full symmetric matrix, returns the lower-triangular matrix.
    Each row i in the lower-triangular matrix contains the first (i+1) elements of D_full[i].
    """
    n = len(D_full)
    L = [D_full[i][:i+1] for i in range(n)]
    return L

def get_distance(i, j, L):
    """
    Returns the distance between nodes i and j using the lower triangular matrix L.
    Since the distance matrix is symmetric, if i < j then the distance is stored in L[j][i].
    """
    if i >= j:
        return L[i][j]
    else:
        return L[j][i]

def solve_cvrp_greedy(n, Q, L, q):
    """
    Greedy CVRP solver using the lower-triangular matrix L.
    Constructs routes starting from the depot (node 0) by repeatedly selecting
    the nearest unvisited customer whose demand fits the remaining capacity.
    """
    routes = []
    unvisited = set(range(1, n))  # Customers (exclude depot)
    
    while unvisited:
        current_route = [0]  # Start at depot
        current_load = 0
        current_position = 0
        
        while True:
            best_candidate = None
            best_distance = float('inf')
            for customer in unvisited:
                if current_load + q[customer] <= Q:
                    dist = get_distance(current_position, customer, L)
                    if dist < best_distance:
                        best_distance = dist
                        best_candidate = customer
            if best_candidate is None:
                break  # No feasible candidate; finish this route.
            current_route.append(best_candidate)
            current_load += q[best_candidate]
            unvisited.remove(best_candidate)
            current_position = best_candidate
        
        # Only add route if at least one customer was added.
        if len(current_route) > 1:
            if current_route[-1] != 0:
                current_route.append(0)
            routes.append(current_route)
        else:
            break
    return routes

def route_cost(route, D_full):
    """Returns the total cost/distance of a single route."""
    return sum(D_full[route[i]][route[i+1]] for i in range(len(route)-1))

def total_distance(routes, D_full):
    """Returns the total distance of the entire solution."""
    return sum(route_cost(route, D_full) for route in routes)

def two_opt(route, D_full):
    """
    Applies the 2-opt heuristic to a single route.
    Returns an improved route if an improvement is found; otherwise returns the original route.
    """
    best_route = route[:]
    best_cost = route_cost(best_route, D_full)
    improved = True
    while improved:
        improved = False
        for i in range(1, len(best_route) - 2):
            for j in range(i+1, len(best_route) - 1):
                new_route = best_route[:i] + best_route[i:j+1][::-1] + best_route[j+1:]
                new_cost = route_cost(new_route, D_full)
                if new_cost < best_cost:
                    best_route = new_route
                    best_cost = new_cost
                    improved = True
        route = best_route
    return best_route

def intra_route_improvement(solution, D_full):
    """
    Applies 2-opt improvement on each route in the solution.
    """
    new_solution = []
    for route in solution:
        new_solution.append(two_opt(route, D_full))
    return new_solution

def inter_route_relocate(solution, D_full, Q, q):
    """
    Tries to improve the solution by relocating a customer from one route to another.
    Iterates over each customer (not depot) in each route and tries to reinsert it
    into another route at the best feasible position (if it reduces the overall distance).
    Returns the new solution if an improvement is found; otherwise returns the original solution.
    """
    best_solution = copy.deepcopy(solution)
    best_cost = total_distance(best_solution, D_full)
    for r1 in range(len(solution)):
        for pos in range(1, len(solution[r1]) - 1):
            customer = solution[r1][pos]
            # Remove customer from route r1 temporarily.
            new_route1 = solution[r1][:pos] + solution[r1][pos+1:]
            # Check if route r1 is still valid after removal.
            if len(new_route1) < 3:
                continue
            for r2 in range(len(solution)):
                if r1 == r2:
                    continue
                # Check capacity feasibility:
                if sum(q[i] for i in new_route1 if i != 0) > Q:
                    continue
                if sum(q[i] for i in solution[r2] if i != 0) + q[customer] > Q:
                    continue
                # Try inserting customer in route r2 at every possible position.
                for pos2 in range(1, len(solution[r2])):
                    new_route2 = solution[r2][:pos2] + [customer] + solution[r2][pos2:]
                    # Create a candidate solution.
                    candidate_solution = copy.deepcopy(solution)
                    candidate_solution[r1] = new_route1
                    candidate_solution[r2] = new_route2
                    candidate_cost = total_distance(candidate_solution, D_full)
                    if candidate_cost < best_cost:
                        best_cost = candidate_cost
                        best_solution = candidate_solution
    return best_solution

def local_search(solution, D_full, Q, q, max_iter=100):
    """
    Iteratively applies intra-route 2-opt improvements and inter-route relocate moves
    until no further improvement is found or a maximum number of iterations is reached.
    """
    current_solution = copy.deepcopy(solution)
    current_solution = intra_route_improvement(current_solution, D_full)
    current_cost = total_distance(current_solution, D_full)
    
    for _ in range(max_iter):
        # First try inter-route relocate moves.
        new_solution = inter_route_relocate(current_solution, D_full, Q, q)
        # Then improve each route with 2-opt.
        new_solution = intra_route_improvement(new_solution, D_full)
        new_cost = total_distance(new_solution, D_full)
        if new_cost < current_cost:
            current_solution = new_solution
            current_cost = new_cost
        else:
            break
    return current_solution

def check(routes, n, Q, D_full, q):
    """
    Validates the solution:
      - Each route starts and ends at the depot (node 0).
      - The total demand on each route does not exceed Q.
      - Every customer (nodes 1 to n-1) is visited exactly once.
    """
    visited = set()
    for route in routes:
        if route[0] != 0 or route[-1] != 0 or len(route) < 3:
            return False
        total_demand = sum(q[i] for i in route if i != 0)
        if total_demand > Q:
            return False
        visited.update(i for i in route if i != 0)
    return visited == set(range(1, n))

def main():
    n, Q, D_full, q = read_input()
    # Build an initial solution using the greedy approach.
    L = convert_to_lower_triangular(D_full)
    initial_solution = solve_cvrp_greedy(n, Q, L, q)
    # Improve the solution using a rigorous local search.
    improved_solution = local_search(initial_solution, D_full, Q, q)
    
    if check(improved_solution, n, Q, D_full, q):
        for route in improved_solution:
            print(" ".join(map(str, route)))
    else:
        print("Invalid solution", file=sys.stderr)

if __name__ == "__main__":
    main()