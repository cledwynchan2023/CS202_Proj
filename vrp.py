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
    Each row i in the lower triangular matrix contains the first (i+1) elements of D_full[i].
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
        
        current_route.append(0)  # Return to depot.
        routes.append(current_route)
    
    return routes

def total_distance(routes, D_full):
    """
    Computes the total travel distance of the solution using the full distance matrix.
    """
    total = 0
    for route in routes:
        for i in range(len(route)-1):
            total += D_full[route[i]][route[i+1]]
    return total

def neighbor(solution, Q, q):
    """
    Generates a neighbor solution using a simple "relocate" move.
    Ensures no [0,0] routes are created or maintained.
    """
    new_solution = copy.deepcopy(solution)
    
    # Choose a route that has at least one customer.
    candidate_routes = [i for i, route in enumerate(new_solution) if len(route) > 2]
    if not candidate_routes:
        return new_solution
    
    r1 = random.choice(candidate_routes)
    route1 = new_solution[r1]
    # Remove a random customer from route1 (not the depot at index 0 or last index)
    pos = random.randint(1, len(route1) - 2)
    customer = route1.pop(pos)
    
    # If route becomes [0,0], remove it completely
    if len(route1) == 2:  # Only depot nodes remain
        del new_solution[r1]
    
    # Try to insert the customer into a randomly chosen route.
    insertion_done = False
    route_indices = list(range(len(new_solution)))
    random.shuffle(route_indices)
    
    for r2 in route_indices:
        route2 = new_solution[r2]
        # Try all possible insertion positions (between index 1 and len(route2))
        for pos2 in range(1, len(route2)):
            # Check capacity for route2.
            current_capacity = sum(q[i] for i in route2 if i != 0)
            if current_capacity + q[customer] <= Q:
                # Insert the customer at position pos2.
                new_route = route2[:pos2] + [customer] + route2[pos2:]
                # Temporarily update route2.
                new_solution[r2] = new_route
                insertion_done = True
                break
        if insertion_done:
            break
    
    # If insertion wasn't possible in any route, create a new route
    if not insertion_done:
        new_solution.append([0, customer, 0])
    
    return new_solution

def simulated_annealing(initial_solution, D_full, Q, q, max_iterations=10000, initial_temp=1000, cooling_rate=0.995):
    """
    Improves the initial CVRP solution using Simulated Annealing.
    At each iteration, a neighbor solution is generated via a relocate move.
    The move is accepted if it improves the solution or with a probability
    proportional to exp(-delta/temperature). Temperature is decreased over iterations.
    """
    current_solution = copy.deepcopy(initial_solution)
    best_solution = copy.deepcopy(initial_solution)
    current_cost = total_distance(current_solution, D_full)
    best_cost = current_cost
    temp = initial_temp
    
    for _ in range(max_iterations):
        new_solution = neighbor(current_solution, Q, q)
        new_cost = total_distance(new_solution, D_full)
        delta = new_cost - current_cost
        
        if delta < 0 or random.random() < math.exp(-delta / temp):
            current_solution = new_solution
            current_cost = new_cost
            if current_cost < best_cost:
                best_solution = copy.deepcopy(current_solution)
                best_cost = current_cost
        
        temp *= cooling_rate
        if temp < 1e-3:
            break
    
    return best_solution

def check(routes, n, Q, D_full, q):
    """
    Validates the solution:
      - Each route starts and ends at depot (node 0).
      - Total demand per route does not exceed Q.
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
    if visited != set(range(1, n)):
        return False
    return True

def main():
    n, Q, D_full, q = read_input()
    # Use the lower-triangular matrix for the greedy solution.
    L = convert_to_lower_triangular(D_full)
    initial_solution = solve_cvrp_greedy(n, Q, L, q)
    # Improve the solution using Simulated Annealing.
    improved_solution = simulated_annealing(initial_solution, D_full, Q, q)
    
    if check(improved_solution, n, Q, D_full, q):
        for route in improved_solution:
            print(" ".join(map(str, route)))
    else:
        print("Invalid solution", file=sys.stderr)

if __name__ == "__main__":
    main()