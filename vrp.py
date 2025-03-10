import sys
import math
import random
import copy
import time
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
    Generates a neighbor solution using a simple "relocate" move:
      - Randomly select a route with at least one customer.
      - Randomly remove one customer (not the depot) from that route.
      - Attempt to reinsert that customer in a randomly chosen route (which can be the same)
        at a random feasible position.
    Returns a new solution (deep copy) that is feasible.
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
    
    # If insertion wasn't possible in any route, put the customer back to original position.
    if not insertion_done:
        new_solution[r1].insert(pos, customer)
    
    return new_solution

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

def clarke_wright_savings(n, Q, D_full, q):
    """Enhanced Clarke-Wright Savings heuristic for initial solution"""
    savings = []
    for i in range(1, n):
        for j in range(i+1, n):
            savings.append((D_full[0][i] + D_full[0][j] - D_full[i][j], i, j))
    
    savings.sort(reverse=True)
    routes = {i: [0, i, 0] for i in range(1, n)}
    route_demands = {i: q[i] for i in range(1, n)}
    active = set(routes.keys())

    for _, i, j in savings:
        if i in active and j in active:
            if route_demands[i] + route_demands[j] <= Q:
                routes[i] = routes[i][:-1] + routes[j][1:]
                route_demands[i] += route_demands[j]
                del routes[j]
                active.discard(j)
    
    return [route for route in routes.values() if len(route) > 2]

def advanced_neighbor(solution, Q, q, D_full):
    """Enhanced neighbor generator with multiple move types"""
    new_sol = copy.deepcopy(solution)
    move_type = random.choice(["relocate", "swap", "2-opt"])
    
    if move_type == "relocate":
        # Existing relocate logic
        return neighbor(new_sol, Q, q)
    
    elif move_type == "swap":
        # Swap two customers between routes
        routes_with_customers = [i for i, r in enumerate(new_sol) if len(r) > 2]
        if len(routes_with_customers) < 2:
            return new_sol
        
        r1, r2 = random.sample(routes_with_customers, 2)
        i = random.randint(1, len(new_sol[r1])-2)
        j = random.randint(1, len(new_sol[r2])-2)
        
        if (sum(q[c] for c in new_sol[r1] if c != 0) - q[new_sol[r1][i]] + q[new_sol[r2][j]] <= Q and
            sum(q[c] for c in new_sol[r2] if c != 0) - q[new_sol[r2][j]] + q[new_sol[r1][i]] <= Q):
            new_sol[r1][i], new_sol[r2][j] = new_sol[r2][j], new_sol[r1][i]
        
        return new_sol
    
    elif move_type == "2-opt":
        # Intra-route optimization
        route_idx = random.randint(0, len(new_sol)-1)
        route = new_sol[route_idx]
        if len(route) < 4:
            return new_sol
        
        i = random.randint(1, len(route)-3)
        j = random.randint(i+1, len(route)-2)
        new_route = route[:i] + route[i:j+1][::-1] + route[j+1:]
        new_sol[route_idx] = new_route
        return new_sol

def adaptive_simulated_annealing(initial_solution, D_full, Q, q):
    """Improved SA with adaptive cooling and candidate list"""
    current = copy.deepcopy(initial_solution)
    best = copy.deepcopy(current)
    current_cost = total_distance(current, D_full)
    best_cost = current_cost
    
    # Adaptive parameters
    temp = current_cost * 0.1  # Initial temperature based on problem scale
    cooling = 0.995
    max_iter = 25000
    restart_interval = 2000
    no_improve_threshold = 1000
    
    last_improvement = 0
    for iter in range(max_iter):
        # Adaptive restart
        if iter % restart_interval == 0:
            current = copy.deepcopy(best)
            current_cost = best_cost
            temp = best_cost * 0.2
        
        # Generate neighbor using candidate list
        candidate = advanced_neighbor(current, Q, q, D_full)
        candidate_cost = total_distance(candidate, D_full)
        delta = candidate_cost - current_cost
        
        # Adaptive acceptance
        if delta < 0 or math.exp(-delta/temp) > random.random():
            current = candidate
            current_cost = candidate_cost
            if candidate_cost < best_cost:
                best = copy.deepcopy(candidate)
                best_cost = candidate_cost
                last_improvement = iter
        
        # Adaptive cooling
        if iter - last_improvement > no_improve_threshold:
            cooling = 0.999
        else:
            cooling = 0.995
        
        temp *= cooling
        
    return best

def post_optimization(solution, D_full, Q, q):
    """2-opt optimization on all routes"""
    optimized = []
    for route in solution:
        optimized_route = two_opt_optimize(route, D_full)
        optimized.append(optimized_route)
    return optimized

def two_opt_optimize(route, D_full):
    """Full 2-opt implementation with first improvement"""
    best = route
    best_cost = sum(D_full[best[i]][best[i+1]] for i in range(len(best)-1))
    improved = True
    
    while improved:
        improved = False
        for i in range(1, len(route)-2):
            for j in range(i+1, len(route)-1):
                new_route = route[:i] + route[i:j+1][::-1] + route[j+1:]
                new_cost = sum(D_full[new_route[k]][new_route[k+1]] for k in range(len(new_route)-1))
                if new_cost < best_cost:
                    best = new_route
                    best_cost = new_cost
                    improved = True
                    break
            if improved:
                break
    return best

def main():
    n, Q, D_full, q = read_input()
    
    # Improved initial solution
    initial_solution = clarke_wright_savings(n, Q, D_full, q)
    
    # Enhanced SA optimization
    start_time = time.time()
    improved_solution = adaptive_simulated_annealing(initial_solution, D_full, Q, q)
    
    # Post-optimization
    final_solution = post_optimization(improved_solution, D_full, Q, q)
    
    if check(final_solution, n, Q, D_full, q):
        for route in final_solution:
            print(" ".join(map(str, route)))
    else:
        print("Invalid solution", file=sys.stderr)

# Keep existing helper functions (total_distance, check, etc)
if __name__ == "__main__":
    main()