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
    Generates a neighbor solution by applying one of two moves:
    
    1. Relocate Move: If there exists a route with >1 customer (length > 3),
       remove a randomly chosen customer from such a route and try to insert it
       into a (possibly different) route at a random feasible position.
       If the removal makes a route empty ([0,0]), remove that route.
       
    2. Swap Move: Otherwise, pick two different routes (if possible) and swap one customer
       from each route (excluding depots) if capacity constraints allow.
       
    The move is only accepted if the resulting solution remains feasible.
    """
    new_solution = copy.deepcopy(solution)
    
    # Identify routes that have more than one customer (i.e. at least two customers).
    relocate_candidates = [i for i, route in enumerate(new_solution) if len(route) > 3]
    
    # Decide move type: prefer relocate if available; otherwise try swap.
    if relocate_candidates:
        move_type = "relocate"
    else:
        if len(new_solution) < 2:
            return new_solution
        move_type = "swap"
    
    if move_type == "relocate":
        # Pick a route with more than one customer.
        r1 = random.choice(relocate_candidates)
        route1 = new_solution[r1]
        # Remove a random customer from route1 (cannot be depot at index 0 or last index)
        pos = random.randint(1, len(route1) - 2)
        customer = route1.pop(pos)
        # If route1 becomes empty (i.e. [0,0]), remove it.
        if len(route1) < 3:
            del new_solution[r1]
        
        insertion_done = False
        route_indices = list(range(len(new_solution)))
        random.shuffle(route_indices)
        for r2 in route_indices:
            route2 = new_solution[r2]
            current_capacity = sum(q[i] for i in route2 if i != 0)
            if current_capacity + q[customer] <= Q:
                possible_positions = list(range(1, len(route2)))
                random.shuffle(possible_positions)
                for pos2 in possible_positions:
                    # Insert the customer and check if the move is feasible.
                    new_route = route2[:pos2] + [customer] + route2[pos2:]
                    # We don't need to fully re-check feasibility here since capacity is ensured.
                    new_solution[r2] = new_route
                    insertion_done = True
                    break
            if insertion_done:
                break
        if not insertion_done:
            # If insertion into any route fails, put the customer back into its original route.
            if r1 < len(new_solution):
                new_solution[r1].insert(pos, customer)
            else:
                new_solution.append([0, customer, 0])
    elif move_type == "swap":
        if len(new_solution) < 2:
            return new_solution
        r1, r2 = random.sample(range(len(new_solution)), 2)
        route1 = new_solution[r1]
        route2 = new_solution[r2]
        if len(route1) < 3 or len(route2) < 3:
            return new_solution
        pos1 = random.randint(1, len(route1) - 2)
        pos2 = random.randint(1, len(route2) - 2)
        customer1 = route1[pos1]
        customer2 = route2[pos2]
        demand_route1 = sum(q[i] for i in route1 if i != 0)
        demand_route2 = sum(q[i] for i in route2 if i != 0)
        if (demand_route1 - q[customer1] + q[customer2] <= Q) and (demand_route2 - q[customer2] + q[customer1] <= Q):
            route1[pos1], route2[pos2] = customer2, customer1
            new_solution[r1] = route1
            new_solution[r2] = route2
    return new_solution

def simulated_annealing(initial_solution, D_full, Q, q, max_iterations=10000, initial_temp=1000, cooling_rate=0.995):
    """
    Improves the initial CVRP solution using Simulated Annealing.
    At each iteration, a neighbor solution is generated.
    The move is accepted if it improves the total distance or with a probability
    based on the temperature. Temperature is decreased over iterations.
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
    Validates that:
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
    if visited != set(range(1, n)):
        return False
    return True

def main():
    n, Q, D_full, q = read_input()
    # Convert the full distance matrix to a lower-triangular matrix.
    L = convert_to_lower_triangular(D_full)
    # Build an initial solution using the greedy algorithm.
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