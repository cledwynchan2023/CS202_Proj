import sys
import random
import math
import copy

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

def solve_cvrp_greedy(n, Q, D, q):
    """
    Generates an initial solution using a greedy approach.
    Each route is constructed by repeatedly choosing the nearest feasible customer.
    """
    routes = []
    unvisited = set(range(1, n))  # Customers to visit (excluding depot)
    
    while unvisited:
        current_route = [0]  # Start at the depot
        current_load = 0
        current_position = 0
        
        while True:
            best_candidate = None
            best_distance = float('inf')
            # Look for the nearest customer that fits the capacity constraint.
            for customer in unvisited:
                if current_load + q[customer] <= Q:
                    dist = D[current_position][customer]
                    if dist < best_distance:
                        best_distance = dist
                        best_candidate = customer
            if best_candidate is None:
                break  # No further feasible customer; finish the route.
            current_route.append(best_candidate)
            current_load += q[best_candidate]
            unvisited.remove(best_candidate)
            current_position = best_candidate
        
        current_route.append(0)  # End at the depot
        routes.append(current_route)
    
    return routes

def total_distance(routes, D):
    """
    Computes the total distance of all routes.
    """
    total = 0
    for route in routes:
        for i in range(len(route)-1):
            total += D[route[i]][route[i+1]]
    return total

def check(routes, n, Q, D, q):
    """
    Validates the solution:
      - Each route starts and ends at depot.
      - Total demand on each route does not exceed Q.
      - Every customer (nodes 1 to n-1) is visited exactly once.
    """
    visited = set()
    for route in routes:
        total_demand = sum(q[i] for i in route if i != 0)
        if total_demand > Q:
            return False
        if route[0] != 0 or route[-1] != 0 or len(route) < 3:
            return False
        visited.update(i for i in route if i != 0)
    
    if visited != set(range(1, n)):
        return False
    
    return True

def neighbor(solution, q, Q):
    """
    Generates a neighbor solution by performing a relocate move.
    A random customer (not depot) is removed from one route and reinserted into another route
    (or used to form a new route) if the capacity constraint allows.
    """
    new_solution = copy.deepcopy(solution)
    
    # Choose a route that has at least one customer (route length > 2 since depot at start and end).
    valid_routes = [i for i, route in enumerate(new_solution) if len(route) > 2]
    if not valid_routes:
        return new_solution
    
    route_index = random.choice(valid_routes)
    route = new_solution[route_index]
    
    # Randomly select a customer from this route (avoid depot at positions 0 and last)
    customer_index = random.randint(1, len(route) - 2)
    customer = route.pop(customer_index)
    
    # If route becomes empty (only depot remains), remove the route.
    if len(route) == 2:
        del new_solution[route_index]
    
    inserted = False
    attempts = 0
    # Try to insert the customer into an existing route.
    while not inserted and attempts < 10:
        attempts += 1
        if new_solution and random.random() < 0.8:
            target_route_index = random.randrange(len(new_solution))
            target_route = new_solution[target_route_index]
            # Check capacity constraint.
            current_demand = sum(q[i] for i in target_route if i != 0)
            if current_demand + q[customer] > Q:
                continue
            pos = random.randint(1, len(target_route)-1)
            target_route.insert(pos, customer)
            inserted = True
        else:
            # Create a new route if insertion into an existing route isn't feasible.
            if q[customer] <= Q:
                new_solution.append([0, customer, 0])
                inserted = True
    # If insertion failed after several attempts, revert the move.
    if not inserted:
        if route_index < len(new_solution):
            new_solution[route_index].insert(customer_index, customer)
        else:
            new_solution.append([0, customer, 0])
    return new_solution

def simulated_annealing(initial_solution, D, q, Q, max_iterations=10000, initial_temp=1000, cooling_rate=0.995):
    """
    Improves an initial CVRP solution using simulated annealing.
    """
    current_solution = initial_solution
    current_cost = total_distance(current_solution, D)
    best_solution = copy.deepcopy(current_solution)
    best_cost = current_cost
    temp = initial_temp
    
    for iteration in range(max_iterations):
        neighbor_solution = neighbor(current_solution, q, Q)
        if not neighbor_solution:
            continue
        
        # Check feasibility; if not feasible, skip this neighbor.
        if not check(neighbor_solution, len(D), Q, D, q):
            continue
        
        neighbor_cost = total_distance(neighbor_solution, D)
        delta = neighbor_cost - current_cost
        
        # Accept better solutions or worse solutions probabilistically.
        if delta < 0 or random.random() < math.exp(-delta / temp):
            current_solution = neighbor_solution
            current_cost = neighbor_cost
            if current_cost < best_cost:
                best_solution = copy.deepcopy(current_solution)
                best_cost = current_cost
        
        temp *= cooling_rate
        if temp < 1e-3:
            break
    return best_solution

def main():
    n, Q, D, q = read_input()
    
    # Generate an initial solution using the greedy approach.
    initial_solution = solve_cvrp_greedy(n, Q, D, q)
    # Uncomment the following line to check the greedy solution:
    # print("Greedy Total Distance:", total_distance(initial_solution, D), file=sys.stderr)
    
    # Improve the solution using simulated annealing.
    improved_solution = simulated_annealing(initial_solution, D, q, Q)
    
    # Final feasibility check.
    if check(improved_solution, n, Q, D, q):
        for route in improved_solution:
            print(" ".join(map(str, route)))
        # Optionally, print the total distance for debugging:
        # print("Total Distance:", total_distance(improved_solution, D), file=sys.stderr)
    else:
        print("Invalid solution", file=sys.stderr)

if __name__ == "__main__":
    main()