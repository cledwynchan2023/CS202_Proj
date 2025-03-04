import sys

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
    Solves the Capacitated Vehicle Routing Problem using a greedy approach.
    Returns a list of routes where each route starts and ends at the depot.
    
    Updated approach:
      - Start at the depot (node 0).
      - From the current position, find the nearest unvisited customer 
        whose demand does not exceed the remaining capacity.
      - If no such customer exists, end the current route (return to depot)
        and start a new route.
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
            # Find the nearest feasible customer from the current position
            for customer in unvisited:
                if current_load + q[customer] <= Q:
                    dist = D[current_position][customer]
                    if dist < best_distance:
                        best_distance = dist
                        best_candidate = customer
            # If no feasible customer exists, end this route
            if best_candidate is None:
                break
            # Otherwise, add the customer to the route and update state
            current_route.append(best_candidate)
            current_load += q[best_candidate]
            unvisited.remove(best_candidate)
            current_position = best_candidate
        
        current_route.append(0)  # End at the depot
        routes.append(current_route)
    
    return routes

def check(routes, n, Q, D, q):
    """
    Validates the solution to ensure all constraints are satisfied:
      - Each route must start and end at the depot.
      - The total demand of each route does not exceed vehicle capacity.
      - Each customer (nodes 1 to n-1) is visited exactly once.
    """
    visited = set()
    for route in routes:
        # Check capacity constraint (ignoring depot 0)
        total_demand = sum(q[i] for i in route if i != 0)
        if total_demand > Q:
            return False
        
        # Check that the route starts and ends at the depot and has at least one customer
        if route[0] != 0 or route[-1] != 0 or len(route) < 3:
            return False
        
        # Add only customer nodes (exclude depot)
        visited.update(i for i in route if i != 0)
    
    # Ensure all customers are visited exactly once
    if visited != set(range(1, n)):
        return False
    
    return True

def main():
    n, Q, D, q = read_input()
    routes = solve_cvrp_greedy(n, Q, D, q)
    
    if check(routes, n, Q, D, q):
        for route in routes:
            print(" ".join(map(str, route)))
    else:
        print("Invalid solution", file=sys.stderr)

if __name__ == "__main__":
    main()