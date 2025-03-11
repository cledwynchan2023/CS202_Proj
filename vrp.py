import sys
import math

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

def route_distance(route, D_full):
    """Computes the total distance of a given route."""
    return sum(D_full[route[i]][route[i+1]] for i in range(len(route)-1))

def two_opt(route, D_full):
    """
    Improves a given route using the 2-opt swap heuristic.
    It repeatedly reverses segments of the route if doing so reduces the total distance.
    """
    best = route
    improved = True
    while improved:
        improved = False
        for i in range(1, len(best) - 2):
            for j in range(i+1, len(best) - 1):
                # Skip adjacent nodes; reversal would have no effect.
                if j - i == 1:
                    continue
                new_route = best[:i] + best[i:j+1][::-1] + best[j+1:]
                if route_distance(new_route, D_full) < route_distance(best, D_full):
                    best = new_route
                    improved = True
        route = best
    return best

def solve_cvrp_clarke_wright_improved(n, Q, D_full, q):
    """
    Solves the CVRP using an improved Clarke–Wright Savings algorithm.
    Improvements include:
      - Allowing route reversal to enable more merging options.
      - Applying a 2-opt local search on each route post merging.
    """
    # Initialize: each customer gets its own route [0, i, 0]
    routes = []
    route_id = {}    # Maps each customer to its current route index.
    route_demand = []  # Holds the total demand for each route.
    
    for i in range(1, n):
        route = [0, i, 0]
        routes.append(route)
        route_id[i] = len(routes) - 1
        route_demand.append(q[i])
    
    # Compute savings for every pair (i, j) with i < j.
    savings = []
    for i in range(1, n):
        for j in range(i+1, n):
            saving = D_full[0][i] + D_full[0][j] - D_full[i][j]
            savings.append((saving, i, j))
    savings.sort(reverse=True, key=lambda x: x[0])
    
    # Process savings list.
    for saving, i, j in savings:
        r_i = route_id.get(i)
        r_j = route_id.get(j)
        # Skip if either customer is no longer available or they are already in the same route.
        if r_i is None or r_j is None or r_i == r_j:
            continue
        
        route_i = routes[r_i]
        route_j = routes[r_j]
        
        # Ensure both customers are at an endpoint of their routes.
        if i not in (route_i[1], route_i[-2]) or j not in (route_j[1], route_j[-2]):
            continue
        
        # If both customers are at the beginning of their routes, reverse route_i.
        if route_i[1] == i and route_j[1] == j:
            route_i = route_i[::-1]
            routes[r_i] = route_i
        
        # If both customers are at the end of their routes, reverse route_j.
        if route_i[-2] == i and route_j[-2] == j:
            route_j = route_j[::-1]
            routes[r_j] = route_j
        
        # Try to merge: Case 1 where i is at the end of route_i and j is at the beginning of route_j.
        if route_i[-2] == i and route_j[1] == j:
            if route_demand[r_i] + route_demand[r_j] <= Q:
                new_route = route_i[:-1] + route_j[1:]
                routes[r_i] = new_route
                route_demand[r_i] += route_demand[r_j]
                for cust in route_j:
                    if cust != 0:
                        route_id[cust] = r_i
                routes[r_j] = []  # Mark route as merged.
                route_demand[r_j] = 0
        # Try to merge: Case 2 where j is at the end of route_j and i is at the beginning of route_i.
        elif route_j[-2] == j and route_i[1] == i:
            if route_demand[r_i] + route_demand[r_j] <= Q:
                new_route = route_j[:-1] + route_i[1:]
                routes[r_j] = new_route
                route_demand[r_j] += route_demand[r_i]
                for cust in route_i:
                    if cust != 0:
                        route_id[cust] = r_j
                routes[r_i] = []  # Mark route as merged.
                route_demand[r_i] = 0
    
    # Remove empty routes (those that have been merged into another).
    merged_routes = [r for r in routes if r]
    
    # Apply 2-opt local search on each route to further reduce the distance.
    improved_routes = []
    for route in merged_routes:
        if len(route) > 3:  # Only optimize routes that include more than one customer.
            improved_route = two_opt(route, D_full)
            improved_routes.append(improved_route)
        else:
            improved_routes.append(route)
    
    return improved_routes

def total_distance(routes, D_full):
    """Computes the total distance of all routes."""
    return sum(route_distance(route, D_full) for route in routes)

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
        if sum(q[i] for i in route if i != 0) > Q:
            return False
        visited.update(cust for cust in route if cust != 0)
    return visited == set(range(1, n))

def main():
    n, Q, D_full, q = read_input()
    routes = solve_cvrp_clarke_wright_improved(n, Q, D_full, q)
    if check(routes, n, Q, D_full, q):
        for route in routes:
            print(" ".join(map(str, route)))
        print(total_distance(routes,D_full))
    else:
        print("Invalid solution", file=sys.stderr)

if __name__ == "__main__":
    main()