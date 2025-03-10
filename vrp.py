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

def solve_cvrp_clarke_wright(n, Q, D_full, q):
    """
    Solves the CVRP using the Clarke–Wright Savings algorithm.
    
    Steps:
      1. Initialize: each customer i (for i=1..n-1) gets its own route [0, i, 0].
      2. Compute savings for each pair of customers (i, j) with i < j:
         S(i, j) = D_full[0][i] + D_full[0][j] - D_full[i][j].
      3. Process savings in descending order. For each pair (i, j):
         - Let route_i be the route containing i and route_j be the route containing j.
         - If route_i ≠ route_j, and if i is at the end of route_i and j is at the beginning of route_j (or vice versa),
           and the total demand of the merged route is ≤ Q, then merge them.
      4. Return the nonempty routes.
    """
    # Initialize each customer in its own route.
    routes = []
    route_id = {}  # Maps each customer to its route index.
    route_demand = []
    
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
        # Skip if either customer has been merged into an empty route or they are in the same route.
        if r_i is None or r_j is None or r_i == r_j:
            continue
        route_i = routes[r_i]
        route_j = routes[r_j]
        # Case 1: if i is the last customer in route_i and j is the first customer in route_j.
        if route_i[-2] == i and route_j[1] == j:
            if route_demand[r_i] + route_demand[r_j] <= Q:
                new_route = route_i[:-1] + route_j[1:]
                routes[r_i] = new_route
                route_demand[r_i] += route_demand[r_j]
                for cust in route_j:
                    if cust != 0:
                        route_id[cust] = r_i
                routes[r_j] = []
                route_demand[r_j] = 0
                route_id[j] = r_i
        # Case 2: if j is the last customer in route_j and i is the first customer in route_i.
        elif route_j[-2] == j and route_i[1] == i:
            if route_demand[r_i] + route_demand[r_j] <= Q:
                new_route = route_j[:-1] + route_i[1:]
                routes[r_j] = new_route
                route_demand[r_j] += route_demand[r_i]
                for cust in route_i:
                    if cust != 0:
                        route_id[cust] = r_j
                routes[r_i] = []
                route_demand[r_i] = 0
                route_id[i] = r_j
    
    # Filter out empty routes.
    final_routes = [r for r in routes if r]
    return final_routes

def total_distance(routes, D_full):
    total = 0
    for route in routes:
        for i in range(len(route) - 1):
            total += D_full[route[i]][route[i+1]]
    return total

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
        visited.update(i for i in route if i != 0)
    return visited == set(range(1, n))

def main():
    n, Q, D_full, q = read_input()
    routes = solve_cvrp_clarke_wright(n, Q, D_full, q)
    if check(routes, n, Q, D_full, q):
        for route in routes:
            print(" ".join(map(str, route)))
    else:
        print("Invalid solution", file=sys.stderr)

if __name__ == "__main__":
    main()