import sys

def read_input():
    """
    Reads input from stdin and returns:
      n: total number of locations (including depot),
      Q: vehicle capacity,
      D: distance matrix,
      q: demand vector (of length n).
    """
    n = int(sys.stdin.readline().strip())
    Q = int(sys.stdin.readline().strip())
    
    D = []
    for _ in range(n):
        row = list(map(int, sys.stdin.readline().strip().split()))
        D.append(row)
    
    # Expecting the demand vector to have n numbers.
    q = list(map(int, sys.stdin.readline().strip().split()))
    # If the demand vector is shorter, assume depot demand (index 0) is 0.
    if len(q) < n:
        q = [0] + q
    return n, Q, D, q

def merge_routes(route_i, route_j, i, j):
    """
    Attempts to merge two routes that contain customers i and j.
    The goal is to connect the route containing i with the route containing j.
    We allow route reversal if necessary.
    Returns the merged route if a valid merge is found, else None.
    
    Each route is assumed to start and end with depot (0).
    """
    # Four possible cases:
    # Case 1: i is at the end of route_i and j is at the beginning of route_j.
    if route_i[-2] == i and route_j[1] == j:
        return route_i[:-1] + route_j[1:]
    # Case 2: i is at the beginning of route_i and j is at the end of route_j.
    if route_i[1] == i and route_j[-2] == j:
        return route_j[:-1] + route_i[1:]
    # Case 3: i is at the beginning of route_i and j is at the beginning of route_j.
    # Reverse route_i so that i becomes the end.
    if route_i[1] == i and route_j[1] == j:
        rev_i = list(reversed(route_i))
        # Now rev_i[-2] == i
        return rev_i[:-1] + route_j[1:]
    # Case 4: i is at the end of route_i and j is at the end of route_j.
    # Reverse route_j so that j becomes the beginning.
    if route_i[-2] == i and route_j[-2] == j:
        rev_j = list(reversed(route_j))
        # Now rev_j[1] == j
        return route_i[:-1] + rev_j[1:]
    return None

def solve_cvrp_clarke_wright(n, Q, D, q):
    """
    Solves the CVRP using an enhanced Clarke & Wright Savings heuristic.
    Returns a list of routes (each route is a list of nodes starting and ending with depot 0).
    """
    # Initialize: each customer (node 1 to n-1) gets its own route.
    # (Assume depot is node 0.)
    routes = []
    route_demand = []
    route_of = [None] * n  # route_of[i] = index of the route that customer i belongs to.
    
    for i in range(1, n):
        routes.append([0, i, 0])
        route_demand.append(q[i])
        route_of[i] = len(routes) - 1

    # Compute savings for every pair of distinct customers (i, j) with i < j.
    savings_list = []
    for i in range(1, n):
        for j in range(i+1, n):
            # Savings: distance from depot to i plus depot to j minus distance from i to j.
            saving = D[0][i] + D[0][j] - D[i][j]
            savings_list.append((saving, i, j))
    
    # Process savings in descending order.
    savings_list.sort(key=lambda x: x[0], reverse=True)
    
    for saving, i, j in savings_list:
        route_i_idx = route_of[i]
        route_j_idx = route_of[j]
        # If either customer has been merged already (route set to None) or they are in same route, skip.
        if route_i_idx is None or route_j_idx is None or route_i_idx == route_j_idx:
            continue
        r_i = routes[route_i_idx]
        r_j = routes[route_j_idx]
        # Attempt to merge the routes using the flexible merge function.
        merged = merge_routes(r_i, r_j, i, j)
        if merged is None:
            continue
        # Check capacity feasibility.
        if route_demand[route_i_idx] + route_demand[route_j_idx] > Q:
            continue
        # Merge is feasible; update the merged route.
        new_demand = route_demand[route_i_idx] + route_demand[route_j_idx]
        
        # Choose new route index as route_i_idx.
        routes[route_i_idx] = merged
        route_demand[route_i_idx] = new_demand
        
        # Mark route_j_idx as merged.
        routes[route_j_idx] = None
        route_demand[route_j_idx] = 0
        
        # Update route_of for all customers in the merged route.
        for customer in merged:
            if customer != 0:
                route_of[customer] = route_i_idx
    
    # Filter out merged (None) routes.
    final_routes = [r for r in routes if r is not None]
    return final_routes

def solve_cvrp_greedy(n, Q, D, q):
    """
    Original greedy approach (nearest feasible customer).
    """
    routes = []
    unvisited = set(range(1, n))
    
    while unvisited:
        current_route = [0]
        current_load = 0
        current_position = 0
        while True:
            best_candidate = None
            best_distance = float('inf')
            for customer in unvisited:
                if current_load + q[customer] <= Q:
                    dist = D[current_position][customer]
                    if dist < best_distance:
                        best_distance = dist
                        best_candidate = customer
            if best_candidate is None:
                break
            current_route.append(best_candidate)
            current_load += q[best_candidate]
            unvisited.remove(best_candidate)
            current_position = best_candidate
        current_route.append(0)
        routes.append(current_route)
    
    return routes

def check(routes, n, Q, D, q):
    """
    Validates that:
      - Each route starts and ends at depot (node 0).
      - The total demand on each route does not exceed Q.
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

def main():
    n, Q, D, q = read_input()
    # Switch between approaches by uncommenting:
    # routes = solve_cvrp_greedy(n, Q, D, q)
    routes = solve_cvrp_clarke_wright(n, Q, D, q)
    
    if check(routes, n, Q, D, q):
        for route in routes:
            print(" ".join(map(str, route)))
    else:
        print("Invalid solution", file=sys.stderr)

if __name__ == "__main__":
    main()