import sys

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
    Solves the CVRP using a greedy approach with the lower-triangular matrix L.
    For each route, starting from the depot (node 0), the algorithm selects the nearest unvisited
    customer (whose demand fits in the remaining capacity) using vectorized distance lookups.
    """
    routes = []
    unvisited = set(range(1, n))  # Customers to visit (excluding depot)
    
    while unvisited:
        current_route = [0]  # Start at depot
        current_load = 0
        current_position = 0
        
        while True:
            best_candidate = None
            best_distance = float('inf')
            # Iterate over unvisited customers and find the nearest one that fits the capacity.
            for customer in unvisited:
                if current_load + q[customer] <= Q:
                    dist = get_distance(current_position, customer, L)
                    if dist < best_distance:
                        best_distance = dist
                        best_candidate = customer
            if best_candidate is None:
                break  # No feasible customer; finish the current route.
            # Update route and capacity.
            current_route.append(best_candidate)
            current_load += q[best_candidate]
            unvisited.remove(best_candidate)
            current_position = best_candidate
        
        current_route.append(0)  # Return to depot.
        routes.append(current_route)
    
    return routes

def check(routes, n, Q, D_full, q):
    """
    Validates that:
      - Each route starts and ends at the depot (node 0).
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
    n, Q, D_full, q = read_input()
    # Convert full distance matrix to a lower-triangular matrix.
    L = convert_to_lower_triangular(D_full)
    routes = solve_cvrp_greedy(n, Q, L, q)
    
    if check(routes, n, Q, D_full, q):
        for route in routes:
            print(" ".join(map(str, route)))
    else:
        print("Invalid solution", file=sys.stderr)

if __name__ == "__main__":
    main()