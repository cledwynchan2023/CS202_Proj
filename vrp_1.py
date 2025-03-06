import sys
import itertools

"""
To use this file with example testcases, run: 

python vrp.py < 1.in > 1.out

This reads input from 1.in and prints output to 1.out. 
"""

def read_input():
    """Reads input from stdin and returns number of locations, vehicle capacity, distance matrix, and demand vector."""
    n = int(sys.stdin.readline().strip())
    Q = int(sys.stdin.readline().strip())
    
    D = []
    for _ in range(n):
        D.append(list(map(int, sys.stdin.readline().strip().split())))
    
    q = list(map(int, sys.stdin.readline().strip().split()))
    
    return n, Q, D, q

def solve_cvrp(n, Q, D, q):
    """TODO: Solve the Capacitated Vehicle Routing Problem and return a list of routes."""
    # n: Number of stops
    # Q: Vehicle capacity
    # D: Distance matrix 
    # q: The number of passengers in the each station
    current_capacity = 0
    routes = [[0]]
    route_idx = 0
    idx = 1
    while idx < n:
        if current_capacity + q[idx] <= Q:
            routes[route_idx].append(idx)
            current_capacity += q[idx]
            idx += 1
        else:
            current_capacity = 0
            routes[route_idx].append(0)
            routes.append([0])
            route_idx += 1
            
    routes[route_idx].append(0)
    return routes


def check(routes, n, Q, D, q):
    node_visited = []
    for route in routes:
        total_demand = sum([q[i] for i in route if i != 0])
        if not (total_demand <= Q):
            return False
        node_visited += route

    if len(set(node_visited)) != len(q):
        return False

    return True

def main():
    n, Q, D, q = read_input()
    routes = solve_cvrp(n, Q, D, q)

    if check(routes, n, Q, D, q): 
        for route in routes:
            print(" ".join(map(str, route)))

if __name__ == "__main__":
    main()
