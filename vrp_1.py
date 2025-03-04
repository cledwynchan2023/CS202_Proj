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

def find_nearest_unvisited(n, D, current_node, node_visited):
    nearest_unvisited_node = None
    nearest_unvisited_node_distance = 0
    
    for node in range(1, n):
        if node not in node_visited:
            if nearest_unvisited_node == None:
                nearest_unvisited_node = node
                nearest_unvisited_node_distance = D[current_node][node]
            elif D[current_node][node] < nearest_unvisited_node_distance:
                nearest_unvisited_node = node
                nearest_unvisited_node_distance = D[current_node][node]

    return nearest_unvisited_node

def solve_cvrp(n, Q, D, q):
    """TODO: Solve the Capacitated Vehicle Routing Problem and return a list of routes."""
    
    # Greedy Solution: 
    # 1. Find nearest unvisited node
    # 2. Go to that node if will not exceed capacity, else return depot
    # 3. Repeat until all nodes visited
    
    routes = [[]]
    node_visited = []
    
    # Loop for all routes
    while len(node_visited) < (n - 1):
        route = [0]
        held_demand = 0
        current_node = 0
        # Loop for 1 route
        while len(node_visited) < (n - 1):
            # Find nearest unvisited node
            nearest_unvisited_node = find_nearest_unvisited(n, D, current_node, node_visited)
            
            # Check whether capacity allows going to the node
            if held_demand + q[nearest_unvisited_node] <= Q:
                route.append(nearest_unvisited_node)
                node_visited.append(nearest_unvisited_node)
                held_demand += q[nearest_unvisited_node]
                current_node = nearest_unvisited_node
            else:
                break
        route.append(0)
        if routes[0] == []:
            routes[0] = route
        else:
            routes.append(route)
    
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
