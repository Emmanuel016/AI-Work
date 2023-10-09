import heapq

# Define the graph and heuristic values
graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'B': 2, 'C': 2},
    'B': {'C': 3},
    'C': {'D': 4, 'G': 4},
    'D': {'G': 1}
}

heuristic = {
    'S': 7,
    'A': 5,
    'B': 7,
    'C': 4,
    'D': 1,
    'G': 0
}

# Depth-First Search (Graph Search)
def depth_first_search_graph(graph, start, goal):
    stack = [(start, [])]
    visited = set()

    while stack:
        node, path = stack.pop()
        if node == goal:
            path.append(node)  # Include the goal state 'G' in the path
            return "Depth-First Search (Graph Search):", path
        if node not in visited:
            visited.add(node)
            neighbors = [neighbor for neighbor in graph[node].keys()]
            stack.extend((neighbor, path + [node]) for neighbor in reversed(neighbors) if neighbor not in visited)

    # If the goal is not found, you can return an appropriate message
    return "Depth-First Search (Graph Search): Goal not found"

# Breadth-First Search (Graph Search)
def breadth_first_search_graph(graph, start, goal):
    queue = [(start, [])]
    visited = set()

    while queue:
        node, path = queue.pop(0)
        if node == goal:
            path.append(node)  # Include the goal state 'G' in the path
            return "Breadth-First Search (Graph Search):", path
        if node not in visited:
            visited.add(node)
            neighbors = [neighbor for neighbor in graph[node].keys()]
            queue.extend((neighbor, path + [node]) for neighbor in neighbors if neighbor not in visited)

    # If the goal is not found, you can return an appropriate message
    return "Breadth-First Search (Graph Search): Goal not found"

# Uniform Cost Search (Graph Search)
def uniform_cost_search_graph(graph, start, goal):
    priority_queue = [(0, start, [])]
    visited = set()

    while priority_queue:
        cost, node, path = heapq.heappop(priority_queue)
        if node == goal:
            path.append(node)  # Include the goal state 'G' in the path
            return "Uniform Cost Search (Graph Search):", path
        if node not in visited:
            visited.add(node)
            neighbors = [(neighbor, cost + graph[node][neighbor]) for neighbor in graph[node].keys()]
            for neighbor, new_cost in neighbors:
                heapq.heappush(priority_queue, (new_cost, neighbor, path + [node]))

    # If the goal is not found, you can return an appropriate message
    return "Uniform Cost Search (Graph Search): Goal not found"

# Greedy Search (Graph Search)
def greedy_search_graph(graph, start, goal):
    priority_queue = [(heuristic[start], start, [])]
    visited = set()

    while priority_queue:
        _, node, path = heapq.heappop(priority_queue)
        if node == goal:
            path.append(node)  # Include the goal state 'G' in the path
            return "Greedy Search (Graph Search):", path
        if node not in visited:
            visited.add(node)
            neighbors = [neighbor for neighbor in graph[node].keys()]
            priority_queue.extend((heuristic[neighbor], neighbor, path + [node]) for neighbor in neighbors if neighbor not in visited)

    # If the goal is not found, you can return an appropriate message
    return "Greedy Search (Graph Search): Goal not found"

# A* Search (Graph Search)
def a_star_search_graph(graph, start, goal):
    priority_queue = [(heuristic[start], 0, start, [])]
    visited = set()

    while priority_queue:
        _, cost_so_far, node, path = heapq.heappop(priority_queue)
        if node == goal:
            path.append(node)  # Include the goal state 'G' in the path
            return "A* Search (Graph Search):", path
        if node not in visited:
            visited.add(node)
            neighbors = [(neighbor, cost_so_far + graph[node][neighbor]) for neighbor in graph[node].keys()]
            for neighbor, new_cost in neighbors:
                heapq.heappush(priority_queue, (new_cost + heuristic[neighbor], new_cost, neighbor, path + [node]))

    # If the goal is not found, you can return an appropriate message
    return "A* Search (Graph Search): Goal not found"

# Test the search algorithms for Graph Search
goal_state = 'G'
print(*depth_first_search_graph(graph, 'S', goal_state), "\n")
print(*breadth_first_search_graph(graph, 'S', goal_state), "\n")
print(*uniform_cost_search_graph(graph, 'S', goal_state), "\n")
print(*greedy_search_graph(graph, 'S', goal_state), "\n")
print(*a_star_search_graph(graph, 'S', goal_state), "\n")
