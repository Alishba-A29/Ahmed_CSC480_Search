# planner

import sys
import heapq
from collections import deque

DIRECTIONS = {
    'N': (-1, 0),
    'S': (1, 0),
    'W': (0, -1),
    'E': (0, 1)
}

class Node:
    def __init__(self, state, parent=None, action=None):
        self.state = state  # (robot_pos, frozenset(dirty_cells))
        self.parent = parent
        self.action = action

#reading the world from a file
#returns the grid, robot position, dirty cells, rows, and columns
def read_world(filename):
    with open(filename, 'r') as f:
        lines = f.read().strip().splitlines()

    cols = int(lines[0])
    rows = int(lines[1])
    grid = lines[2:]

    robot_pos = None
    dirty = set()

    for r in range(rows):
        for c in range(cols):
            cell = grid[r][c]
            if cell == '@':
                robot_pos = (r, c)
            elif cell == '*':
                dirty.add((r, c))

    return grid, robot_pos, dirty, rows, cols

def is_valid(pos, grid, rows, cols):
    r, c = pos
    return 0 <= r < rows and 0 <= c < cols and grid[r][c] != '#'

def get_successors(state, grid, rows, cols):
    robot, dirties = state
    successors = []

    for action, (dr, dc) in DIRECTIONS.items():
        nr, nc = robot[0] + dr, robot[1] + dc
        if is_valid((nr, nc), grid, rows, cols):
            successors.append(((nr, nc), dirties, action))

    if robot in dirties:
        new_dirties = set(dirties)
        new_dirties.remove(robot)
        successors.append((robot, frozenset(new_dirties), 'V'))

    return successors
# Depth-First Search (DFS)
def dfs(start_pos, dirty_cells, grid, rows, cols):
    start_state = (start_pos, frozenset(dirty_cells))
    frontier = [Node(start_state)]
    visited = set()
    nodes_generated = 0
    nodes_expanded = 0

    while frontier:
        node = frontier.pop()
        nodes_expanded += 1

        if node.state in visited:
            continue
        visited.add(node.state)

        robot, dirties = node.state
        if not dirties:
            actions = []
            while node.parent:
                actions.append(node.action)
                node = node.parent
            actions.reverse()
            for a in actions:
                print(a)
            print(f"{nodes_generated} nodes generated")
            print(f"{nodes_expanded} nodes expanded")
            return

        for new_robot, new_dirties, action in get_successors(node.state, grid, rows, cols):
            new_state = (new_robot, new_dirties)
            if new_state not in visited:
                nodes_generated += 1
                frontier.append(Node(new_state, node, action))

    print("No solution found.")

# Uniform Cost Search (UCS)
def ucs(start_pos, dirty_cells, grid, rows, cols):
    start_state = (start_pos, frozenset(dirty_cells))
    frontier = []
    heapq.heappush(frontier, (0, id(start_state), Node(start_state)))

    visited = {}
    nodes_generated = 0
    nodes_expanded = 0

    while frontier:
        cost, _, node = heapq.heappop(frontier)
        nodes_expanded += 1

        if node.state in visited and visited[node.state] <= cost:
            continue
        visited[node.state] = cost

        robot, dirties = node.state
        if not dirties:
            actions = []
            while node.parent:
                actions.append(node.action)
                node = node.parent
            actions.reverse()
            for a in actions:
                print(a)
            print(f"{nodes_generated} nodes generated")
            print(f"{nodes_expanded} nodes expanded")
            return

        for new_robot, new_dirties, action in get_successors(node.state, grid, rows, cols):
            new_state = (new_robot, new_dirties)
            new_node = Node(new_state, node, action)
            heapq.heappush(frontier, (cost + 1, id(new_node), new_node))
            nodes_generated += 1

    print("No solution found.")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 planner.py [uniform-cost|depth-first] [world-file]")
        sys.exit(1)

    algorithm = sys.argv[1]
    world_file = sys.argv[2]

    grid, robot_pos, dirty, rows, cols = read_world(world_file)
    if algorithm == 'depth-first':
        dfs(robot_pos, dirty, grid, rows, cols)
    elif algorithm == 'uniform-cost':
        ucs(robot_pos, dirty, grid, rows, cols)
    else:
        print("Unsupported algorithm.")