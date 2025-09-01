# -*- coding: utf-8 -*-
"""
This code is a modified version of the A* pathfinding algorithm from
https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2

The modification restricts movement to only "forward" and "turn right".
"""

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None, direction=None):
        self.parent = parent
        self.position = position
        self.direction = direction # The direction we took to get to this node

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position and self.direction == other.direction
    
    def __hash__(self):
        return hash((self.position, self.direction))


def astar(maze, start, end):
    """
    Returns a list of tuples as a path from the given start to the given end in the given maze.
    It automatically determines the best initial direction.
    """

    # Create start and end node
    # The start node has no parent and NO initial direction.
    start_node = Node(None, start, None)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end, None)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Define the directions: Up, Right, Down, Left
    UP = (-1, 0)
    RIGHT = (0, 1)
    DOWN = (1, 0)
    LEFT = (0, -1)
    
    DIRECTIONS = [UP, RIGHT, DOWN, LEFT]

    # This dictionary helps us calculate a "turn right" move.
    turn_right_map = {
        UP: RIGHT,
        RIGHT: DOWN,
        DOWN: LEFT,
        LEFT: UP
    }

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node.position == end_node.position:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children (the next possible moves)
        children = []
        possible_moves = []
        
        # ** THE KEY CHANGE IS HERE **
        # If the node has no direction, it's the start node.
        # From the start, we can move in ANY valid cardinal direction.
        if current_node.direction is None:
            for move_dir in DIRECTIONS:
                possible_moves.append(move_dir)
        # Otherwise, it's a normal node, and we can only go forward or turn right.
        else:
            current_direction = current_node.direction
            forward_direction = current_direction
            turn_right_direction = turn_right_map[current_direction]
            possible_moves = [forward_direction, turn_right_direction]
        
        for move_direction in possible_moves:
            # Get node position
            node_position = (current_node.position[0] + move_direction[0], current_node.position[1] + move_direction[1])

            # Make sure within range
            if not (0 <= node_position[0] < len(maze) and 0 <= node_position[1] < len(maze[0])):
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node with the new position and the new direction
            new_node = Node(current_node, node_position, move_direction)
            children.append(new_node)

        # Loop through children
        for child in children:
            if child in closed_list:
                continue

            child.g = current_node.g + 1
            child.h = abs(child.position[0] - end_node.position[0]) + abs(child.position[1] - end_node.position[1])
            child.f = child.g + child.h

            is_in_open_list = False
            for open_node in open_list:
                if child == open_node and child.g >= open_node.g:
                    is_in_open_list = True
                    break
            
            if is_in_open_list:
                continue

            open_list.append(child)
            
    return None # Return None if no path is found


def main():

    maze1 = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 1, 0, 1, 0 ,1, 1, 1, 0, 1, 1, 1, 0],
             [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 1, 0, 1, 0 ,1, 1, 1, 1, 1, 0, 1, 0],
             [0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0],
             [0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
             [1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0],
             [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
             [1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0],
             [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1]]

    start1 = (11, 10)
    end1 = (11, 2)
    
    # We no longer need to provide a starting direction.
    path = astar(maze1, start1, end1)
    
    if path:
        print("Path found:")
        print(path)
    else:
        print("No path found!")


if __name__ == '__main__':
    main()