# -*- coding: utf-8 -*-
"""
This code is a modified version of the A* pathfinding algorithm from
https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2

The modification restricts movement to only "moving forward" and "turning right"
relative to the current direction of travel.
"""

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None, direction=None):
        self.parent = parent
        self.position = position
        self.direction = direction # Stores the direction (e.g., UP, DOWN) used to arrive at this node.

        self.g = 0 # Cost from start to current node
        self.h = 0 # Heuristic cost from current node to end
        self.f = 0 # Total cost (g + h)

    def __eq__(self, other):
        # Two nodes are considered equal if they share the same position AND direction of arrival.
        # This is key for treating, for example, (10,10) arrived from 'down' as a different
        # state from (10,10) arrived from 'right'.
        return self.position == other.position and self.direction == other.direction
    
    def __hash__(self):
        # We need to define a hash function because we changed the __eq__ method.
        # This allows Node objects to be stored correctly in data structures like sets.
        return hash((self.position, self.direction))

def astar(maze, start, end):
    """
    Finds a path from a start to an end point in a maze using a modified A* algorithm.
    The algorithm is constrained to only move "forward" or "turn right" relative to the current direction.
    """
    
    # The start node has no parent and its direction is 'None'
    # This is a special case handled in the main loop to allow it to move in any valid cardinal direction initially
    start_node = Node(None, start, None)
    start_node.g = start_node.h = start_node.f = 0
    
    # The end node is only used for its position, so its direction can be None
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

    # Main Search Loop
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
        # We only need to check the position, not the direction of arrival at the end node.
        if current_node.position == end_node.position:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate Next Possible Moves (Children)        
        children = []
        possible_moves = []
        
        # Check if this is the start node (which has no initial direction).
        if current_node.direction is None:
            # If it is, all four cardinal directions are possible first moves.
            possible_moves = DIRECTIONS
        else:
            # If it's not the start node, determine the "forward" and "turn right" directions.
            current_direction = current_node.direction
            forward_direction = current_direction # "Forward" is just the direction we used to get here.
            turn_right_direction = turn_right_map[current_direction] # "Turn right" is calculated from our map.
            possible_moves = [forward_direction, turn_right_direction]
        
        for move_direction in possible_moves:
            # Get the new position based on the move
            node_position = (current_node.position[0] + move_direction[0], current_node.position[1] + move_direction[1])

            # Make sure the new position is within the maze boundaries
            if not (0 <= node_position[0] < len(maze) and 0 <= node_position[1] < len(maze[0])):
                continue

            # Make sure the new position is a walkable path (0) and not a wall (1)
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create a new node for this valid move, storing the direction of travel.
            new_node = Node(current_node, node_position, move_direction)
            children.append(new_node)

        # --- Process Children ---

        for child in children:
            # If the child's state (position + direction) is already in the closed list, skip it.
            if child in closed_list:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            # Heuristic: Manhattan distance. We use this instead of Euclidean distance because
            # it's a more accurate estimate for a grid where we can't move diagonally.
            child.h = abs(child.position[0] - end_node.position[0]) + abs(child.position[1] - end_node.position[1])
            child.f = child.g + child.h

            # Check if this child node's state is already in the open list.
            # If it is, and our new path to it isn't better (i.e., has an equal or higher g cost), we skip it.
            is_in_open_list = False
            for open_node in open_list:
                if child == open_node and child.g >= open_node.g:
                    is_in_open_list = True
                    break
            
            if is_in_open_list:
                continue

            # Otherwise, add the child to the open list for evaluation.
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
    
    path = astar(maze1, start1, end1)
    
    if path:
        print("Path found:")
        print(path)
    else:
        print("No path found!")

if __name__ == '__main__':
    main()