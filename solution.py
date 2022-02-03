#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
import heapq
from tokenize import Name
from search import *  # for search engines
from sokoban import SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems
import copy

def sokoban_goal_state(state):
    '''
    @return: Whether all boxes are stored.
    '''
    pass #CHANGE THIS!

    # check if there is a box in every storagepoint
    # loop over each storage point and check if a box is there. If not, return False
    # if there is a box in every storage point, return True
    for box in state.boxes:
        if box not in state.storage:
            return False
    return True

def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.
    
    # loop over each box and calculate the manhattan distance to the nearest storage point
    # add all distances together and return the sum

    total_distance = 0

    for box in state.boxes:
        min_distance = math.inf # this var stores the min manhattan dist of current box to storage point
        for storage in state.storage: # calculate the manhattan distance to each storage point
            distance = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
            if distance < min_distance:
                min_distance = distance # set the new min distance
        total_distance += min_distance # add the manhattan distance to nearest storage point to total distance

    return total_distance



# SOKOBAN HEURISTICS
def trivial_heuristic(state):
    '''trivial admissible sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
    return 0  # CHANGE THIS


def check_if_corner_with_box(state, box):
    # check if a box is in a corner

    combined_obstacles = state.obstacles | state.boxes # obstacles that are either boxes or walls

    # box is blocked from either above or below
    blocked_above_below = box[1] == 0 or (box[0], box[1] - 1) in combined_obstacles or box[1] == state.height - 1 or (box[0], box[1] + 1) in combined_obstacles
    
    # box is blocked from either left or right
    blocked_left_right = box[0] == 0 or (box[0] - 1, box[1]) in combined_obstacles or box[0] == state.width - 1 or (box[0] + 1, box[1]) in combined_obstacles

    if blocked_above_below and blocked_left_right and box not in state.storage:
        return True

    return False


def check_if_edge(state, box, storage):
    # check if box is against a wall and no storage is along the same wall

    x_box_coord = box[0]
    y_box_coord = box[1]

    x_storage_coord = storage[0]
    y_storage_coord = storage[1]

    puzzle_width = state.width - 1
    puzzle_height = state.height - 1

    # Check if box is either at the leftmost or rightmost wall, and check if storage is not along that wall
    if (x_box_coord == 0 and (x_storage_coord != 0 or x_storage_coord == puzzle_width)):
        return True

    elif (x_box_coord == puzzle_width and (x_storage_coord == 0 or x_storage_coord != puzzle_width)):
        return True


    elif (y_box_coord == 0 and (y_storage_coord == 0 or y_storage_coord != puzzle_height)):
        return True


    elif (y_box_coord == puzzle_height and (y_storage_coord == 0 or y_storage_coord != puzzle_height)):
        return True

    return False

def heur_alternate(state):

    # initialize total distance heuristic
    final_heuristic = 0

    boxes_not_in_storage = set() # set of boxes that are not in storage
    for box in state.boxes:
        if box not in state.storage:
            boxes_not_in_storage.add(box)


    for box in boxes_not_in_storage: # only loop over boxes that aren't in storage
        if check_if_corner_with_box(state, box): # check if box is in a corner
            return math.inf
        # TODO: PRIORITIZE STORAGE POINTS THAT ARE IN CORNERS OR CLOSER TO THE OUTSIDE
        min_distance = math.inf # this var stores the min manhattan dist of current box to storage point
        for storage in state.storage:
            if storage in state.boxes: # skip storage points that are occupied by a box
                continue

            if check_if_edge(state, box, storage): # check if box is on a border that no storage points are on
                return math.inf # return infinity if this is the case

            distance = abs(box[0] - storage[0]) + abs(box[1] - storage[1]) # the manhattan distance

            if distance < min_distance:
                min_distance = distance # set the new min distance

        final_heuristic += min_distance # add the manhattan distance to nearest storage point to total distance
        
        min_robot_distance = math.inf # the closest robot to the current box
        for robot in state.robots: # find the closest robot's manhattan dist to the current box
            robot_dist = abs(box[0] - robot[0]) + abs(box[1] - robot[1])
            if robot_dist < min_robot_distance:
                min_robot_distance = robot_dist
            

        # all objects around the current box
        objects_around = set(((box[0] - 1, box[1] - 1), (box[0], box[1] - 1), (box[0] + 1, box[1] - 1),
                            (box[0] - 1, box[1]), (box[0] + 1, box[1]),
                            (box[0] - 1, box[1] + 1), (box[0], box[1] + 1), (box[0] + 1, box[1] + 1),  
                             ))
                            

        obstacles_around = state.obstacles|boxes_not_in_storage # all obstacles around the current box
        final_heuristic += len(obstacles_around&objects_around) # add number of obstacles or other boxes around the current box
        final_heuristic += 2*min_robot_distance # robot distance weighted double


    return final_heuristic

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return sN.gval + weight * sN.hval


# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a warehouse state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''

    # while the frontier is not empty and we are within the timebound
    # instantiate a searchengine with custom search strategy
    engine = SearchEngine('custom', 'full')
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    engine.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
    return engine.search(timebound) # search for a solution with the given timebound and return it with stats



def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a warehouse state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of realtime astar algorithm'''
    curr_weight = weight
    weight_multiplier = 0.1  # decrease the weight by this amount per iteration

    # perform first astar search with the given weight
    engine = SearchEngine('custom', 'full')
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    engine.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)


    curr_time = os.times()[0] + 1 - 1
    time_remaining = timebound
    curr_best_path, curr_best_path_stats = engine.search(timebound) # search for a solution with the given timebound and return it with stats
    time_remaining -= (os.times()[0] - curr_time)

    while time_remaining > 0:
        # perform another a-star with lower weight
        engine = SearchEngine('custom', 'full')
        curr_weight *= weight_multiplier
        wrapped_fval_function = (lambda sN: fval_function(sN, curr_weight))
        engine.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)

        
        # TODO: question: prune for g/h/f or only f?
        new_best_path, new_best_path_stats = engine.search(time_remaining, (math.inf, math.inf, curr_best_path.gval))
        if new_best_path:
            curr_best_path = new_best_path
            curr_best_path_stats = new_best_path_stats
        time_remaining -= (os.times()[0] - curr_time)
    return curr_best_path, curr_best_path_stats



def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''

    # while the frontier is not empty and we are within the timebound
    # instantiate a searchengine with greedy best first strategy
    engine = SearchEngine('best_first', 'full')
    engine.init_search(initial_state, sokoban_goal_state, heur_fn)


    curr_time = os.times()[0]
    time_remaining = timebound
    curr_best_path, curr_best_path_stats = engine.search(timebound) # search for a solution with the given timebound and return it with stats
    time_remaining -= (os.times()[0] - curr_time)

    
    while time_remaining > 0:
        # perform another a-star with lower weight
        engine = SearchEngine('best_first', 'full')
        engine.init_search(initial_state, sokoban_goal_state, heur_fn)

        # TODO: question: prune for g only?
        new_best_path, new_best_path_stats = engine.search(time_remaining, (curr_best_path.gval, math.inf, math.inf))
        if new_best_path:
            curr_best_path = new_best_path
            curr_best_path_stats = new_best_path_stats
        time_remaining -= (os.times()[0] - curr_time)
    return curr_best_path, curr_best_path_stats



