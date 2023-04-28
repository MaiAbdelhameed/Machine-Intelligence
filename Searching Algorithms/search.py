from problem import HeuristicFunction, Problem, S, A, Solution
from collections import deque
from helpers import utils
from queue import PriorityQueue

# TODO: Import any modules you want to use

# All search functions take a problem and a state
# If it is an informed search function, it will also receive a heuristic function
# S and A are used for generic typing where S represents the state type and A represents the action type

# All the search functions should return one of two possible type:
# 1. A list of actions which represent the path from the initial state to the final state
# 2. None if there is no solution


def BreadthFirstSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    # TODO: ADD YOUR CODE HERE
    Paths = [[]]  # list of all the possible paths from the initial state
    frontier = deque()  # a queue of all the nodes ready to be explored; i used a queue so that the first node ready to be explored is actually explored first
    visited = list()  # list of the state that we visited
    if (problem.is_goal(initial_state)):  # check if the initial state is a goal
        return [initial_state]

    # adding the states ready to be explored in the frontier
    frontier.append(initial_state)

    while (frontier):  # to loop of the frontier as long as it has nodes to be explored
        currState = frontier.popleft()  # pop the state to be explored next
        # pop the current path that we are currently using (walking in)
        currPath = Paths.pop(0)

        if (currState not in visited):  # check if the state is visited (explored) before

            # add the state to the visited (explored)
            visited.append(currState)

            # loop over the available actions from the current state
            for action in problem.get_actions(currState):

                # apply the action to the current state to get the next state (neighbour)
                neighbour = problem.get_successor(currState, action)

                # check if the next state (neighbour) is not visited and not in the frontier (not explored and not yet added as ready to be explored)
                if (neighbour not in visited and neighbour not in frontier):

                    # add the current action (child) to the current path
                    path = currPath + [action]
                    # add this path to all the paths we walked in
                    Paths.append(path)

                    if (problem.is_goal(neighbour)):  # check if this next state is the goal
                        return path  # if yes, return the path that lead us to this goal state

                    # add the next state to the frontier to be ready to be explored later on
                    frontier.append(neighbour)

    return


def DepthFirstSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    # TODO: ADD YOUR CODE HERE
    Paths = [[]]  # list of all the possible paths from the initial state
    frontier = list()  # list of all the nodes ready to be explored
    # adding the states ready to be explored in the frontier
    frontier.append(initial_state)
    visited = list()  # list of the state that we visited

    while (frontier):  # to loop of the frontier as long as it has nodes to be explored
        currState = frontier.pop()  # pop the state to be explored next
        currPath = Paths.pop()  # pop the current path that we are currently using (walking in)

        # check if the state is visited (explored) before
        if currState not in visited:
            if (problem.is_goal(currState)):  # check if the current state is a goal
                return currPath  # if yes, return the path that lead us to this goal state

            # add this state to the visited (explored)
            visited.append(currState)

            # loop over the available actions from the current state
            for child in problem.get_actions(currState):

                neighbour = problem.get_successor(currState, child)

                # check if the next state (neighbour) is not visited and not in the frontier (not explored and not yet added as ready to be explored)
                if (neighbour not in visited and neighbour not in frontier):

                    # add the current action (child) to the current path
                    path = currPath + [child]
                    # add this path to all the paths we walked in
                    Paths.append(path)
                    # add the next state to the frontier to be ready to be explored later on
                    frontier.append(neighbour)

    return


def UniformCostSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    # TODO: ADD YOUR CODE HERE

    frontier = PriorityQueue()  # a priority queue of all the nodes ready to be explored; i used a priority queue so that whenever a state is added to the queue, the LEAST cost state always comes first (index 0)
    visited = list()  # list of the state that we visited

    # use an id to each state so that if two states have the same cost, the priority queue orders them according to the id, i.e: the state that arrived first (order of entering the frontier)
    id = 0
    # tuple of (cost, id, current state, path to this current state)
    frontier.put((0, id, initial_state, []))
    # loop over the states inside the frontier as long as the frontier is not empty
    while (not frontier.empty()):
        curr = frontier.get()  # get the state to be explored next (according to cost)
        currNode = curr[2]  # get the current state out of the tuple
        if (currNode not in visited):  # if this state has not yet to be explored
            path = curr[3]  # get the path from the tuple
            cost = curr[0]  # get the cost from the tuple
            # check if this state is goal and not visited
            if (problem.is_goal(currNode) and currNode not in visited):
                return path  # return the path we used to get to this goal state
            visited.append(currNode)  # add the state to the visited (explored)
            # loop over the available actions from the current state
            for action in problem.get_actions(currNode):
                id = id+1  # incement the id
                # apply the action to the current state to get the next state (neighbour)
                neighbour = problem.get_successor(currNode, action)
                # calculate the cumulative cost
                nodeCost = problem.get_cost(currNode, action) + cost
                # add the current action (child) to the current path
                currPath = path + [action]
                # add the next state and its parameters to the frontier
                frontier.put((nodeCost, id, neighbour, currPath))

    return


def AStarSearch(problem: Problem[S, A], initial_state: S, heuristic: HeuristicFunction) -> Solution:
    # priority queue for the state to be ordered by the heuristic + the cumulative cost
    frontier = PriorityQueue()
    visited = list()  # list of the state that we visited

    # use an id to each state so that if two states have the same cost, the priority queue orders them according to the id, i.e: the state that arrived first (order of entering the frontier)
    id = 0
    # tuple of (heuiristic + cost, id, current state, path to this current state)
    frontier.put((heuristic(problem, initial_state), 0, id, initial_state, []))
    # loop over the states inside the frontier as long as the frontier is not empty
    while (not frontier.empty()):
        curr = frontier.get()  # get the state to be explored next (according to cost)
        currNode = curr[3]  # get the current state out of the tuple
        if (currNode not in visited):  # if this state has not yet to be explored
            path = curr[4]  # get the path from the tuple
            cost = curr[1]  # get the cost from the tuple
            # check if this state is goal and not visited
            if (problem.is_goal(currNode) and currNode not in visited):
                return path  # return the path we used to get to this goal state
            visited.append(currNode)  # add the state to the visited (explored)
            # loop over the available actions from the current state
            for action in problem.get_actions(currNode):
                id = id+1  # incement the id
                # apply the action to the current state to get the next state (neighbour)
                neighbour = problem.get_successor(currNode, action)
                # calculate the cumulative cost
                nodeCost = problem.get_cost(currNode, action) + cost
                # add the current action (child) to the current path
                currPath = path + [action]
                # add the next state and its parameters to the frontier
                frontier.put((nodeCost + heuristic(problem, neighbour),
                             nodeCost, id, neighbour, currPath))

    return


def BestFirstSearch(problem: Problem[S, A], initial_state: S, heuristic: HeuristicFunction) -> Solution:
    # TODO: ADD YOUR CODE HERE

    frontier = PriorityQueue()  # a priority queue of all the nodes ready to be explored; i used a priority queue so that whenever a state is added to the queue, the LEAST cost state always comes first (index 0)
    visited = list()  # list of the state that we visited

    # use an id to each state so that if two states have the same heuristic, the priority queue orders them according to the id, i.e: the state that arrived first (order of entering the frontier)
    id = 0
    # tuple of (heuristic, id, current state, path to this current state)
    frontier.put((heuristic(problem, initial_state), id, initial_state, []))
    # loop over the states inside the frontier as long as the frontier is not empty
    while (not frontier.empty()):
        curr = frontier.get()  # get the state to be explored next (according to cost)
        currNode = curr[2]  # get the current state out of the tuple
        if (currNode not in visited):  # if this state has not yet to be explored
            path = curr[3]  # get the path from the tuple
            # check if this state is goal and not visited
            if (problem.is_goal(currNode) and currNode not in visited):
                return path  # return the path we used to get to this goal state
            visited.append(currNode)  # add the state to the visited (explored)
            # loop over the available actions from the current state
            for action in problem.get_actions(currNode):
                id = id+1  # incement the id
                # apply the action to the current state to get the next state (neighbour)
                neighbour = problem.get_successor(currNode, action)
                # add the current action (child) to the current path
                currPath = path + [action]
                # add the next state and its parameters to the frontier
                frontier.put((heuristic(problem, neighbour),
                             id, neighbour, currPath))

    return
