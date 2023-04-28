from typing import Any, Dict, Set, Tuple, List
from problem import Problem
from mathutils import Direction, Point
from helpers import utils

# TODO: (Optional) Instead of Any, you can define a type for the parking state
ParkingState = Tuple[Point]
# An action of the parking problem is a tuple containing an index 'i' and a direction 'd' where car 'i' should move in the direction 'd'.
ParkingAction = Tuple[int, Direction]

# This is the implementation of the parking problem


class ParkingProblem(Problem[ParkingState, ParkingAction]):
    # A set of points which indicate where a car can be (in other words, every position except walls).
    passages: Set[Point]
    # A tuple of points where state[i] is the position of car 'i'.
    cars: Tuple[Point]
    # A dictionary which indicate the index of the parking slot (if it is 'i' then it is the lot of car 'i') for every position.
    slots: Dict[Point, int]
    # if a position does not contain a parking slot, it will not be in this dictionary.
    width: int              # The width of the parking lot.
    height: int             # The height of the parking lot.

    # This function should return the initial state
    def get_initial_state(self) -> ParkingState:
        # return the initial state of the parking, which the state[i] is the position of car i
        return self.cars

    # This function should return True if the given state is a goal. Otherwise, it should return False.

    def is_goal(self, state: ParkingState) -> bool:
        # get all the correct position slots of the cars from the dictionary slots
        slotPos = list(self.slots.keys())
        for i in range(0, len(slotPos)):  # loop of all the slot positions
            # if the position of car i is in the slot position and the position of this car has a value of i in the dictionary slots
            if (state[i] in slotPos and i == self.slots[state[i]]):
                flag = True  # i.e: the car i is parked in "its correct" parking slots
            else:
                flag = False  # false means: either this car is parking in a parking slot of another car or this car position is not a parking slot
                break  # hence, not a goal state
        return flag

    # This function returns a list of all the possible actions that can be applied to the given state

    def get_actions(self, state: ParkingState) -> List[ParkingAction]:
        actions = list()  # a list to get all the possible actions a car can take, i.e: all the positions a car can move in (right/left/up/down)

        # loop of the length of state, which is the number of cars; to get EVERY possible action of EVERY car
        for i in range(0, len(state)):
            x = tuple(state[i])[0]  # x-coordinate of car i
            y = tuple(state[i])[1]  # y-coordinate of car i

            p = Point(x+1, y)  # RIGHT
            # check if car 'i' can move right (check if this movement is possible)
            if (p in self.passages and p not in state):
                # add this possible action to the list of action
                actions.append(tuple((i, 'R')))

            p = Point(x-1, y)  # LEFT
            # check if car 'i' can move left (check if this movement is possible)
            if (p in self.passages and p not in state):
                # add this possible action to the list of action
                actions.append(tuple((i, 'L')))

            p = Point(x, y-1)  # UP
            # check if car 'i' can move up (check if this movement is possible)
            if (p in self.passages and p not in state):
                # add this possible action to the list of action
                actions.append(tuple((i, 'U')))

            p = Point(x, y+1)  # DOWN
            # check if car 'i' can move down (check if this movement is possible)
            if (p in self.passages and p not in state):
                # add this possible action to the list of action
                actions.append(tuple((i, 'D')))

        return actions

    # This function returns a new state which is the result of applying the given action to the given state

    def get_successor(self, state: ParkingState, action: ParkingAction) -> ParkingState:
        newState = list(state)  # change the state into a list to apply changes
        ind, dir = action  # get the index of the car and its movement direction
        # apply the movement direction to the state to get a new position
        newPos = state[ind]+dir.to_vector()
        # add this new position to the state[i], i.e: the position of car i
        newState[ind] = newPos

        newState = tuple(newState)  # change the state into a tuple again
        return newState

    # This function returns the cost of applying the given action to the given state

    def get_cost(self, state: ParkingState, action: ParkingAction) -> float:

        # apply the action to the state to get a new state, using previously implement function
        newState = self.get_successor(state, action)

        index = action[0]  # get the index of the car from the action
        cost = 1  # set the default of the cost to be 1
        # get all the correct position slots of the cars from the dictionary slots
        slotPos = list(self.slots.keys())

        # check if this car is in a wrong parking slot
        if (newState[index] in slotPos and index != self.slots[newState[index]]):
            cost = 101  # i.e: the parking slot of another car, hence apply a cost of 100 + 1

        return cost

    # Read a parking problem from text containing a grid of tiles

    @staticmethod
    def from_text(text: str) -> 'ParkingProblem':
        passages = set()
        cars, slots = {}, {}
        lines = [line for line in (line.strip()
                                   for line in text.splitlines()) if line]
        width, height = max(len(line) for line in lines), len(lines)
        for y, line in enumerate(lines):
            for x, char in enumerate(line):
                if char != "#":
                    passages.add(Point(x, y))
                    if char == '.':
                        pass
                    elif char in "ABCDEFGHIJ":
                        cars[ord(char) - ord('A')] = Point(x, y)
                    elif char in "0123456789":
                        slots[int(char)] = Point(x, y)
        problem = ParkingProblem()
        problem.passages = passages
        problem.cars = tuple(cars[i] for i in range(len(cars)))
        problem.slots = {position: index for index, position in slots.items()}
        problem.width = width
        problem.height = height
        return problem

    # Read a parking problem from file containing a grid of tiles
    @staticmethod
    def from_file(path: str) -> 'ParkingProblem':
        with open(path, 'r') as f:
            return ParkingProblem.from_text(f.read())
