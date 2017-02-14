# Nazar Stelmakh
# 13 February 2017
# PS 2 - Eight Puzzle

from search import *
from nn_puzzle import *
import heapq
import math


"""
3. ((1, 0, 8, 5, 2, 4, 7, 3, 6), None)
((1, 2, 8, 5, 0, 4, 7, 3, 6), 'Down')
((1, 2, 8, 5, 4, 0, 7, 3, 6), 'Right')
((1, 2, 0, 5, 4, 8, 7, 3, 6), 'Up')
((1, 0, 2, 5, 4, 8, 7, 3, 6), 'Left')
((1, 4, 2, 5, 0, 8, 7, 3, 6), 'Down')
((1, 4, 2, 0, 5, 8, 7, 3, 6), 'Left')
((1, 4, 2, 7, 5, 8, 0, 3, 6), 'Down')
((1, 4, 2, 7, 5, 8, 3, 0, 6), 'Right')
((1, 4, 2, 7, 5, 8, 3, 6, 0), 'Right')
((1, 4, 2, 7, 5, 0, 3, 6, 8), 'Up')
((1, 4, 2, 7, 0, 5, 3, 6, 8), 'Left')
((1, 4, 2, 0, 7, 5, 3, 6, 8), 'Left')
((1, 4, 2, 3, 7, 5, 0, 6, 8), 'Down')
((1, 4, 2, 3, 7, 5, 6, 0, 8), 'Right')
((1, 4, 2, 3, 0, 5, 6, 7, 8), 'Up')
((1, 0, 2, 3, 4, 5, 6, 7, 8), 'Up')
((0, 1, 2, 3, 4, 5, 6, 7, 8), 'Left')

4. To perform the search on the heap, the 
runtime to perform this operation is O(n).

"""


class AStar(Search):
	def __init__(self, problem):
		Search.__init__(self, problem)
		self.open_list = []
		self.closed_list = []
	
	def search(self, initial_state):
		"""This is an implementation for the
		A* search algorithm that takes in a
		shuffled nxn puzzle and it attempts to
		find the solution. You are guarenteed a
		solution and it does it really efficiently"""

		h = self.heuristic(initial_state) # get heuristic cost to get to the solution from the initial states
		node = (h, h, 0, None, initial_state, None) # node = (f, (h)euristic, depth, parent, state, action)
		self.open_list.append(node) # add node onto a list
		heapq.heapify(self.open_list) # heapify the list of nodes so least heuristic cost is at the top

		while (len(self.open_list) > 0): # while there is something in the open list
			node = heapq.heappop(self.open_list) # assign node to be the top node in the list
			if self.problem.isgoal(node[4]): # if goal/solution is found
				return node # return that solution

			total_actions = self.problem.actions(node[4]) # get all possible actions from node

			for action in total_actions: # for every action
				child_i = self.child_node(node, action) # get child of that action

				if (not self.is_in_list(self.closed_list, child_i)): # if the child has already been visited/looked at
					self.closed_list.append(node) # add node that has already been checked onto the closed list
					heapq.heappush(self.open_list, child_i) # push the child onto the heap

		return None
				
	def is_in_list(self, list, node):
		for i in list:
			if(node[4] == i[4]):
				return True
		return False

	def solution(self, node):

		path = []
		p_node = node
		while (p_node != None):
			pair = (p_node[4], p_node[5])
			path.append(pair)
			p_node = self.parent(p_node)
		path.reverse()
		return path

	def heuristic(self, state):
		h = 0
		n = int(math.sqrt(len(state)))
		for i in range(len(state)):
			row_i = int(i / n)
			col_i = i % n
			row_f = int(state[i] / n)
			col_f = state[i] % n
			h += (abs(row_f - row_i) + abs(col_f - col_i))

		return h

	def child_node(self, node, action):
		result = self.problem.result(node[4], action)
		c_state = result[0]
		c_h = self.heuristic(c_state)
		c_g = self.depth(node) + 1
		c_f =  c_h + c_g
		c_node = (c_f, c_h, c_g, node, c_state, action)
		return c_node

	def parent(self, node):
		return node[3]

	def depth(self, node):
		return node[2]
        
class BFS(Search):
    def __init__(self, problem):
        Search.__init__(self, problem)
        self.open_list = []

    def search(self, initial_state):
    	"""This implementation of BFS takes a given state
    	of a shuffled nxn puzzle and searches for the move
    	involving least amount of moves in order to get 
    	to the goal state (solved puzzle). It is a slow
    	method but is guaranteed to find the solution
    	given enough time"""


    	node = (0, None, initial_state, None) # node = (depth, parent, state, action)

    	self.open_list.append(node) # add node onto the list

    	heapq.heapify(self.open_list) # heapify the list in order to have the smallest depth first in the list

    	node = heapq.heappop(self.open_list) # assign node to be the top node at the top of the heap

    	while(not self.problem.isgoal(node[2])): # while we haven't found the solution
    		total_actions =  self.problem.actions(node[2]) # get all possible actions from node
    		for i in total_actions: # for each action
    			child_i = self.child_node(node, i) # get the child node for this action
    			heapq.heappush(self.open_list, child_i) # push the child onto the heap
    		node = heapq.heappop(self.open_list) # node is reassigned to be the node at the top of the heap

    	return node # return node that is the goal found in while loop

    def solution(self, node):
        path = []
        p_node = node
        while (p_node != None):
        	pair = (p_node[2], p_node[3])
        	path.append(pair)
        	p_node = self.parent(p_node)
        path.reverse()
        print(len(path))
        return path

        raise ValueError("This is unimplemented.")

    def child_node(self, node, action):
        result = self.problem.result(node[2], action)
        c_state = result[0]
        c_node = (node[0] + 1, node, c_state, action) # node = (depth, parent, state, action) 

        return c_node

    def parent(self, node):
        return node[1]

    def depth(self, node):
        return node[0]

if __name__ == "__main__":

    puzzle = NNPuzzle(3)
    initial = puzzle.get_shuffled_state(200)
    solver = AStar(puzzle)
    goal = solver.search(initial)
    print("Initial State", initial)
    solver.solution(goal)
    print("Found goal")
    for sa in solver.solution(goal):
        puzzle.display(sa)