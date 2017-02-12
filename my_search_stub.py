from search import *
from nn_puzzle import *
import heapq
import math

class AStar(Search):
	def __init__(self, problem):
		Search.__init__(self, problem)
		self.open_list = []
		self.closed_list = []
	
	def search(self, initial_state):
		h = self.heuristic(initial_state)
		node = (h, 0, None, initial_state, None) # node = (heuristic, depth, parent, state, action)
		self.open_list.append(node)
		heapq.heapify(self.open_list)
		#node = heapq.heappop(self.open_list)

		while (len(self.open_list) > 0):
			node = heapq.heappop(self.open_list)
			if self.problem.isgoal(node[3]):
				return node

			self.closed_list.append(node)
			total_actions = self.problem.actions(node[3])
			for action in total_actions:
				child_i = self.child_node(node, action)
				if (self.is_in_list(self.closed_list, child_i)):
					continue
				if (not self.is_in_list(self.open_list, child_i)):
					if(not self.is_in_list(self.closed_list, child_i)):
						heapq.heappush(self.open_list, child_i)
				elif ((node[1]) >= child_i[1]):
					continue
				
	def is_in_list(self, list, node):
		for i in list:
			if(node[3] == i[3]):
				return True
		return False

	def solution(self, node):

		path = []
		p_node = node
		while (p_node != None):
			pair = (p_node[3], p_node[4])
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
		result = self.problem.result(node[3], action)
		c_state = result[0]
		c_h = self.heuristic(c_state)
		c_node = (c_h, self.depth(node) + 1, node, c_state, action)
		return c_node

	def parent(self, node):
		return node[2]

	def depth(self, node):
		return node[1]
        
class BFS(Search):
    def __init__(self, problem):
        Search.__init__(self, problem)
        self.open_list = []

    def search(self, initial_state):
        node = (0, None, initial_state, None) # node = (depth, parent, state, action)

        self.open_list.append(node)

        heapq.heapify(self.open_list)

        node = heapq.heappop(self.open_list)

        while(not self.problem.isgoal(node[2])):
            total_actions =  self.problem.actions(node[2])
            for i in total_actions:
                child_i = self.child_node(node, i)
                heapq.heappush(self.open_list, child_i)
            node = heapq.heappop(self.open_list)


        return node

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
    initial = puzzle.get_shuffled_state(1000000)
    solver = AStar(puzzle)
    goal = solver.search(initial)
    print("Initial State", initial)
    solver.solution(goal)
    print("Found goal")
    for sa in solver.solution(goal):
        puzzle.display(sa)