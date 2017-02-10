from search import *
from nn_puzzle import *

class AStar(Search):

    def search(self, initial_state):
        """Given an initial problem state, encode a search-tree
        node representing the state and systematically explore
        the state-space until a goal-state is found.

        Returns:
         a search-tree node representing a goal state (if found)
         or None, if no goal is discovered"""

        return None

    def solution(self, node):
        """Returns the 'solution' for the specified node in the search tree.
        That is, this method should return a sequence of tuples:
        [(state_0, action_0), (state_1, action_1), ..., (state_n, action_n)]

        such that:
        state_0 is the initial state in the search
        action_0 is the first action taken
        action_i is the action taken to transition from state_i to state_{i+1}
        state_n is the state encapsulated by the 'search_node' argument
        action_n is None
        """
        raise ValueError("This is unimplemented.")

    def child_node(self, node, action):
        """Create a child node for this search tree given a parent
        search tree node and an action to execute. Don't confuse nodes
        in the search-tree and verticies in the state-space graph."""
        return None

    def parent(self, node):
        """Return the parent of the specified node in the search tree"""
        return None

    def depth(self, node):


    	
        """Determine how deep the search tree node is in the search tree.
        Consider the initial state (root) to be at depth 0

        Note: this method is NOT required by the Search class interface. But
        if you wanted to implement BFS or DFS or a variant, you'd likely
        want such a function.
        """

        return -1
        
class BFS(Search):

    def search(self, initial_state):
        """Given an initial problem state, encode a search-tree
        node representing the state and systematically explore
        the state-space until a goal-state is found.

        Returns:
         a search-tree node representing a goal state (if found)
         or None, if no goal is discovered"""

        return None

    def solution(self, node):
        """Returns the 'solution' for the specified node in the search tree.
        That is, this method should return a sequence of tuples:
        [(state_0, action_0), (state_1, action_1), ..., (state_n, action_n)]

        such that:
        state_0 is the initial state in the search
        action_0 is the first action taken
        action_i is the action taken to transition from state_i to state_{i+1}
        state_n is the state encapsulated by the 'search_node' argument
        action_n is None
        """
        raise ValueError("This is unimplemented.")

    def child_node(self, node, action):
        """Create a child node for this search tree given a parent
        search tree node and an action to execute. Don't confuse nodes
        in the search-tree and verticies in the state-space graph."""
        return None

    def parent(self, node):
        """Return the parent of the specified node in the search tree"""
        return None

    def depth(self, node):


    	
        """Determine how deep the search tree node is in the search tree.
        Consider the initial state (root) to be at depth 0

        Note: this method is NOT required by the Search class interface. But
        if you wanted to implement BFS or DFS or a variant, you'd likely
        want such a function.
        """

        return -1
if __name__ == "__main__":

    puzzle = NNPuzzle(3)
    initial = puzzle.get_shuffled_state(2)
    solver = AStar(puzzle)
    goal = solver.search(initial)
    print("Initial State", initial)
    print("Found goal")
    for sa in solver.solution(goal):
        puzzle.display(sa)
