"""
This code is adapted from search.py in the AIMA Python implementation, which is published with the license below:

	The MIT License (MIT)

	Copyright (c) 2016 aima-python contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Search (Chapters 3-4)

The way to use this code is to subclass Problem to create a class of problems,
then create problem instances and solve them with calls to the various search
functions."""

import sys
import os
import heapq
from subway import build_boston_map, build_london_map, straight_line_distance

#______________________________________________________________________________

'''DO NOT MODIFY THIS CLASS'''

class Problem:
	"""The abstract class for a formal problem.  You should subclass this and
	implement the method successor, and possibly __init__, goal_test, and
	path_cost. Then you will create instances of your subclass and solve them
	with the various search functions."""
	
	

	def __init__(self, initial, goal=None):
		"""The constructor specifies the initial state, and possibly a goal
		state, if there is a unique goal.  Your subclass's constructor can add
		other arguments."""
		self.initial = initial; self.goal = goal
		
	def successor(self, state):
		"""Given a state, return a sequence of (action, state) pairs reachable
		from this state. If there are many successors, consider an iterator
		that yields the successors one at a time, rather than building them
		all at once. Iterators will work fine within the framework."""
		raise NotImplementedError("successor() must be implemented in subclass")
	
	def goal_test(self, state):
		"""Return True if the state is a goal. The default method compares the
		state to self.goal, as specified in the constructor. Implement this
		method if checking against a single self.goal is not enough."""
		return state == self.goal
	
	def path_cost(self, c, state1, action, state2):
		"""Return the cost of a solution path that arrives at state2 from
		state1 via action, assuming cost c to get up to state1. If the problem
		is such that the path doesn't matter, this function will only look at
		state2.  If the path does matter, it will consider c and maybe state1
		and action. The default method costs 1 for every step in the path."""
		return c + 1
		
	def h(self, node):
		"""Return the heuristic function value for a particular node. Implement
		this if using informed (heuristic) search."""
		return 0

class SubwayProblem(Problem):
	def __init__(self, subway_map,start_station, goal_station, distance_threshold=0.0):
		super().__init__(start_station,goal_station)
		self.subway_map = subway_map
		self.distance_threshold = distance_threshold
	
	def successor(self, state):
		successors = []
		for link in self.subway_map.incident_links(state):
			next_station = link.opposite(state)
			successors.append((link,next_station))
		return successors
	
	def path_cost(self, c, state1, action, state2):
		return c + action.get_distance()

	def h(self, node):
		return straight_line_distance(node.state, self.goal)

	def goal_test(self, state):
		return straight_line_distance(state, self.goal) <= self.distance_threshold


class EightPuzzleProblem(Problem):
	def __init__(self, initial_state, goal_state='012345678'):
		super().__init__(initial_state, goal_state)

	def successor(self, state):
		zero_index = state.index('0')
		row = zero_index // 3
		col = zero_index % 3
		successors = []

		def swap_positions(i, j):
			cells = list(state)
			cells[i], cells[j] = cells[j], cells[i]
			return ''.join(cells)

		# Required action order: up, left, down, right
		if row > 0:
			next_state = swap_positions(zero_index, zero_index - 3)
			successors.append(('up', next_state))
		if col > 0:
			next_state = swap_positions(zero_index, zero_index - 1)
			successors.append(('left', next_state))
		if row < 2:
			next_state = swap_positions(zero_index, zero_index + 3)
			successors.append(('down', next_state))
		if col < 2:
			next_state = swap_positions(zero_index, zero_index + 1)
			successors.append(('right', next_state))

		return successors

	def h(self, node):
		distance = 0
		for tile in '12345678':
			current_index = node.state.index(tile)
			goal_index = self.goal.index(tile)
			current_row, current_col = divmod(current_index, 3)
			goal_row, goal_col = divmod(goal_index, 3)
			distance += abs(current_row - goal_row) + abs(current_col - goal_col)
		return distance
#______________________________________________________________________________
	
'''DO NOT MODIFY THIS CLASS'''

class Node:
	"""A node in a search tree. Contains a pointer to the parent (the node
	that this is a successor of) and to the actual state for this node. Note
	that if a state is arrived at by two paths, then there are two nodes with
	the same state.  Also includes the action that got us to this state, and
	the total path_cost (also known as g) to reach the node.  Other functions
	may add an f and h value. You will not need to
	subclass this class."""

	__nextID = 1

	def __init__(self, state, parent=None, action=None, path_cost=0):
		"Create a search tree Node, derived from a parent by an action."
		self.state = state
		self.parent = parent
		self.action = action
		self.path_cost = path_cost
		self.depth = 0
		self.id = Node.__nextID
		Node.__nextID += 1
		
		if parent:
			self.depth = parent.depth + 1
			
	def __str__(self):
		return "<Node " + str(self.state) + ">"
	
	def __repr__(self):
		return "<Node " + str(self.state) + ">"	
	
	def path(self):
		"Create a list of nodes from the root to this node."
		x, result = self, [self]
		while x.parent:
			result.append(x.parent)
			x = x.parent
		return result[::-1]

	def expand(self, problem):
		"Return a list of nodes reachable from this node. [Fig. 3.8]"
		return [Node(next, self, act,
					 problem.path_cost(self.path_cost, self.state, act, next))
				for (act, next) in problem.successor(self.state)]
	
	def __eq__(self, other):
		if isinstance(other, Node):
			return self.id == other.id
		return False
	
	def __lt__(self, other):
		if isinstance(other, Node):
			return self.id < other.id
		raise TypeError("\'<\' not supported between instances of Node and "+str(type(other)))
	
	def __hash__(self):
		return hash(self.id)

#______________________________________________________________________________
## Uninformed Search algorithms

'''DO NOT MODIFY THE HEADERS OF ANY OF THESE FUNCTIONS'''

def breadth_first_search(problem):
	root = Node(problem.initial)
	if problem.goal_test(root.state):
		problem.visited_count = 0
		return root

	frontier = [root]
	frontier_states = {root.state}
	visited_states = set()
	problem.visited_count = 0

	while frontier:
		current = frontier.pop(0)
		frontier_states.discard(current.state)
		visited_states.add(current.state)
		problem.visited_count += 1

		for child in current.expand(problem):
			if child.state in visited_states or child.state in frontier_states:
				continue
			if problem.goal_test(child.state):
				return child
			frontier.append(child)
			frontier_states.add(child.state)

	return None
	
def depth_first_search(problem):
	frontier = [Node(problem.initial)]
	frontier_states = {problem.initial}
	visited_states = set()
	problem.visited_count = 0

	while frontier:
		current = frontier.pop()
		frontier_states.discard(current.state)

		if current.state in visited_states:
			continue

		visited_states.add(current.state)
		problem.visited_count += 1

		if problem.goal_test(current.state):
			return current

		children = current.expand(problem)
		for child in reversed(children):
			if child.state not in visited_states and child.state not in frontier_states:
				frontier.append(child)
				frontier_states.add(child.state)

	return None

def uniform_cost_search(problem):
	root = Node(problem.initial)
	frontier = []
	heapq.heappush(frontier, (root.path_cost, root.id, root))
	best_frontier_cost = {root.state: 0}
	visited_states = set()
	problem.visited_count = 0

	while frontier:
		_, _, current = heapq.heappop(frontier)

		if current.state in visited_states:
			continue
		if current.path_cost > best_frontier_cost.get(current.state, float('inf')):
			continue

		visited_states.add(current.state)
		problem.visited_count += 1

		if problem.goal_test(current.state):
			return current

		for child in current.expand(problem):
			if child.state in visited_states:
				continue
			if child.path_cost < best_frontier_cost.get(child.state, float('inf')):
				best_frontier_cost[child.state] = child.path_cost
				heapq.heappush(frontier, (child.path_cost, child.id, child))

	return None
#______________________________________________________________________________
# Informed (Heuristic) Search

def astar_search(problem):
	root = Node(problem.initial)
	frontier = []
	root_f = root.path_cost + problem.h(root)
	heapq.heappush(frontier, (root_f, root.id, root))
	best_frontier_g = {root.state: 0}
	visited_states = set()
	problem.visited_count = 0

	while frontier:
		_, _, current = heapq.heappop(frontier)

		if current.state in visited_states:
			continue
		if current.path_cost > best_frontier_g.get(current.state, float('inf')):
			continue

		visited_states.add(current.state)
		problem.visited_count += 1

		if problem.goal_test(current.state):
			return current

		for child in current.expand(problem):
			if child.state in visited_states:
				continue
			if child.path_cost < best_frontier_g.get(child.state, float('inf')):
				best_frontier_g[child.state] = child.path_cost
				child_f = child.path_cost + problem.h(child)
				heapq.heappush(frontier, (child_f, child.id, child))

	return None

#______________________________________________________________________________
## Main

def main():
	os.chdir(os.path.dirname(os.path.abspath(__file__)))

	if len(sys.argv) < 4:
		print('Usage: python search.py <city|eight> <algorithm> <args...>')
		return

	city = sys.argv[1].lower()
	algorithm = sys.argv[2].lower()
	if city == 'eight':
		if len(sys.argv) not in (4, 5):
			print('Usage: python search.py eight <algorithm> <initial_state> [goal_state]')
			return
		initial_state = sys.argv[3]
		goal_state = '012345678' if len(sys.argv) == 4 else sys.argv[4]
		valid_tiles = set('012345678')

		if len(initial_state) != 9 or set(initial_state) != valid_tiles:
			print('Initial state must be a permutation of 012345678.')
			return
		if len(goal_state) != 9 or set(goal_state) != valid_tiles:
			print('Goal state must be a permutation of 012345678.')
			return

		problem = EightPuzzleProblem(initial_state, goal_state)
	elif city in ('boston', 'london'):
		if len(sys.argv) not in (5, 6):
			print('Usage: python search.py <boston|london> <algorithm> <start_station> <goal_station> [distance_km]')
			return
		start_name = sys.argv[3]
		goal_name = sys.argv[4]
		distance_threshold = 0.0

		if len(sys.argv) == 6:
			try:
				distance_threshold = float(sys.argv[5])
			except ValueError:
				print(f"Distance must be a number in kilometers: {sys.argv[5]}")
				return
			if distance_threshold < 0:
				print("Distance must be >= 0.")
				return

		subway_map = build_boston_map() if city == 'boston' else build_london_map()

		start_station = subway_map.get_station_by_name(start_name)
		goal_station = subway_map.get_station_by_name(goal_name)

		if start_station is None:
			print(f"Start station not found: {start_name}")
			return
		if goal_station is None:
			print(f"Goal station not found: {goal_name}")
			return

		problem = SubwayProblem(subway_map, start_station, goal_station, distance_threshold)
	else:
		print("First argument must be 'boston', 'london', or 'eight'.")
		return

	if algorithm == 'dfs':
		goal_node = depth_first_search(problem)
	elif algorithm == 'bfs':
		goal_node = breadth_first_search(problem)
	elif algorithm == 'ucs':
		goal_node = uniform_cost_search(problem)
	elif algorithm == 'astar':
		goal_node = astar_search(problem)
	else:
		print("Supported algorithms right now: dfs, bfs, ucs, astar")
		return

	visited_count = getattr(problem, 'visited_count', 0)

	if goal_node is None:
		print("No path found.")
		print(f"Search nodes visited: {visited_count}")
		return

	path_states = []
	for node in goal_node.path():
		if hasattr(node.state, 'name'):
			path_states.append(node.state.name)
		else:
			path_states.append(str(node.state))
	print('Path:', ' -> '.join(path_states))
	print(f'Total cost: {goal_node.path_cost:.3f}')
	print(f'Search nodes visited: {visited_count}')


if __name__ == '__main__':
	main()
