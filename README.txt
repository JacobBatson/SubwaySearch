SubwaySearch
============

Overview
--------
This project implements general search algorithms and applies them to two domains:

1. Subway navigation (Boston MBTA and London Underground)
2. 8-puzzle state-space search

Implemented algorithms:
- DFS (graph search)
- BFS (graph search)
- UCS (uniform-cost search)
- A* (with problem-specific heuristic)

Project Structure
-----------------
- `python/search.py`: problem definitions, search algorithms, and command-line runner
- `python/subway.py`: subway graph classes and map-building utilities
- `data/`: CSV data files for Boston and London stations/links

Requirements
------------
- Python 3.x
- No external packages required (standard library only)

How to Run
----------
Run from the repository root:

`python3 python/search.py <problem> <algorithm> <args...>`

Algorithms:
- `dfs`
- `bfs`
- `ucs`
- `astar`

Subway Problem
--------------
Arguments:

`python3 python/search.py <boston|london> <algorithm> <start_station> <goal_station> [distance_km]`

Notes:
- If station names contain spaces, wrap them in quotes.
- Optional `distance_km` defines a goal radius around the named goal station.
- If `distance_km` is omitted, it defaults to `0.0` (exact goal station).

Examples:
- `python3 python/search.py boston dfs Fenway "South Station"`
- `python3 python/search.py london astar "Piccadilly Circus" Wimbledon`
- `python3 python/search.py boston astar Riverside "Cleveland Circle" 0.25`

Subway heuristic (A*):
- Straight-line distance (SLD) from current station to the named goal station.

8-Puzzle Problem
----------------
Arguments:

`python3 python/search.py eight <algorithm> <initial_state> [goal_state]`

State format:
- 9-character string using digits `0..8`
- `0` represents the blank tile
- Must be a permutation of `012345678`

Default goal state:
- `012345678`

Examples:
- `python3 python/search.py eight bfs 142305678`
- `python3 python/search.py eight astar 142305678`
- `python3 python/search.py eight astar 142305678 012345678`

8-puzzle heuristic (A*):
- Manhattan distance (sum for tiles 1..8)

Output
------
Each run prints:
- `Path: ...`
- `Total cost: ...`
- `Search nodes visited: ...`

Node-visit counting rule:
- A node is counted as visited when it is removed from the frontier.

Notes
-----
- Search code is domain-independent through the `Problem` abstraction.
- `SubwayProblem` and `EightPuzzleProblem` provide domain-specific
  `successor`, `goal_test`, `path_cost`, and `h` behavior.
