import heapq
import utils

def astar(graph, start, end):
    heuristic = lambda x: abs(end[0] - x[0]) + abs(end[1] - x[1])
    visited = set()
    pr = {}
    q = [(heuristic(start), start)]
    gs = {start: (0, heuristic(start))}
    while len(q) > 0:
        (g, curr) = heapq.heappop(q)
        if curr == end:
            break
        if curr not in visited:
            visited.add(curr)
            for neighbor in graph[curr]:
                new_d = gs[curr][0] + 1
                new_g = new_d + heuristic(neighbor)
                if neighbor not in visited and (neighbor not in gs or new_g < sum(gs[neighbor])):
                    pr[neighbor] = curr
                    heapq.heappush(q, (new_g, neighbor))
                    gs[neighbor] = (new_d, heuristic(neighbor))
    return pr
