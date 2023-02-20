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

if __name__ == '__main__':
    map_path = input('Enter path to map CSV: ')
    out_path = input('Enter path to save gif to: ')
    map_ = utils.load_map(map_path)
    graph, start, end = utils.construct_graph(map_)
    pr = astar(graph, start, end)
    path = utils.pr_to_path(pr, start, end)
    utils.animate(map_, pr, path, f'A* {map_path}', out_path)
