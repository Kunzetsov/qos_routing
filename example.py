import copy
import pprint


def find_optimal_path(src, dst, g):
    # graph = self.graph_to_matrix(self.reversed_optimality_graph)
    graph = g
    distance, predecessor = dict(), dict()
    for node in g:
        distance[node], predecessor[node] = float('inf'), None
    distance[src] = 0

    for _ in range(len(graph) - 1):
        for node in graph:
            for neighbour in graph[node]:
                if graph[node][neighbour] != float('Inf'):
                    # print('neighbour -> %s , node -> %s, value -> %s' % (neighbour, node, graph[node][neighbour]))
                    if distance[neighbour] > distance[node] + graph[node][neighbour]:
                        print('маршрутизатор %s, сусідній маршрутизатор %s, попередня вага = %s, нова вага = %s ' % (neighbour, node, distance[neighbour], distance[node] + graph[node][neighbour]))
                        distance[neighbour] = distance[node] + graph[node][neighbour]
                        predecessor[neighbour] = node
                        print('найоптимальніший шлях з маршрутизатора %s в маршрутизатор %s ' % (neighbour, node))
        # Step 3: Check for negative weight cycles
    for node in graph:
        for neighbour in graph[node]:
            assert distance[neighbour] <= distance[node] + graph[node][neighbour], "Negative weight cycle."
    print(distance)
    print(predecessor)

    current = dst
    path = []
    while current is not None:
        path.append(current)
        if predecessor.get(current) is None:
            break
        current = predecessor[current]
    print(path)
    return list(reversed(path))


def local_convolution(z1, z2, z3):
    a1 = 0.33
    a2 = -0.33
    a3 = -0.33
    if z1 is None: z1 = 0
    if z2 is None: z2 = 0
    if z3 is None: z3 = 0
    return a1 * z1 + a2 * z2 + a3 * z3


def local_optimal_criteria(x, x_max):
    print('local_optimal_criteria')
    if x_max == 0: return 0
    return x / x_max


def local_reversed_convolution(w, w_max):
    return w_max - w + 1


def calc_convolutions(graph1, graph2, graph3):
    optimality_graph = {}
    for src, dst_list in graph1.items():
        if optimality_graph.get(src) is None:
            optimality_graph.setdefault(src, {})

        for dst, val in dst_list.items():
            if graph1.get(src).get(dst) == 0 and graph2.get(src).get(dst) == 0 \
                    and graph3.get(src).get(dst) == 0:
                optimality_graph[src][dst] = -float('Inf')
            else:
                optimality_graph[src][dst] = \
                    local_convolution(
                        graph1.get(src).get(dst),
                        graph2.get(src).get(dst),
                        graph3.get(src).get(dst)
                    )

    print('calc_convolutions')
    return optimality_graph


def find_max(graph):
    max_val = None
    for src, dist_list in graph.items():
        curr_max_link = max(dist_list, key=lambda dst: dist_list[dst])
        curr_max_val = dist_list[curr_max_link]
        if max_val is None or max_val < curr_max_val:
            max_val = curr_max_val
    return max_val


def calc_reversed_convolutions(optimality_conv):
    rev_conv = {}
    max_conv = find_max(optimality_conv)
    print(max_conv)
    for src, dst_list in optimality_conv.items():
        if rev_conv.get(src) is None:
            rev_conv.setdefault(src, {})
        for dst, val in dst_list.items():
            if optimality_conv[src][dst] == -float('Inf'):
                rev_conv[src][dst] = float('Inf')
            else:
                rev_conv[src][dst] = local_reversed_convolution(optimality_conv[src][dst], max_conv)
    # self.max_convolution = max_conv

    return rev_conv


graph1 = {
    1: {1: 0, 2: 0.85714, 3: 0, 4: 1.0, 5: 0, 6: 0, 7: 0, 8: 0, 9: 0},
    2: {1: 0.85714, 2: 0, 3: 0.57142, 4: 0, 5: 0.57142, 6: 0, 7: 0, 8: 0, 9: 0},
    3: {1: 0, 2: 0.57142, 3: 0, 4: 0, 5: 0, 6: 0.85714, 7: 0, 8: 0, 9: 0},
    4: {1: 1.0, 2: 0, 3: 0, 4: 0, 5: 0.71428, 6: 0, 7: 0.62857, 8: 0, 9: 0},
    5: {1: 0, 2: 0.91428, 3: 0, 4: 0.71428, 5: 0, 6: 0.91428, 7: 0, 8: 1.0, 9: 0},
    6: {1: 0, 2: 0, 3: 0.85714, 4: 0, 5: 0.91428, 6: 0, 7: 0, 8: 0, 9: 0.68571},
    7: {1: 0, 2: 0, 3: 0, 4: 0.62857, 5: 0, 6: 0, 7: 0, 8: 0.88571, 9: 0},
    8: {1: 0, 2: 0, 3: 0, 4: 0, 5: 1.0, 6: 0, 7: 0.88571, 8: 0, 9: 0.8},
    9: {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0.68571, 7: 0, 8: 0.8, 9: 0}
}

graph2 = {
    1: {1: 0, 2: 0.83333, 3: 0, 4: 0.91666, 5: 0, 6: 0, 7: 0, 8: 0, 9: 0},
    2: {1: 0.83333, 2: 0, 3: 0.41666, 4: 0, 5: 0.75, 6: 0, 7: 0, 8: 0, 9: 0},
    3: {1: 0, 2: 0.41666, 3: 0, 4: 0, 5: 0, 6: 0.64583, 7: 0, 8: 0, 9: 0},
    4: {1: 0.91666, 2: 0, 3: 0, 4: 0, 5: 0.4166, 6: 0, 7: 1.0, 8: 0, 9: 0},
    5: {1: 0, 2: 0.75, 3: 0, 4: 0.4166, 5: 0, 6: 0.8333, 7: 0, 8: 0.79166, 9: 0},
    6: {1: 0, 2: 0, 3: 0.64583, 4: 0, 5: 0.8333, 6: 0, 7: 0, 8: 0, 9: 0.625},
    7: {1: 0, 2: 0, 3: 0, 4: 1.0, 5: 0, 6: 0, 7: 0, 8: 0.9166, 9: 0},
    8: {1: 0, 2: 0, 3: 0, 4: 0, 5: 0.79166, 6: 0, 7: 0.9166, 8: 0, 9: 0.5},
    9: {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0.625, 7: 0, 8: 0.5, 9: 0}
}
graph3 = {
    1: {1: 0, 2: 0.2941, 3: 0, 4: 0.35294, 5: 0, 6: 0, 7: 0, 8: 0, 9: 0},
    2: {1: 0.2941, 2: 0, 3: 0.4411, 4: 0, 5: 0.22058, 6: 0, 7: 0, 8: 0, 9: 0},
    3: {1: 0, 2: 0.4411, 3: 0, 4: 0, 5: 0, 6: 0.20588, 7: 0, 8: 0, 9: 0},
    4: {1: 0.35294, 2: 0, 3: 0, 4: 0, 5: 0.44117, 6: 0, 7: 0.35294, 8: 0, 9: 0},
    5: {1: 0, 2: 0.22058, 3: 0, 4: 0.44117, 5: 0, 6: 0.29411, 7: 0, 8: 0.3529, 9: 0},
    6: {1: 0, 2: 0, 3: 0.20588, 4: 0, 5: 0.29411, 6: 0, 7: 0, 8: 0, 9: 0.5882},
    7: {1: 0, 2: 0, 3: 0, 4: 0.35294, 5: 0, 6: 0, 7: 0, 8: 0.3529, 9: 0},
    8: {1: 0, 2: 0, 3: 0, 4: 0, 5: 0.3529, 6: 0, 7: 0.3529, 8: 0, 9: 0.3529},
    9: {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0.5882, 7: 0, 8: 1.0, 9: 0}
}

conv = calc_convolutions(graph1, graph2, graph3)

rev_conv = calc_reversed_convolutions(conv)

pprint.pprint(find_optimal_path(1, 9, rev_conv))
# print(calc_convolutions(graph1, graph2, graph3))
