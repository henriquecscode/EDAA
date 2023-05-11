import os
import sys
import networkx as nx

G=nx.Graph()

BASE_DIR = "./data/solutions/"
solution_file = sys.argv[1]
filepath = BASE_DIR + solution_file
edges = []
with open(os.path.expanduser(filepath), "r") as f:
    number_of_edges = int(f.readline())
    for _ in range(number_of_edges):
        line = f.readline().split(",")
        line = [value.strip() for value in line]
        G.add_edge(line[0], line[1])

node_to_cycles = {}
for source in G.nodes():
    paths = []
    for target in G.neighbors(source):
        paths += [l + [source] for l in list(nx.all_simple_paths(G, source=source, target=target)) if len(l) > 2]
    node_to_cycles[source] = paths

print(node_to_cycles)