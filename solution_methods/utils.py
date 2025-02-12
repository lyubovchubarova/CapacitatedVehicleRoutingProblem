from dataclasses import dataclass
import math
import abc

@dataclass
class Node:
    idx: int
    x: int
    y: int
    demand: int = -1

def read_file(filename):

    with open(filename) as f:

        nodes = {}

        state = None

        for line in f:
            line = line.strip()

            if not line:
                continue

            # number of trucks
            if line.startswith("NAME"):
                state = "NAME"
                value = line.split(": ", 1)[1]
                k = int(value.split("-")[-1].replace("k", ""))

            # capacity of one truck
            elif line.startswith("CAPACITY"):
                state = "CAPACITY"
                capacity = int(line.split(": ", 1)[1])

            elif line.startswith("NODE_COORD_SECTION"):
                state = "NODE_COORD_SECTION"

            elif line.startswith("DEMAND_SECTION"):
                state = "DEMAND_SECTION"

            elif line.startswith("DEPOT_SECTION"):
                state = "DEPOT_SECTION"

            elif line.startswith("EOF"):
                break

            else:
                if state == "NODE_COORD_SECTION":
                    idx, x, y = line.split(" ")
                    nodes[int(idx)] = Node(
                                        idx = int(idx),
                                        x = int(x),
                                        y = int(y),
                                    )

                elif state == "DEMAND_SECTION":
                    idx, demand = line.split(" ")
                    nodes[int(idx)].demand = int(demand)

                elif state == "DEPOT_SECTION":
                    idx = int(line)
                    if idx == -1:
                        continue
                    depot_idx = idx

    return k, capacity, nodes, depot_idx


def euc_2d(node_from: Node,
           node_to: Node):
    return math.sqrt((node_from.x - node_to.x)**2 
                     + (node_from.y - node_to.y)**2)

class Solution(abc.ABC):

    @abc.abstractmethod
    def solution(self): pass

            
# if __name__ == "__main__":
#     filename = "/Users/liubovchubaroba/Downloads/A/A-n32-k5.vrp"
#     k, capacity, nodes, depots_idxs = read_file(filename)
#     print(k) 
#     print(capacity)
#     print(nodes)
#     print(depots_idxs)

