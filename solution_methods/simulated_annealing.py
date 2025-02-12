import math
import random
from solution_methods.utils import Solution

class SimulatedAnnealing(Solution):
    def __init__(self, nodes, depot_idx, initial_temp=1000, cooling_rate=0.999):
        self.nodes = nodes
        self.depot_idx = depot_idx 
        self.initial_temp = initial_temp
        self.cooling_rate = cooling_rate

    def distance(self, node1, node2):
        return math.dist((node1.x, node1.y), (node2.x, node2.y))

    def calculate_cost(self, route):
        cost = 0
        for i in range(len(route) - 1):
            cost += self.distance(self.nodes[route[i]], self.nodes[route[i + 1]])
        return cost

    def solution(self, group):
        group_without_depot = [node for node in group if node != self.depot_idx]
        current_route = [self.depot_idx] + random.sample(group_without_depot, len(group_without_depot)) + [self.depot_idx]
        current_cost = self.calculate_cost(current_route)

        best_route = current_route[:]
        best_cost = current_cost

        temperature = self.initial_temp

        if len(current_route) <= 3:
            return current_route

        while temperature > 1:
            new_route = current_route[:]
            i, j = random.sample(range(1, len(new_route) - 1), 2)
            new_route[i], new_route[j] = new_route[j], new_route[i]

            new_cost = self.calculate_cost(new_route)

            if new_cost < current_cost or random.random() < math.exp((current_cost - new_cost) / temperature):
                current_route = new_route[:]
                current_cost = new_cost

                if current_cost < best_cost:
                    best_route = current_route[:]
                    best_cost = current_cost
            temperature *= self.cooling_rate

        return best_route