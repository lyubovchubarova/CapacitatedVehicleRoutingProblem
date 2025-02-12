from dataclasses import dataclass
from solution_methods.utils import euc_2d, read_file, Solution
import pathlib
from collections import defaultdict
from sklearn.metrics import mean_absolute_percentage_error
import matplotlib.pyplot as plt
import numpy as np


@dataclass
class Truck:
    idx: int
    max_capacity: int  # максимальная грузоподъемность
    current_capacity: int  # текущая оставшаяся грузоподъемность
    visited_nodes: list
    cost: float = 0.0

    def reset_capacity(self):
        self.current_capacity = self.max_capacity

class GreedyCVRPSolution(Solution):

    @staticmethod
    def create_distance_matrix(nodes):
        matrix = [[0.0] * len(nodes) for _ in range(len(nodes))]
        for i in range(len(nodes)):
            for j in range(len(nodes)):
                matrix[i][j] = euc_2d(nodes[i+1], nodes[j+1])
        return matrix

    def __init__(self, k, capacity, nodes, depot_idx):
        self.nodes = nodes
        for node in self.nodes.values():
            node.visited = False
        self.distance_matrix = self.create_distance_matrix(nodes)
        self.depot = depot_idx
        self.k = k
        self.truck_capacity = capacity
        self.trucks = [Truck(idx=idx,
                             max_capacity=capacity,
                             current_capacity=capacity,
                             visited_nodes=[self.depot])
                       for idx in range(k)]

    def find_best_node(self, truck):
        current_node = truck.visited_nodes[-1]
        best_node = None
        min_distance = float('inf')
        for node in self.nodes:
            node_obj = self.nodes[node]
            if not node_obj.visited and node_obj.demand > 0:
                if node_obj.demand <= truck.current_capacity:
                    distance = self.distance_matrix[current_node - 1][node - 1]
                    if distance < min_distance:
                        min_distance = distance
                        best_node = node
        return best_node

    def check_solution_valid(self):
        for node in self.nodes.values():
            if not node.visited and node.demand > 0:
                return False
        return True

    def solution(self):
        unvisited_nodes = [node for node in self.nodes if not self.nodes[node].visited and self.nodes[node].demand > 0]
        
        for truck in self.trucks:
            while True:
                next_node = self.find_best_node(truck)
                if next_node is None:
                    # возврат в депо, если узлов больше нет
                    if truck.visited_nodes[-1] != self.depot:
                        truck.cost += self.distance_matrix[truck.visited_nodes[-1] - 1][self.depot - 1]
                        truck.visited_nodes.append(self.depot)
                    break
                
                # добавляем узел к маршруту
                distance = self.distance_matrix[truck.visited_nodes[-1] - 1][next_node - 1]
                truck.cost += distance
                truck.visited_nodes.append(next_node)
                truck.current_capacity -= self.nodes[next_node].demand
                self.nodes[next_node].visited = True
                
                # проверяем, осталась ли грузоподъемность
                if truck.current_capacity <= 0:
                    # возвращаемся в депо
                    truck.cost += self.distance_matrix[next_node - 1][self.depot - 1]
                    truck.visited_nodes.append(self.depot)
                    truck.reset_capacity()
        
        # проверяем оставшиеся узлы
        unvisited = [node for node in self.nodes if not self.nodes[node].visited and self.nodes[node].demand > 0]
        for node in unvisited:
            for truck in self.trucks:
                if self.nodes[node].demand <= truck.max_capacity:
                    # начинаем новый маршрут с депо
                    if truck.visited_nodes[-1] != self.depot:
                        truck.cost += self.distance_matrix[truck.visited_nodes[-1] - 1][self.depot - 1]
                        truck.visited_nodes.append(self.depot)
                    truck.reset_capacity()
                    distance = self.distance_matrix[self.depot - 1][node - 1]
                    truck.cost += distance
                    truck.visited_nodes.append(node)
                    truck.current_capacity -= self.nodes[node].demand
                    self.nodes[node].visited = True
                    truck.cost += self.distance_matrix[node - 1][self.depot - 1]
                    truck.visited_nodes.append(self.depot)
                    break

        if not self.check_solution_valid():
            return float('inf')
        
        total_cost = sum(truck.cost for truck in self.trucks)
        return total_cost
    